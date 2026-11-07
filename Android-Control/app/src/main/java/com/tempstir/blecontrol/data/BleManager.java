package com.tempstir.blecontrol.data;

import android.Manifest;
import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothProfile;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanResult;
import android.bluetooth.le.ScanSettings;
import android.content.Context;
import android.os.Handler;
import android.os.Looper;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.UUID;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import androidx.annotation.RequiresPermission;

/**
 * ECB01C BLE 透传通道管理（GATT 客户端）。
 *
 * 依据《通信协议.md》9.3 / 13-B：
 *   Service  FFF0
 *   Notify   FFF1  —— 必须先订阅 CCCD，否则模块不会把串口数据透传给手机
 *   Write    FFF2  —— WRITE_TYPE_NO_RESPONSE
 *   出厂名 BT_ECB1A6B01BA2（模块不支持改名），UUID 均为 16bit 蓝牙 SIG 基址扩展
 *
 * 约束：
 *   - Android BLE 栈同一时刻只允许一个未完成操作；本类用"单帧 outstanding"串行发送。
 *   - 芯片要求两次发送间隔 ≥100ms；这里统一留 120ms。
 *   - 手机 3 分钟无数据模块会主动断开，故连接后 1s 轮询心跳。
 */
public class BleManager {

    public static final UUID UUID_SERVICE = UUID.fromString("0000fff0-0000-1000-8000-00805f9b34fb");
    public static final UUID UUID_NOTIFY  = UUID.fromString("0000fff1-0000-1000-8000-00805f9b34fb");
    public static final UUID UUID_WRITE   = UUID.fromString("0000fff2-0000-1000-8000-00805f9b34fb");
    public static final UUID UUID_CCCD    = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");

    private static final long MIN_TX_GAP_MS   = 120L;  // 芯片要求 ≥100ms
    private static final long RESP_TIMEOUT_MS = 1200L;
    private static final long POLL_PERIOD_MS  = 1000L;
    private static final long SCAN_PERIOD_MS  = 12000L;
    private static final int  REQ_MTU         = 247;

    public interface Listener {
        void onStatus(String text);
        void onLog(String line);

        /** 完整设备列表（已在子线程构建完成，主线程只负责绑定）。 */
        void onDevices(List<DeviceItem> devices);

        void onConnected(String name, String address);
        void onDisconnected();
        void onRead(int startReg, int[] values);
        void onWriteAck();
        void onException(int code);
        void onMtu(int mtu);
    }

    /** 一次待发的 Modbus 请求（同时用来把应答对应回寄存器起始地址）。 */
    private static final class Request {
        final byte[] frame;
        final int startReg;
        final int count;
        final boolean isRead;
        final String label;

        Request(byte[] frame, int startReg, int count, boolean isRead, String label) {
            this.frame = frame;
            this.startReg = startReg;
            this.count = count;
            this.isRead = isRead;
            this.label = label;
        }
    }

    private final Context context;
    private final Listener listener;
    private final Handler main = new Handler(Looper.getMainLooper());

    private BluetoothAdapter adapter;
    private BluetoothLeScanner scanner;
    private BluetoothGatt gatt;
    private BluetoothGattCharacteristic notifyChar;
    private BluetoothGattCharacteristic writeChar;

    private final LinkedHashMap<String, BluetoothDevice> found = new LinkedHashMap<>();
    private final Object foundLock = new Object();          // 保护 found
    private final LinkedHashMap<String, Integer> rssiMap = new LinkedHashMap<>();
    private final ExecutorService worker = Executors.newSingleThreadExecutor(r -> {
        Thread t = new Thread(r, "ble-device-list");
        t.setDaemon(true);
        return t;
    });
    private boolean listBuildPending;                        // 受 foundLock 保护
    private final ArrayDeque<Request> queueHigh = new ArrayDeque<>();
    private final ArrayDeque<Request> queueLow = new ArrayDeque<>();
    private Request pending;
    private boolean gattWriteInFlight;
    private boolean ready;          // CCCD 已订阅，可收发
    private boolean scanning;
    private boolean serviceDiscoveryStarted;
    private long lastTxMs;
    private int mtu = 23;
    private int pollPhase;
    private int pollImuPhase;
    /** P9：对端固件版本（来自 0x000F），0 = 尚未读到；<0x0102 时不发 IMU 相位，避免每秒异常 02 */
    private int peerFwVersion;

    /* P9：IMU 区块 0x0020~0x0039 分 4 帧轮转（每帧 ≤7 个寄存器，MTU 23 也能容纳）
     * v1.3：追加第 5 帧 0x003A~0x003C（加热过流阈值 / 堵转辅助 / 实测统计），
     *       仅当对端固件 >= 0x0103 时才发，旧固件不会收到异常 02。 */
    private static final int[] IMU_PHASE_START = { 0x0020, 0x0027, 0x002E, 0x0035, 0x003A };
    private static final int[] IMU_PHASE_COUNT = { 7, 7, 7, 5, 3 };

    private byte[] rxBuf = new byte[512];
    private int rxLen;

    public BleManager(Context context, Listener listener) {
        this.context = context.getApplicationContext();
        this.listener = listener;
        BluetoothManager mgr = (BluetoothManager) this.context.getSystemService(Context.BLUETOOTH_SERVICE);
        if (mgr != null) {
            adapter = mgr.getAdapter();
        }
    }

    public boolean isBluetoothOn() {
        return adapter != null && adapter.isEnabled();
    }

    public boolean isReady() {
        return ready;
    }

    public int getMtu() {
        return mtu;
    }

    /* ------------------------------------------------------------------ 扫描 */

    @SuppressLint("MissingPermission")
    public boolean startScan() {
        if (!isBluetoothOn()) {
            listener.onStatus("蓝牙未开启，请先在系统设置里打开蓝牙");
            return false;
        }
        stopScanInternal(false);
        synchronized (foundLock) {
            found.clear();
            rssiMap.clear();
        }
        scanner = adapter.getBluetoothLeScanner();
        if (scanner == null) {
            listener.onStatus("无法获取 BLE 扫描器");
            return false;
        }
        ScanSettings settings = new ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                .build();
        scanning = true;
        scanner.startScan(null, settings, scanCallback);
        listener.onStatus("正在扫描 BLE 设备…（12 秒）");
        main.postDelayed(() -> {
            if (scanning) {
                stopScanInternal(true);
            }
        }, SCAN_PERIOD_MS);
        return true;
    }

    /** 主动结束扫描（例如关闭设备选择弹窗），不改变状态提示。 */
    public void stopScan() {
        stopScanInternal(false);
    }

    @SuppressLint("MissingPermission")
    private void stopScanInternal(boolean notify) {
        if (scanning && scanner != null) {
            scanner.stopScan(scanCallback);
        }
        scanning = false;
        if (notify) {
            boolean empty;
            synchronized (foundLock) {
                empty = found.isEmpty();
            }
            listener.onStatus(empty ? "扫描结束：未发现设备" : "扫描结束：选择设备连接");
        }
    }

    private final ScanCallback scanCallback = new ScanCallback() {
        @Override
        public void onScanResult(int callbackType, ScanResult result) {
            BluetoothDevice d = result.getDevice();
            if (d == null || d.getAddress() == null) {
                return;
            }
            synchronized (foundLock) {
                found.put(d.getAddress(), d);
                rssiMap.put(d.getAddress(), result.getRssi());
            }
            scheduleDeviceListBuild();
        }

        @Override
        public void onScanFailed(int errorCode) {
            main.post(() -> {
                scanning = false;
                listener.onStatus("扫描失败，错误码 " + errorCode + "（请确认蓝牙/定位已开启）");
            });
        }
    };

    /**
     * 合并 260ms 内的扫描回调后，在子线程解析设备名称并构建完整列表，
     * 构建完成后一次性回主线程刷新，避免主线程被 Binder 调用和频繁 notify 拖住。
     */
    private void scheduleDeviceListBuild() {
        synchronized (foundLock) {
            if (listBuildPending) {
                return;
            }
            listBuildPending = true;
        }
        main.postDelayed(() -> worker.execute(() -> {
            synchronized (foundLock) {
                listBuildPending = false;
            }
            List<DeviceItem> items = buildDeviceList();
            main.post(() -> listener.onDevices(items));
        }), 260L);
    }

    /** 必须在子线程执行：BluetoothDevice#getName 可能阻塞。 */
    @SuppressLint("MissingPermission")
    private List<DeviceItem> buildDeviceList() {
        List<BluetoothDevice> snapshot = new ArrayList<>();
        List<Integer> rssi = new ArrayList<>();
        synchronized (foundLock) {
            for (Map.Entry<String, BluetoothDevice> e : found.entrySet()) {
                snapshot.add(e.getValue());
                Integer r = rssiMap.get(e.getKey());
                rssi.add(r == null ? 0 : r);
            }
        }
        List<DeviceItem> out = new ArrayList<>(snapshot.size());
        for (int i = 0; i < snapshot.size(); i++) {
            BluetoothDevice d = snapshot.get(i);
            String name = null;
            try {
                name = d.getName();
            } catch (SecurityException ignored) {
            }
            if (name == null || name.trim().isEmpty()) {
                name = "(未广播名称)";
            }
            out.add(new DeviceItem(name, d.getAddress(), rssi.get(i), false, false));
        }
        // 信号强的排在前面，界面刷新时的顺序更稳定
        out.sort((a, b) -> Integer.compare(b.rssi, a.rssi));
        return out;
    }

    /* ------------------------------------------------------------------ 连接 */

    @SuppressLint("MissingPermission")
    public void connect(String address) {
        if (adapter == null) {
            return;
        }
        stopScanInternal(false);
        disconnect();
        BluetoothDevice device = adapter.getRemoteDevice(address);
        resetChannel();
        listener.onStatus("正在连接 " + address + " …");
        gatt = device.connectGatt(context, false, gattCallback, BluetoothDevice.TRANSPORT_LE);
    }

    @SuppressLint("MissingPermission")
    public void disconnect() {
        ready = false;
        main.removeCallbacks(pollRunnable);
        main.removeCallbacks(timeoutRunnable);
        main.removeCallbacks(pumpRunnable);
        if (gatt != null) {
            gatt.disconnect();
            gatt.close();
            gatt = null;
        }
    }

    private void resetChannel() {
        ready = false;
        notifyChar = null;
        writeChar = null;
        serviceDiscoveryStarted = false;
        mtu = 23;
        pollPhase = 0;
        pollImuPhase = 0;
        peerFwVersion = 0;   // P9：重新连接后等 0x000F 回来再决定是否轮询 IMU 区块
        pending = null;
        gattWriteInFlight = false;
        queueHigh.clear();
        queueLow.clear();
        rxLen = 0;
        lastTxMs = 0L;
    }

    private final BluetoothGattCallback gattCallback = new BluetoothGattCallback() {

        @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
        @Override
        public void onConnectionStateChange(BluetoothGatt g, int status, int newState) {
            main.post(() -> {
                if (status != BluetoothGatt.GATT_SUCCESS) {
                    listener.onStatus(String.format(Locale.CHINA, "连接失败 status=%d", status));
                    teardown(g);
                    return;
                }
                if (newState == BluetoothProfile.STATE_CONNECTED) {
                    listener.onStatus("已连接，正在协商 MTU / 发现服务…");
                    boolean ok = g.requestMtu(REQ_MTU);
                    // 部分芯片不回调 onMtuChanged，做一次兜底
                    main.postDelayed(() -> {
                        if (!serviceDiscoveryStarted && gatt == g) {
                            startDiscoverServices(g);
                        }
                    }, 1600L);
                    if (!ok) {
                        startDiscoverServices(g);
                    }
                } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                    teardown(g);
                }
            });
        }

        @Override
        public void onMtuChanged(BluetoothGatt g, int newMtu, int status) {
            main.post(() -> {
                mtu = status == BluetoothGatt.GATT_SUCCESS ? newMtu : 23;
                listener.onMtu(mtu);
                startDiscoverServices(g);
            });
        }

        @Override
        public void onServicesDiscovered(BluetoothGatt g, int status) {
            main.post(() -> {
                if (status != BluetoothGatt.GATT_SUCCESS) {
                    listener.onStatus("服务发现失败 status=" + status);
                    teardown(g);
                    return;
                }
                BluetoothGattService svc = g.getService(UUID_SERVICE);
                if (svc == null) {
                    listener.onStatus("未找到 FFF0 服务，请确认连接的是 TempStir 蓝牙模块");
                    teardown(g);
                    return;
                }
                notifyChar = svc.getCharacteristic(UUID_NOTIFY);
                writeChar = svc.getCharacteristic(UUID_WRITE);
                if (notifyChar == null || writeChar == null) {
                    listener.onStatus("FFF0 服务缺少 FFF1/FFF2 特征");
                    teardown(g);
                    return;
                }
                subscribeNotify(g);
            });
        }

        @Override
        public void onDescriptorWrite(BluetoothGatt g, BluetoothGattDescriptor d, int status) {
            main.post(() -> {
                if (status != BluetoothGatt.GATT_SUCCESS) {
                    listener.onStatus("订阅 FFF1 Notify 失败 status=" + status);
                    teardown(g);
                    return;
                }
                ready = true;
                String name = safeName(g.getDevice());
                listener.onConnected(name, g.getDevice().getAddress());
                listener.onStatus("FFF1 已订阅，通道就绪（MTU=" + mtu + "）");
                // 上电先读一次设备信息，然后启动心跳轮询
                enqueue(buildReadRequest(ModbusProtocol.REG_DEV_ID_LO, 3, "设备信息"), true);
                enqueue(buildReadRequest(0x0000, 7, "状态快照A"), true);
                enqueue(buildReadRequest(0x0007, 7, "状态快照B"), true);
                main.removeCallbacks(pollRunnable);
                main.post(pollRunnable);
                pump();
            });
        }

        @Override
        public void onCharacteristicWrite(BluetoothGatt g, BluetoothGattCharacteristic c, int status) {
            main.post(() -> {
                gattWriteInFlight = false;
                if (status != BluetoothGatt.GATT_SUCCESS) {
                    listener.onStatus("FFF2 写入失败 status=" + status);
                    pending = null;
                    main.removeCallbacks(timeoutRunnable);
                }
                schedulePump(MIN_TX_GAP_MS);
            });
        }

        /**
         * 【底层蓝牙回传入口 · API 33 以下】FFF1 Notify 收到数据时由 BLE 栈回调。
         * API 33+ 由下面的三参回调处理，这里直接返回避免重复解析。
         * 收到的就是下位机透传上来的 Modbus RTU 应答字节流（可能被拆成多个分片）。
         */
        @Override
        public void onCharacteristicChanged(BluetoothGatt g, BluetoothGattCharacteristic c) {
            if (android.os.Build.VERSION.SDK_INT >= 33) {
                return;
            }
            final byte[] chunk = c.getValue();
            main.post(() -> onNotifyBytes(chunk));
        }

        /**
         * 【底层蓝牙回传入口 · API 33/Android 13 及以上】Notify 数据直接以 byte[] 给出。
         * 同样交给 onNotifyBytes() 做拼包 + 帧解析，解析结果最终回调 listener.onRead()。
         */
        @Override
        public void onCharacteristicChanged(BluetoothGatt g, BluetoothGattCharacteristic c,
                                            byte[] value) {
            final byte[] chunk = value;
            main.post(() -> onNotifyBytes(chunk));
        }
    };

    @SuppressLint("MissingPermission")
    private void startDiscoverServices(BluetoothGatt g) {
        if (serviceDiscoveryStarted || gatt != g) {
            return;
        }
        serviceDiscoveryStarted = true;
        g.discoverServices();
    }

    @SuppressLint("MissingPermission")
    private void subscribeNotify(BluetoothGatt g) {
        g.setCharacteristicNotification(notifyChar, true);
        BluetoothGattDescriptor cccd = notifyChar.getDescriptor(UUID_CCCD);
        if (cccd == null) {
            listener.onStatus("FFF1 缺少 CCCD 描述符");
            teardown(g);
            return;
        }
        cccd.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
        if (!g.writeDescriptor(cccd)) {
            listener.onStatus("写 CCCD 失败");
            teardown(g);
        }
    }

    @SuppressLint("MissingPermission")
    private void teardown(BluetoothGatt g) {
        ready = false;
        main.removeCallbacks(pollRunnable);
        main.removeCallbacks(pumpRunnable);
        main.removeCallbacks(timeoutRunnable);
        resetChannel();
        if (g != null) {
            g.close();
        }
        if (gatt == g) {
            gatt = null;
        }
        listener.onDisconnected();
        listener.onStatus("已断开");
    }

    @SuppressLint("MissingPermission")
    private String safeName(BluetoothDevice d) {
        try {
            String n = d.getName();
            return n == null ? "(未知名称)" : n;
        } catch (SecurityException e) {
            return "(未知名称)";
        }
    }

    /* ------------------------------------------------------------------ 发送队列 */

    private Request buildReadRequest(int start, int count, String label) {
        return new Request(ModbusProtocol.buildRead(start, count), start, count, true, label);
    }

    private void enqueue(Request r, boolean high) {
        (high ? queueHigh : queueLow).addLast(r);
        pump();
    }

    /** 用户命令（高优先级）：写单个寄存器。 */
    public void sendWriteSingle(int reg, int value) {
        if (!ready) {
            listener.onStatus("尚未连接，无法下发命令");
            return;
        }
        enqueue(new Request(ModbusProtocol.buildWriteSingle(reg, value), reg, 1, false,
                String.format(Locale.CHINA, "写 0x%04X = %d", reg, value)), true);
    }

    /** 轮询读（低优先级）。 */
    public void sendRead(int start, int count, boolean high) {
        if (!ready) {
            return;
        }
        enqueue(buildReadRequest(start, count, "读表"), high);
    }

    private final Runnable pumpRunnable = new Runnable() {
        @Override
        public void run() {
            pump();
        }
    };

    private void schedulePump(long delayMs) {
        main.removeCallbacks(pumpRunnable);
        main.postDelayed(pumpRunnable, delayMs);
    }

    @SuppressLint("MissingPermission")
    private void pump() {
        if (!ready || gatt == null || writeChar == null || pending != null || gattWriteInFlight) {
            return;
        }
        long gap = MIN_TX_GAP_MS - (System.currentTimeMillis() - lastTxMs);
        if (gap > 0) {
            schedulePump(gap);
            return;
        }
        Request r = queueHigh.pollFirst();
        if (r == null) {
            r = queueLow.pollFirst();
        }
        if (r == null) {
            return;
        }
        pending = r;
        gattWriteInFlight = true;
        lastTxMs = System.currentTimeMillis();
        writeChar.setWriteType(BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE);
        writeChar.setValue(r.frame);
        boolean ok = gatt.writeCharacteristic(writeChar);
        if (!ok) {
            gattWriteInFlight = false;
            pending = null;
            listener.onStatus("GATT 写入被拒绝");
            schedulePump(200L);
            return;
        }
        listener.onLog("TX  " + r.label + "  " + ModbusProtocol.hex(r.frame));
        main.removeCallbacks(timeoutRunnable);
        main.postDelayed(timeoutRunnable, RESP_TIMEOUT_MS);
    }

    private final Runnable timeoutRunnable = new Runnable() {
        @Override
        public void run() {
            if (pending != null) {
                listener.onLog("!! 应答超时：" + pending.label);
                pending = null;
                gattWriteInFlight = false;
                pump();
            }
        }
    };

    /**
     * 【数据链起点】心跳轮询：连接就绪后每 1s 触发一次，把读请求压入低优先级队列。
     *   phase 0：0x0000~0x0006（液温/目标液温/转速/目标转速/剩余时间/状态字/故障码）
     *   phase 1：0x0007~0x000D（命令字/铝板温度/加热占空比/电机占空比/设定时间/风扇/设备ID低）
     *   phase 2：0x0014~0x0015（面板焦点、铝板限温，FW>=1.1）
     * 应答回来后会依次经过 onCharacteristicChanged → onNotifyBytes → handleFrame
     * → listener.onRead()，最终由 MainViewModel.onRead() 落到界面。
     */
    private final Runnable pollRunnable = new Runnable() {
        @Override
        public void run() {
            if (ready) {
                // 轮转三块（每块 ≤7 个寄存器，应答 19 字节，默认 MTU 20 也能容纳）
                // P9：每拍再追加一帧 IMU 区块（4 帧轮转），基础块刷新仍为 3s，IMU 为 4s
                if (queueLow.size() < 2) {
                    int phase = pollPhase % 3;
                    if (phase == 0) {
                        queueLow.addLast(buildReadRequest(0x0000, 7, "轮询A"));
                    } else if (phase == 1) {
                        queueLow.addLast(buildReadRequest(0x0007, 7, "轮询B"));
                    } else {
                        // FW>=1.1：面板焦点 0x0014 / 铝板限温 0x0015
                        queueLow.addLast(buildReadRequest(ModbusProtocol.REG_UI_FOCUS, 2, "轮询C"));
                    }
                    pollPhase++;

                    // 只在固件 >= 1.2 时才发 IMU 相位；旧固件会回异常 02（每秒弹 toast）
                    if (peerFwVersion >= ModbusProtocol.FW_VER_IMU) {
                        int ip = pollImuPhase % IMU_PHASE_START.length;
                        // 第 5 帧（0x003A~0x003C）是 v1.3 才有的，旧固件跳过
                        if (ip == 4 && peerFwVersion < ModbusProtocol.FW_VER_TUNE) {
                            pollImuPhase++;
                        } else {
                            queueLow.addLast(buildReadRequest(IMU_PHASE_START[ip], IMU_PHASE_COUNT[ip],
                                    "IMU" + ip));
                            pollImuPhase++;
                        }
                    }
                }
                pump();
            }
            main.postDelayed(this, POLL_PERIOD_MS);
        }
    };

    /* ------------------------------------------------------------------ 接收解析 */

    /**
     * 【拼包】把 Notify 分片追加进 rxBuf，并按 Modbus RTU 帧长切出完整帧。
     * 下位机一次应答可能被 BLE 拆成 2~3 个 notify，这里等够长度再交给 handleFrame()。
     */
    private void onNotifyBytes(byte[] chunk) {
        if (chunk == null || chunk.length == 0) {
            return;
        }
        if (rxLen + chunk.length > rxBuf.length) {
            rxLen = 0; // 缓冲异常，丢弃重新同步
        }
        System.arraycopy(chunk, 0, rxBuf, rxLen, chunk.length);
        rxLen += chunk.length;

        while (rxLen >= 3) {
            int expect = ModbusProtocol.expectedFrameLength(rxBuf, rxLen);
            if (expect == -1) {
                listener.onLog("!! 收到非协议数据，清空接收缓冲");
                rxLen = 0;
                return;
            }
            if (expect == 0 || rxLen < expect) {
                return; // 等下一个 notify 分片
            }
            byte[] frame = new byte[expect];
            System.arraycopy(rxBuf, 0, frame, 0, expect);
            if (rxLen > expect) {
                System.arraycopy(rxBuf, expect, rxBuf, 0, rxLen - expect);
            }
            rxLen -= expect;
            handleFrame(frame);
        }
    }

    /**
     * 【帧解析 + 分发】校验 CRC、区分功能码，并把 03 读应答回调给上层：
     *   listener.onRead(r.startReg, values) —— MainViewModel.onRead() 就挂在这里，
     *   startReg 是本次读的起始寄存器，values 是紧随其后的原始 16bit 值。
     */
    private void handleFrame(byte[] frame) {
        main.removeCallbacks(timeoutRunnable);
        int fc = frame[1] & 0xFF;
        Request r = pending;
        pending = null;

        if (!ModbusProtocol.crcOk(frame)) {
            listener.onLog("RX  CRC 错误  " + ModbusProtocol.hex(frame));
            schedulePump(MIN_TX_GAP_MS);
            return;
        }

        if ((fc & 0x80) != 0) {
            int code = frame[2] & 0xFF;
            listener.onLog("RX  异常 " + ModbusProtocol.exceptionText(code) + "  " + ModbusProtocol.hex(frame));
            listener.onException(code);
        } else if (fc == 0x03) {
            int[] values = ModbusProtocol.parseReadValues(frame);
            listener.onLog("RX  " + (r == null ? "读应答" : r.label) + "  " + ModbusProtocol.hex(frame));
            if (values != null && r != null && r.isRead) {
                // P9：0x000D~0x000F 一帧里带固件版本，记下来用于 IMU 区块的版本门控
                if (r.startReg == ModbusProtocol.REG_DEV_ID_LO && values.length >= 3) {
                    peerFwVersion = values[2] & 0xFFFF;
                }
                listener.onRead(r.startReg, values);
            }
        } else {
            listener.onLog("RX  写应答  " + ModbusProtocol.hex(frame));
            listener.onWriteAck();
        }
        schedulePump(MIN_TX_GAP_MS);
    }

    /** 供界面显示的扫描结果名称（缺少权限时回退为 MAC）。 */
    @SuppressLint("MissingPermission")
    public static String describe(BluetoothDevice d) {
        String n = null;
        try {
            n = d.getName();
        } catch (SecurityException ignored) {
        }
        if (n == null || n.isEmpty()) {
            n = "(未广播名称)";
        }
        return n + "\n" + d.getAddress();
    }
}
