package com.tempstir.blecontrol.viewmodel;

import android.content.Context;
import android.os.Handler;
import android.os.Looper;

import androidx.lifecycle.LiveData;
import androidx.lifecycle.MutableLiveData;
import androidx.lifecycle.ViewModel;

import com.tempstir.blecontrol.data.BleManager;
import com.tempstir.blecontrol.data.DeviceItem;
import com.tempstir.blecontrol.data.DeviceState;
import com.tempstir.blecontrol.data.ModbusProtocol;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * 首页 ViewModel（MVVM 的 VM 层）：
 *   Model  = {@link BleManager}（BLE/Modbus 通道）+ {@link DeviceState}（寄存器快照）
 *   View   = MainActivity，只订阅 LiveData，不再持有任何蓝牙状态
 *
 * 焦点轮转、参数步进/限幅、运行联锁判断等业务规则全部收敛在这里；
 * 所有对 LiveData 的写入都发生在主线程（BleManager 的回调已 post 到主线程）。
 */
public class MainViewModel extends ViewModel implements BleManager.Listener {

    /** 出厂模块 MAC（模块不支持改名，按 MAC 直连）。 */
    public static final String DEFAULT_DEVICE_MAC = "EC:B1:A6:B0:1B:A2"; // BT_ECB1A6B01BA2

    /** 与下位机 menu_ui.c 的 KEY1 循环顺序一致。 */
    public enum Focus {
        SPEED,   // 0 转速
        PLATE,   // 1 铝板上限温度
        LIQUID,  // 2 液体目标温度
        TIME;    // 3 定时

        Focus next() {
            Focus[] all = values();
            return all[(ordinal() + 1) % all.length];
        }
    }

    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private final DeviceState state = new DeviceState();
    private final StringBuilder logBuilder = new StringBuilder();
    private final BleManager ble;



    /* ---------------- LiveData：View 唯一数据来源 ---------------- */

    private final MutableLiveData<UiState> uiState = new MutableLiveData<>();
    private final MutableLiveData<List<DeviceItem>> devices = new MutableLiveData<>();
    private final MutableLiveData<String> deviceCountText = new MutableLiveData<>();
    private final MutableLiveData<String> logText = new MutableLiveData<>("");
    private final MutableLiveData<Event<String>> toastEvent = new MutableLiveData<>();
    private final MutableLiveData<Event<String>> blockedStartEvent = new MutableLiveData<>();
    private final MutableLiveData<Event<String>> deviceInfoEvent = new MutableLiveData<>();
    private final MutableLiveData<Event<String>> connectedEvent = new MutableLiveData<>();

    private Focus focus = Focus.SPEED;
    /** 本地最近一次切焦点的时间，1.5s 内不被下位机旧快照回滚。 */
    private long focusLocalAt;
    private String deviceNameCache;
    private String connText = "未连接";

    public MainViewModel(Context context) {
        ble = new BleManager(context.getApplicationContext(), this);
        devices.setValue(Collections.singletonList(DeviceItem.preset(DEFAULT_DEVICE_MAC)));
        deviceCountText.setValue("扫描中…点击设备直接连接");
        publishUi();
        appendLog("TempStir BLE 控制台就绪");
        appendLog("目标模块：BT_ECB1A6B01BA2 / FFF0-FFF1-FFF2");
        appendLog("提示：点击「扫描 / 连接设备」开始");
    }

    @Override
    protected void onCleared() {
        ble.disconnect();
        super.onCleared();
    }

    /* ------------------------------------------------------------ 只读出口 */

    public LiveData<UiState> getUiState() {
        return uiState;
    }

    public LiveData<List<DeviceItem>> getDevices() {
        return devices;
    }

    public LiveData<String> getDeviceCountText() {
        return deviceCountText;
    }

    public LiveData<String> getLogText() {
        return logText;
    }

    public LiveData<Event<String>> getToastEvent() {
        return toastEvent;
    }

    public LiveData<Event<String>> getBlockedStartEvent() {
        return blockedStartEvent;
    }

    public LiveData<Event<String>> getDeviceInfoEvent() {
        return deviceInfoEvent;
    }

    public LiveData<Event<String>> getConnectedEvent() {
        return connectedEvent;
    }

    /* ------------------------------------------------------------ 连接与扫描 */

    public boolean isBluetoothOn() {
        return ble.isBluetoothOn();
    }

    /** 打开设备选择弹窗时调用：清空上次结果，重新开始扫描。 */
    public void startScan() {
        devices.setValue(Collections.singletonList(DeviceItem.preset(DEFAULT_DEVICE_MAC)));
        deviceCountText.setValue("扫描中…点击设备直接连接");
        ble.startScan();
    }

    public void stopScan() {
        ble.stopScan();
    }

    public void connect(DeviceItem item) {
        appendLog("选择设备 " + item.displayName() + " [" + item.address + "]");
        connText = "正在连接 " + item.displayName() + " …";
        publishUi();
        ble.connect(item.address);
    }

    public void disconnect() {
        ble.disconnect();
        toast("已断开连接");
    }

    /* ------------------------------------------------------------ 4 个软按键 */

    public void cycleFocus() {
        Focus next = focus.next();
        focus = next;
        focusLocalAt = System.currentTimeMillis();
        // FW>=1.1：把焦点写进 0x0014，下位机 OLED 的 ">" 箭头会移动到同一行
        if (focusRemoteSupported()) {
            ble.sendWriteSingle(ModbusProtocol.REG_UI_FOCUS, next.ordinal());
        }
        appendLog("KEY1 焦点 -> " + focusLabel()
                + (focusRemoteSupported() ? "（0x0014=" + next.ordinal() + "，面板箭头同步）"
                                          : "（固件 <1.1，仅 App 内切换）"));
        publishUi();
    }

    public void adjust(int dir) {
        int reg = focusReg();
        int cur = focusValue();
        int next = Math.max(0, Math.min(focusMax(), cur + dir * focusStep()));
        if (next == cur) {
            toast(focusLabel() + " 已到边界");
            return;
        }
        setFocusValue(next);                       // 本地乐观刷新，随后由轮询校准
        ble.sendWriteSingle(reg, next);
        appendLog(String.format(Locale.CHINA, "KEY%d  0x%04X  %d -> %d",
                dir > 0 ? 2 : 3, reg, cur, next));
        publishUi();
    }

    public void toggleRun() {
        if (state.isRunning()) {
            ble.sendWriteSingle(ModbusProtocol.REG_CTRL_CMD, ModbusProtocol.CMD_STOP);
            appendLog("KEY4 停止 (bit1)");
            toast("已下发停止命令");
            return;
        }
        // 固件 App_CommExecCommand()：故障位非 0 时直接返回异常 04，这里先给出明确原因
        if (state.everRead && state.faultCode != 0) {
            blockedStartEvent.setValue(new Event<>(buildBlockedMessage()));
            return;
        }
        startNow();
    }

    public void clearFault() {
        ble.sendWriteSingle(ModbusProtocol.REG_CTRL_CMD, ModbusProtocol.CMD_CLEAR_FAULT);
        appendLog("下发清故障 (bit2)");
    }

    public void clearFaultAndStart() {
        appendLog("下发清故障 (bit2)，700ms 后启动");
        ble.sendWriteSingle(ModbusProtocol.REG_CTRL_CMD, ModbusProtocol.CMD_CLEAR_FAULT);
        mainHandler.postDelayed(() ->
                ble.sendWriteSingle(ModbusProtocol.REG_CTRL_CMD, ModbusProtocol.CMD_START), 700L);
    }

    /** P9：请求下位机做一次陀螺零偏标定（下发 0x0030 bit1，1~2s 内需保持静止）。 */
    public void calibrateImuBias() {
        ble.sendWriteSingle(ModbusProtocol.REG_IMU_CTRL, ModbusProtocol.IMUCTL_CAL_BIAS);
        toast("已下发零偏标定，请保持静止约 2 秒");
        appendLog("下发 IMU 零偏标定 0x0030="
                + String.format("%04X", ModbusProtocol.IMUCTL_CAL_BIAS));
    }

    public void requestDeviceInfo() {
        ble.sendRead(ModbusProtocol.REG_DEV_ID_LO, 3, true);
        String msg = "设备：" + (deviceNameCache == null ? "未连接" : deviceNameCache) + "\n"
                + state.deviceIdText() + "\n"
                + "协商 MTU：" + ble.getMtu() + "\n\n"
                + "下位机物理 KEY1 长按会进入 RTC 校时页；蓝牙协议未开放校时寄存器，"
                + "因此 App 的长按 KEY1 改为显示本设备信息。校时请在面板上完成。";
        deviceInfoEvent.setValue(new Event<>(msg));
    }

    public void clearLog() {
        logBuilder.setLength(0);
        logText.setValue("");
        appendLog("日志已清空");
    }

    /* ------------------------------------------------------------ 焦点参数 */

    private void startNow() {
        ble.sendWriteSingle(ModbusProtocol.REG_CTRL_CMD, ModbusProtocol.CMD_START);
        appendLog("KEY4 启动 (bit0)");
        toast("已下发启动命令");
    }

    /** 启动被下位机联锁拒绝时，说明原因并提供清故障入口。 */
    private String buildBlockedMessage() {
        // 首页把 RTC 掉电（0x0080）隐藏了，这里如果只剩它，要说明清楚避免显示"无（故障码 0x0080）"
        String desc = state.displayFaultCode() != 0 ? state.faultText() : "仅 RTC 掉电（首页不提示）";
        String faults = desc + String.format(Locale.CHINA, "（故障码 0x%04X）", state.faultCode);
        return "下位机当前故障：" + faults + "\n\n"
                + "下位机固件在故障位非 0 时，会把 0x0007 的启动命令直接拒绝并回异常 04"
                + "（数值越界 / 保护拒绝）。这不是 App 的问题，也不是蓝牙参数问题。\n\n"
                + "「清故障」只释放加热故障锁存；铝板限温 0x0010 需要板温降到"
                + "「上限 − 2℃」以下才会自动解除。";
    }

    /** 下位机固件是否支持 0x0014 焦点寄存器（>=1.1）。 */
    private boolean focusRemoteSupported() {
        return state.everRead && state.fwVersion >= ModbusProtocol.FW_VER_REMOTE_FOCUS;
    }

    private String focusLabel() {
        switch (focus) {
            case SPEED:  return "转速";
            case PLATE:  return "铝板限温";
            case LIQUID: return "液温";
            case TIME:   return "定时";
            default:     return "";
        }
    }

    private int focusReg() {
        switch (focus) {
            case SPEED:  return ModbusProtocol.REG_TARGET_SPEED;
            case PLATE:  return ModbusProtocol.REG_PLATE_LIMIT;
            case LIQUID: return ModbusProtocol.REG_TARGET_TEMP;
            case TIME:   return ModbusProtocol.REG_SET_TIME;
            default:     return 0;
        }
    }

    private int focusValue() {
        switch (focus) {
            case SPEED:  return state.targetRpm;
            case PLATE:  return state.plateLimit;
            case LIQUID: return state.targetTempC10;
            case TIME:   return state.setTimeSec;
            default:     return 0;
        }
    }

    private void setFocusValue(int v) {
        switch (focus) {
            case SPEED:  state.targetRpm = v; break;
            case PLATE:  state.plateLimit = v; break;
            case LIQUID: state.targetTempC10 = v; break;
            case TIME:   state.setTimeSec = v; break;
        }
    }

    private int focusStep() {
        switch (focus) {
            case SPEED:  return 5;   // rpm，与 MENU_UI_SPEED_STEP 一致
            case PLATE:  return 1;   // ℃
            case LIQUID: return 10;  // 0.1℃ -> 1℃，与 MENU_UI_LIQUID_STEP 一致
            case TIME:   return 1;   // s
            default:     return 1;
        }
    }

    private int focusMax() {
        switch (focus) {
            case SPEED:  return ModbusProtocol.SPEED_MAX_RPM;
            case PLATE:  return 120;                       // MENU_UI_PLATE_MAX
            case LIQUID: return ModbusProtocol.LIQUID_MAX_C10;
            case TIME:   return ModbusProtocol.TIME_MAX_SEC;
            default:     return 0;
        }
    }

    private String focusValueText() {
        switch (focus) {
            case SPEED:  return state.targetRpm + " rpm";
            case PLATE:  return state.plateLimit + " ℃（上限）";
            case LIQUID: return DeviceState.tempText(state.targetTempC10);
            case TIME:   return state.setTimeSec + " s" + (state.setTimeSec == 0 ? "（不限时）" : "");
            default:     return "--";
        }
    }

    /* ------------------------------------------------------------ 状态发布 */

    /**
     * 把当前寄存器快照 + 焦点状态整理成一份 UI 状态推给 View。
     * 截图里的"液温 设定/实测""转速 设定/实测""铝板温度""设定/剩余时间""加热/电机占空比"
     * "系统状态""故障码"以及下方"当前调节项"都在这里取值；View 只做 setText。
     */
    private void publishUi() {
        // 下位机固件支持 0x0014 时，面板 KEY1 切焦点后箭头位置会回传，这里跟随
        if (focusRemoteSupported()
                && System.currentTimeMillis() - focusLocalAt > 1500L
                && state.uiFocus >= 0 && state.uiFocus <= 3) {
            Focus remote = Focus.values()[state.uiFocus];
            if (remote != focus) {
                focus = remote;
                appendLog("下位机面板 KEY1 -> 焦点 " + focusLabel());
            }
        }

        UiState s = new UiState();
        s.connText = connText;
        s.focusLabel = "当前调节项：" + focusLabel()
                + "   [寄存器 0x" + String.format("%04X", focusReg()) + "]";
        s.focusValue = focusValueText();
        s.focusHint = "KEY1 切换 · KEY2 +" + focusStep() + " · KEY3 −" + focusStep()
                + " · 长按连发";

        s.liquidText = DeviceState.tempText(state.targetTempC10) + " / "
                + DeviceState.tempText(state.curTempC10);
        s.speedText = state.targetRpm + " / " + state.curRpm + " rpm";
        s.plateText = DeviceState.tempText(state.plateTempC10)
                + (focusRemoteSupported() ? " / 上限 " + state.plateLimit + "℃" : "");
        s.timeText = state.setTimeSec + " / " + state.remainSec + " s";
        s.percentText = state.heatPct + "% / " + state.motorPct + "%";
        s.statusText = state.everRead ? state.statusText() : "--";
        // 首页不展示 RTC 掉电（0x0080）：判断与文字都用 displayFaultCode()，
        // 只设置该位时就显示"无"，原始 faultCode 仍保留给协议/联锁逻辑使用。
        int displayFault = state.displayFaultCode();
        s.fault = state.everRead && displayFault != 0;
        s.faultText = state.everRead
                ? (s.fault ? state.faultText() + String.format(Locale.CHINA, "（0x%04X）", displayFault)
                           : "无")
                : "--";
        s.running = state.isRunning();

        /* P9：倾斜角（下位机判据 = max(|roll|,|pitch|)，门限来自 0x0036，默认 10.0°） */
        if (state.imuEverRead) {
            int thC10 = (state.tiltTh > 0) ? state.tiltTh : 100;
            int absRoll = Math.abs(state.rollC10);
            int absPitch = Math.abs(state.pitchC10);
            int maxTilt = Math.max(absRoll, absPitch);
            s.tiltAlarm = maxTilt >= thC10;
            s.tiltText = String.format(Locale.CHINA, "%.1f°（R %.1f / P %.1f）门限 %.1f°%s",
                    maxTilt / 10.0f, state.rollC10 / 10.0f, state.pitchC10 / 10.0f,
                    thC10 / 10.0f, s.tiltAlarm ? " 超限" : "");
        }

        /* P9：六轴区块（固件 < 1.2 或尚未读到该区块时保持 "--"） */
        if (state.imuEverRead) {
            s.imuAccText = String.format(Locale.CHINA, "%d / %d / %d mg",
                    state.accX, state.accY, state.accZ);
            s.imuGyroText = String.format(Locale.CHINA, "%.1f / %.1f / %.1f dps",
                    state.gyroX / 10.0f, state.gyroY / 10.0f, state.gyroZ / 10.0f);
            s.imuAttText = String.format(Locale.CHINA, "R %.1f°  P %.1f°  Y %.1f°  振动 %d mg",
                    state.rollC10 / 10.0f, state.pitchC10 / 10.0f,
                    state.yawC10 / 10.0f, state.vibRms);
            s.imuMiscText = String.format(Locale.CHINA,
                    "%s %d rpm · 打滑 %.1f%% · %s · α %.1f dps/s · 加速 %d ms / 减速 %d ms",
                    state.rampStateText(), state.rampCmdRpm, state.slipErr / 10.0f,
                    state.imuStateText(), state.angAccel / 10.0f,
                    state.accTimeMs, state.decTimeMs);
        }

        /* A4/A2：参数页预填（固件 >= 1.3；否则 paramSupported=false，参数按钮会拒绝） */
        s.paramSupported = state.everRead && state.fwVersion >= ModbusProtocol.FW_VER_TUNE;
        s.heatOcRaw = state.paramEverRead ? state.heatOcRaw : 0;
        s.assistOn = state.paramEverRead && (state.motorAssist & ModbusProtocol.ASSIST_STALL_EN) != 0;
        s.tuneOn = state.paramEverRead && (state.tuneCtrl & ModbusProtocol.TUNECTL_PRINT) != 0;

        uiState.setValue(s);
    }

    /* ------------------------------------------------------------ A4/A2：参数写入 */

    /** 固件 >= 1.3 才支持 0x003A~0x003C。 */
    public boolean paramSupported() {
        return state.everRead && state.fwVersion >= ModbusProtocol.FW_VER_TUNE;
    }

    /** 打开参数页时主动读一次最新值（高优先级，不等轮询）。 */
    public void requestParams() {
        if (paramSupported()) {
            ble.sendRead(ModbusProtocol.REG_HEAT_OC, 3, true);
            ble.sendRead(ModbusProtocol.REG_ALPHA_LIMIT, 4, true);
        }
    }

    /** 参数页预填值：[0]打滑0.1% [1]振动mg [2]倾斜0.1° [3]斜率rpm/s [4]过流raw [5]辅助(0/1) [6]统计(0/1) */
    public int[] paramSnapshot() {
        return new int[]{
                state.slipTh,
                state.vibTh,
                state.tiltTh,
                state.alphaLimit,
                state.paramEverRead ? state.heatOcRaw : 0,
                ((state.motorAssist & ModbusProtocol.ASSIST_STALL_EN) != 0) ? 1 : 0,
                ((state.tuneCtrl & ModbusProtocol.TUNECTL_PRINT) != 0) ? 1 : 0
        };
    }

    /** 写入单个参数寄存器（0x0033~0x0036 / 0x003A~0x003C），带版本门控。 */
    public void writeParam(int reg, int value) {
        if (!paramSupported()) {
            toast("固件版本过低（需 >= 1.3），该参数只能在面板/重烧固件里改");
            return;
        }
        ble.sendWriteSingle(reg, value);
        appendLog(String.format(Locale.CHINA, "参数写入 0x%04X = %d", reg, value));
    }

    /** 一次性下发参数页全部字段（未改的也重发，幂等）。 */
    public void applyImuParams(int slipTh, int vibTh, int tiltTh, int alphaLimit,
                               int heatOc, boolean assistOn, boolean tuneOn) {
        if (!paramSupported()) {
            toast("固件版本过低（需 >= 1.3），无法写入参数");
            return;
        }
        ble.sendWriteSingle(ModbusProtocol.REG_SLIP_TH, clamp(slipTh, 10, 1000));
        ble.sendWriteSingle(ModbusProtocol.REG_VIB_TH, clamp(vibTh, 10, 5000));
        ble.sendWriteSingle(ModbusProtocol.REG_TILT_TH, clamp(tiltTh, 10, 900));
        ble.sendWriteSingle(ModbusProtocol.REG_ALPHA_LIMIT, clamp(alphaLimit, 50, 5000));
        ble.sendWriteSingle(ModbusProtocol.REG_HEAT_OC, clamp(heatOc, 100, 4095));
        ble.sendWriteSingle(ModbusProtocol.REG_MOTOR_ASSIST,
                assistOn ? ModbusProtocol.ASSIST_STALL_EN : 0);
        ble.sendWriteSingle(ModbusProtocol.REG_TUNE_CTRL,
                tuneOn ? ModbusProtocol.TUNECTL_PRINT : 0);
        appendLog("参数已下发：打滑/振动/倾斜/斜率/过流 + 辅助堵转 + 实测统计");
        toast("参数已下发（掉电保持，实测统计不落盘）");
        mainHandler.postDelayed(this::requestParams, 900L);
    }

    private static int clamp(int v, int min, int max) {
        return Math.max(min, Math.min(max, v));
    }

    private void appendLog(String line) {
        String time = new SimpleDateFormat("HH:mm:ss", Locale.CHINA).format(new Date());
        logBuilder.append(time).append("  ").append(line).append('\n');
        String text = logBuilder.toString();
        // 只保留最近 120 行
        int lines = 0, idx = text.length() - 1;
        while (idx >= 0 && lines < 120) {
            idx = text.lastIndexOf('\n', idx - 1);
            lines++;
        }
        if (idx > 0) {
            text = text.substring(idx + 1);
        }
        logText.setValue(text);
    }

    private void toast(String msg) {
        toastEvent.setValue(new Event<>(msg));
    }

    /* ------------------------------------------------------------ BleManager 回调 */

    @Override
    public void onStatus(String text) {
        connText = text;
        publishUi();
        appendLog("[状态] " + text);
    }

    /**
     * 日志过滤（用户 2026-11-06 要求）：
     * 1 s 轮询产生的 TX/RX 十六进制明细**完全不上屏**（连 5 s 汇总也不打），
     * 只保留主要操作日志：写寄存器、按需读、KEY 操作、连接/断开、异常/超时、状态变化。
     * 功能不受影响，只是不打印。
     */
    @Override
    public void onLog(String line) {
        if (isPollTraffic(line)) {
            return;
        }
        appendLog(line);
    }

    /** 是否为 1 s 轮询产生的帧（这类日志只做 5 s 汇总）。 */
    private static boolean isPollTraffic(String line) {
        if (line == null) {
            return false;
        }
        return line.contains("轮询A") || line.contains("轮询B") || line.contains("轮询C")
                || line.contains("IMU0") || line.contains("IMU1") || line.contains("IMU2")
                || line.contains("IMU3") || line.contains("IMU4");
    }

    @Override
    public void onDevices(List<DeviceItem> scanned) {
        List<DeviceItem> items = new ArrayList<>(scanned.size() + 1);
        items.add(DeviceItem.preset(DEFAULT_DEVICE_MAC));
        for (DeviceItem d : scanned) {
            boolean target = DEFAULT_DEVICE_MAC.equalsIgnoreCase(d.address);
            items.add(new DeviceItem(d.name, d.address, d.rssi, target, false));
        }
        devices.setValue(items);
        deviceCountText.setValue("已发现 " + scanned.size() + " 台 · 点击设备直接连接");
    }

    @Override
    public void onConnected(String name, String address) {
        deviceNameCache = name + " / " + address;
        ble.stopScan();
        connText = "已连接成功：" + deviceNameCache;
        appendLog("已连接成功 " + deviceNameCache + "（扫描已停止）");
        publishUi();
        connectedEvent.setValue(new Event<>(deviceNameCache));
    }

    @Override
    public void onDisconnected() {
        deviceNameCache = null;
        connText = "未连接";
        appendLog("连接已断开");
        publishUi();
    }

    /**
     * 【截图数据的回调点】BleManager 解析出一帧 03 读应答后回调到这里：
     *   1) DeviceState.applyRead() 按 startReg+i 把每个寄存器写进快照（液温/转速/铝板温度/时间/占空比/状态字/故障码…）
     *   2) publishUi() 把整份快照加工成 UiState 并 setValue 给 LiveData
     *   3) MainActivity.render(UiState) 负责把文字贴到截图里的那几行 TextView 上。
     * 该回调已被 BleManager 切到主线程，可以直接操作 LiveData。
     */
    @Override
    public void onRead(int startReg, int[] values) {
        state.applyRead(startReg, values);
        publishUi();
    }

    @Override
    public void onWriteAck() {
        appendLog("      下位机已确认写入");
    }

    @Override
    public void onException(int code) {
        String msg = ModbusProtocol.exceptionText(code);
        if (code == 0x04) {
            msg = "04 保护拒绝："
                    + (state.displayFaultCode() != 0 ? "下位机故障 " + state.faultText()
                                                     : "数值越界（检查参数上限）");
        }
        toast(msg);
        appendLog("[异常] " + msg);
    }

    @Override
    public void onMtu(int mtu) {
        appendLog("MTU = " + mtu);
    }
}
