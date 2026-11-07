package com.tempstir.blecontrol.ui;

import android.Manifest;
import android.annotation.SuppressLint;
import android.app.AlertDialog;
import android.app.Dialog;
import android.bluetooth.BluetoothAdapter;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.graphics.drawable.ColorDrawable;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.text.Editable;
import android.text.TextWatcher;
import android.util.DisplayMetrics;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.view.Window;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.lifecycle.ViewModel;
import androidx.lifecycle.ViewModelProvider;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import com.tempstir.blecontrol.R;
import com.tempstir.blecontrol.data.DeviceItem;
import com.tempstir.blecontrol.data.ModbusProtocol;
import com.tempstir.blecontrol.databinding.ActivityMainBinding;
import com.tempstir.blecontrol.databinding.DialogDeviceListBinding;
import com.tempstir.blecontrol.viewmodel.MainViewModel;
import com.tempstir.blecontrol.viewmodel.UiState;

import java.util.ArrayList;
import java.util.List;

/**
 * 首页（MVVM 的 View 层）：
 *   - 布局全部通过 ViewBinding 访问，没有 findViewById
 *   - 不做任何业务计算，只把用户操作转给 {@link MainViewModel}
 *   - 只订阅 ViewModel 的 LiveData 并渲染
 *
 * 面板定义（menu_ui.h）：
 *   KEY1 短按  切换焦点项：速度 -> 铝板温度 -> 液体温度 -> 时间
 *   KEY2       当前项参数 +（长按连发）
 *   KEY3       当前项参数 -
 *   KEY4       电机运行 / 停止
 * 蓝牙通道只暴露 Modbus 寄存器，不能注入"按键事件"，因此 App 用等价寄存器写实现。
 */
public class MainActivity extends AppCompatActivity {

    private static final int REQ_PERMISSIONS = 1001;
    /** 设备列表弹窗高度上限：屏幕高度的 60%（整个弹窗，含标题与底部按钮）。 */
    private static final float DEVICE_DIALOG_MAX_HEIGHT_RATIO = 0.60f;

    /** AppCompatActivity 自带 Lifecycle/ViewModelStore，无需手写 LifecycleRegistry。 */
    private final Handler mainHandler = new Handler(Looper.getMainLooper());

    private ActivityMainBinding binding;
    private MainViewModel viewModel;

    private DeviceAdapter deviceAdapter;
    private Dialog deviceDialog;
    private DialogDeviceListBinding dialogBinding;
    private RecyclerView.AdapterDataObserver deviceObserver;
    private int dialogWidthPx;
    private int dialogMaxHeightPx;
    private boolean devicePickHandled;
    private boolean wantScanAfterPermission;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        binding = ActivityMainBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        viewModel = new ViewModelProvider(this, new ViewModelProvider.Factory() {
            @NonNull
            @Override
            @SuppressWarnings("unchecked")
            public <T extends ViewModel> T create(@NonNull Class<T> modelClass) {
                return (T) new MainViewModel(getApplicationContext());
            }
        }).get(MainViewModel.class);

        binding.btnConnect.setOnClickListener(v -> onConnectClicked());
        binding.btnDisconnect.setOnClickListener(v -> viewModel.disconnect());
        binding.btnClearLog.setOnClickListener(v -> viewModel.clearLog());

        binding.btnKey1.setOnClickListener(v -> viewModel.cycleFocus());
        binding.btnKey1.setOnLongClickListener(v -> {
            viewModel.requestDeviceInfo();
            return true;
        });
        binding.btnKey2.setOnTouchListener(new HoldRepeatListener(() -> viewModel.adjust(+1)));
        binding.btnKey3.setOnTouchListener(new HoldRepeatListener(() -> viewModel.adjust(-1)));
        binding.btnKey4.setOnClickListener(v -> viewModel.toggleRun());
        binding.btnImuCal.setOnClickListener(v -> viewModel.calibrateImuBias());
        binding.btnParam.setOnClickListener(v -> showImuParamDialog());

        observeViewModel();
    }

    /* -------------------------------------------------------------- 订阅 LiveData */

    /**
     * 订阅链：MainViewModel.publishUi() → LiveData&lt;UiState&gt; → 这里的 render()。
     * 页面只在这里挂一次观察者，Activity 销毁时由 LiveData 自动反注册。
     */
    private void observeViewModel() {
        viewModel.getUiState().observe(this, this::render);

        viewModel.getLogText().observe(this, text -> {
            binding.tvLog.setText(text);
            binding.scrollLog.post(() -> binding.scrollLog.fullScroll(View.FOCUS_DOWN));
        });

        viewModel.getDevices().observe(this, list -> {
            if (deviceAdapter != null) {
                deviceAdapter.submitList(new ArrayList<>(list));
            }
        });

        viewModel.getDeviceCountText().observe(this, text -> {
            if (dialogBinding != null) {
                dialogBinding.tvDevCount.setText(text);
            }
        });

        viewModel.getToastEvent().observe(this, event -> {
            String msg = event.getContentIfNotHandled();
            if (msg != null) {
                Toast.makeText(this, msg, Toast.LENGTH_SHORT).show();
            }
        });

        viewModel.getBlockedStartEvent().observe(this, event -> {
            String msg = event.getContentIfNotHandled();
            if (msg != null) {
                showStartBlockedDialog(msg);
            }
        });

        viewModel.getDeviceInfoEvent().observe(this, event -> {
            String msg = event.getContentIfNotHandled();
            if (msg != null) {
                new AlertDialog.Builder(this)
                        .setTitle("设备信息 / KEY1 长按")
                        .setMessage(msg)
                        .setPositiveButton("知道了", null)
                        .show();
            }
        });

        viewModel.getConnectedEvent().observe(this, event -> {
            String name = event.getContentIfNotHandled();
            if (name != null) {
                dismissDeviceDialog();
                Toast.makeText(this, "已连接成功，FFF1 已订阅", Toast.LENGTH_SHORT).show();
            }
        });
    }

    /**
     * 截图里"下位机实时状态"这一块就是在这里刷新的（数据来自 MainViewModel.onRead → publishUi）：
     *   tvLiquid 液温 设定/实测 ← 0x0001 / 0x0000
     *   tvSpeed  转速 设定/实测 ← 0x0003 / 0x0002
     *   tvPlate  铝板温度        ← 0x0008（只读）
     *   tvTime   设定/剩余时间   ← 0x000B / 0x0004
     *   tvPct    加热/电机占空比 ← 0x0009 / 0x000A
     *   tvSys    系统状态字      ← 0x0005
     *   tvFault  故障码          ← 0x0006
     * 下方"当前调节项 / 0 rpm"来自 tvFocusLabel / tvFocusValue / tvFocusHint。
     */
    private void render(UiState s) {
        binding.tvConn.setText(s.connText);
        binding.tvFocusLabel.setText(s.focusLabel);
        binding.tvFocusValue.setText(s.focusValue);
        binding.tvFocusHint.setText(s.focusHint);
        binding.tvLiquid.setText(s.liquidText);
        binding.tvSpeed.setText(s.speedText);
        binding.tvPlate.setText(s.plateText);
        binding.tvTime.setText(s.timeText);
        binding.tvPct.setText(s.percentText);
        binding.tvSys.setText(s.statusText);
        binding.tvTilt.setText(s.tiltText);
        binding.tvTilt.setTextColor(getColor(s.tiltAlarm ? R.color.red : R.color.textMain));
        binding.tvImuAcc.setText(s.imuAccText);
        binding.tvImuGyro.setText(s.imuGyroText);
        binding.tvImuAtt.setText(s.imuAttText);
        binding.tvImuMisc.setText(s.imuMiscText);
        binding.tvFault.setText(s.faultText);
        binding.tvFault.setTextColor(getColor(s.fault ? R.color.red : R.color.green));
        binding.btnKey4.setText(s.running ? "KEY4\n停止" : "KEY4\n启动");
        binding.btnKey4.setBackgroundResource(s.running ? R.drawable.btn_stop : R.drawable.btn_run);
    }

    /* -------------------------------------------------------------- 权限与扫描 */

    private String[] requiredPermissions() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            return new String[]{
                    Manifest.permission.BLUETOOTH_SCAN,
                    Manifest.permission.BLUETOOTH_CONNECT
            };
        }
        return new String[]{Manifest.permission.ACCESS_FINE_LOCATION};
    }

    private boolean hasPermissions() {
        for (String p : requiredPermissions()) {
            if (checkSelfPermission(p) != PackageManager.PERMISSION_GRANTED) {
                return false;
            }
        }
        return true;
    }

    private void onConnectClicked() {
        if (!hasPermissions()) {
            wantScanAfterPermission = true;
            requestPermissions(requiredPermissions(), REQ_PERMISSIONS);
            return;
        }
        if (!viewModel.isBluetoothOn()) {
            try {
                startActivity(new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE));
                toast("请先开启蓝牙，再点一次连接");
            } catch (Exception e) {
                toast("蓝牙未开启，请在系统设置中打开");
            }
            return;
        }
        showDeviceDialog();
        viewModel.startScan();
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == REQ_PERMISSIONS && wantScanAfterPermission) {
            wantScanAfterPermission = false;
            boolean ok = grantResults.length > 0;
            for (int r : grantResults) {
                if (r != PackageManager.PERMISSION_GRANTED) {
                    ok = false;
                }
            }
            if (ok) {
                onConnectClicked();
            } else {
                toast("缺少蓝牙权限，无法扫描设备");
            }
        }
    }

    /* -------------------------------------------------------------- 设备选择弹窗 */

    @SuppressLint("MissingPermission")
    private void showDeviceDialog() {
        if (deviceDialog != null) {
            if (deviceDialog.isShowing()) {
                return;
            }
            deviceDialog = null;           // 残留引用，重建
        }
        devicePickHandled = false;
        dialogBinding = DialogDeviceListBinding.inflate(getLayoutInflater());
        RecyclerView rv = dialogBinding.rvDevices;

        deviceAdapter = new DeviceAdapter(this::onDevicePicked);
        rv.setLayoutManager(new LinearLayoutManager(this));
        rv.setAdapter(deviceAdapter);
        rv.setItemAnimator(null);          // 扫描中频繁刷新时避免闪烁
        rv.setHasFixedSize(false);

        List<DeviceItem> initial = viewModel.getDevices().getValue();
        if (initial != null) {
            deviceAdapter.submitList(new ArrayList<>(initial));
        }
        String countText = viewModel.getDeviceCountText().getValue();
        if (countText != null) {
            dialogBinding.tvDevCount.setText(countText);
        }

        // 宽度 = 屏宽 90%（上限 420dp）；高度先按内容包裹，再收敛到屏幕 60% 以内
        DisplayMetrics dm = getResources().getDisplayMetrics();
        dialogWidthPx = Math.min((int) (dm.widthPixels * 0.90f), dp(420));
        dialogMaxHeightPx = (int) (dm.heightPixels * DEVICE_DIALOG_MAX_HEIGHT_RATIO);
        dialogBinding.getRoot().setLayoutParams(new ViewGroup.LayoutParams(
                dialogWidthPx, ViewGroup.LayoutParams.WRAP_CONTENT));

        // 设备数量变化（扫描到新设备）时重新贴合高度，最多长到屏幕 60%
        deviceObserver = new RecyclerView.AdapterDataObserver() {
            @Override
            public void onChanged() {
                requestFitDeviceDialogHeight();
            }

            @Override
            public void onItemRangeInserted(int positionStart, int itemCount) {
                requestFitDeviceDialogHeight();
            }

            @Override
            public void onItemRangeRemoved(int positionStart, int itemCount) {
                requestFitDeviceDialogHeight();
            }

            @Override
            public void onItemRangeChanged(int positionStart, int itemCount) {
                requestFitDeviceDialogHeight();
            }

            @Override
            public void onItemRangeChanged(int positionStart, int itemCount, Object payload) {
                requestFitDeviceDialogHeight();
            }

            @Override
            public void onItemRangeMoved(int fromPosition, int toPosition, int itemCount) {
                requestFitDeviceDialogHeight();
            }
        };
        deviceAdapter.registerAdapterDataObserver(deviceObserver);

        deviceDialog = new Dialog(this, R.style.DeviceDialogTheme);
        deviceDialog.setContentView(dialogBinding.getRoot());
        deviceDialog.setCanceledOnTouchOutside(true);
        deviceDialog.setOnDismissListener(d -> {
            if (deviceAdapter != null && deviceObserver != null) {
                deviceAdapter.unregisterAdapterDataObserver(deviceObserver);
            }
            deviceObserver = null;
            deviceDialog = null;
            deviceAdapter = null;
            dialogBinding = null;
            // 关闭弹窗即结束扫描，避免后台继续耗电
            viewModel.stopScan();
        });
        dialogBinding.btnDialogCancel.setOnClickListener(v -> dismissDeviceDialog());

        deviceDialog.show();
        Window win = deviceDialog.getWindow();
        if (win != null) {
            win.setBackgroundDrawable(new ColorDrawable(Color.TRANSPARENT));
            // 卡片自身已定尺寸，窗口按内容包裹即可（透明边框不可见）
            win.setLayout(ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT);
        }
        rv.scrollToPosition(0);
        fitDeviceDialogHeight();
    }

    private void requestFitDeviceDialogHeight() {
        if (dialogBinding != null) {
            dialogBinding.getRoot().post(this::fitDeviceDialogHeight); // 合并同一批数据变化，下一帧再量
        }
    }

    /**
     * 把弹窗高度收敛到「内容高度」与「屏幕 60%」的较小值：
     * 设备少时弹窗贴合内容，设备多时封顶到屏幕 60%，多出的部分在列表内滚动。
     */
    private void fitDeviceDialogHeight() {
        final View card = dialogBinding == null ? null : dialogBinding.getRoot();
        final RecyclerView rv = dialogBinding == null ? null : dialogBinding.rvDevices;
        if (card == null || rv == null || deviceDialog == null || dialogMaxHeightPx <= 0) {
            return;
        }
        int widthSpec = View.MeasureSpec.makeMeasureSpec(dialogWidthPx, View.MeasureSpec.EXACTLY);
        int capSpec = View.MeasureSpec.makeMeasureSpec(dialogMaxHeightPx, View.MeasureSpec.AT_MOST);

        // 1) 先量出标题栏、分隔线、底部按钮这些固定部分的高度
        rv.setVisibility(View.GONE);
        card.measure(widthSpec, capSpec);
        int chrome = card.getMeasuredHeight();
        rv.setVisibility(View.VISIBLE);

        // 2) 列表实际高度 = min(列表内容高度, 剩余空间)，剩余空间之外靠滚动
        int listMax = Math.max(0, dialogMaxHeightPx - chrome);
        rv.measure(widthSpec, View.MeasureSpec.makeMeasureSpec(listMax, View.MeasureSpec.AT_MOST));
        int listHeight = Math.min(rv.getMeasuredHeight(), listMax);
        if (listHeight <= 0 && deviceAdapter != null && deviceAdapter.getItemCount() > 0) {
            return;                        // 列表还没量出来，保持现状，等下一次数据回调
        }

        int cardHeight = Math.min(chrome + listHeight, dialogMaxHeightPx);
        ViewGroup.LayoutParams rvLp = rv.getLayoutParams();
        if (rvLp != null && rvLp.height != listHeight) {
            rvLp.height = listHeight;
            rv.setLayoutParams(rvLp);
        }
        ViewGroup.LayoutParams cardLp = card.getLayoutParams();
        if (cardLp != null && cardLp.height != cardHeight) {
            cardLp.height = cardHeight;
            card.setLayoutParams(cardLp);
        }
    }

    /** 点击列表项：先让点击波纹播完，再关闭弹窗并发起连接。 */
    private void onDevicePicked(DeviceItem item) {
        if (deviceAdapter == null || devicePickHandled) {
            return;
        }
        devicePickHandled = true;          // 防止连点重复发起连接
        mainHandler.postDelayed(() -> {
            dismissDeviceDialog();
            viewModel.connect(item);
        }, 140L);
    }

    private void dismissDeviceDialog() {
        if (deviceDialog != null && deviceDialog.isShowing()) {
            deviceDialog.dismiss();
        }
    }

    /* -------------------------------------------------------------- 弹窗与工具 */

    /**
     * A5：六轴门限 / 现场可调参数弹窗（固件 >= 1.3，寄存器 0x0033~0x0036 + 0x003A~0x003C）。
     * 打开时先按当前快照预填，同时向固件发一次高优先级读（requestParams），
     * 用户点“下发”后一次性写入全部字段（幂等），掉电由固件 EEPROM 保持。
     */
    private void showImuParamDialog() {
        if (!viewModel.paramSupported()) {
            Toast.makeText(this, "固件版本过低（需 >= 1.3），请先升级固件", Toast.LENGTH_LONG).show();
            return;
        }

        View content = getLayoutInflater().inflate(R.layout.dialog_imu_param, null);
        EditText etSlip = content.findViewById(R.id.etSlipTh);
        EditText etVib = content.findViewById(R.id.etVibTh);
        EditText etTilt = content.findViewById(R.id.etTiltTh);
        EditText etAlpha = content.findViewById(R.id.etAlphaLimit);
        EditText etOc = content.findViewById(R.id.etHeatOc);
        CheckBox cbAssist = content.findViewById(R.id.cbAssist);
        CheckBox cbTune = content.findViewById(R.id.cbTune);

        TextView hint = content.findViewById(R.id.tvParamHint);

        int[] p = viewModel.paramSnapshot();
        etSlip.setText(String.valueOf(p[0]));
        etVib.setText(String.valueOf(p[1]));
        etTilt.setText(String.valueOf(p[2]));
        etAlpha.setText(String.valueOf(p[3]));
        etOc.setText(String.valueOf(p[4]));
        cbAssist.setChecked(p[5] != 0);
        cbTune.setChecked(p[6] != 0);

        viewModel.requestParams();   // 后台刷一次最新值（轮询也会同步）

        /* 刚连上就打开弹窗时，0x0033~0x0036 / 0x003A 往往还没轮询到，快照全是 0。
         * 这里显示「读取中…」并每 400 ms 重试回填（最多 6 s），只填空/为 0 且用户没改过的框，
         * 避免用户正在输入的值被覆盖。 */
        final boolean[] touch = {false};
        etSlip.addTextChangedListener(new SimpleWatcher(() -> touch[0] = true));
        etVib.addTextChangedListener(new SimpleWatcher(() -> touch[0] = true));
        etTilt.addTextChangedListener(new SimpleWatcher(() -> touch[0] = true));
        etAlpha.addTextChangedListener(new SimpleWatcher(() -> touch[0] = true));
        etOc.addTextChangedListener(new SimpleWatcher(() -> touch[0] = true));

        final Handler ui = new Handler(Looper.getMainLooper());
        final int[] tries = {0};
        Runnable refill = new Runnable() {
            @Override
            public void run() {
                int[] q = viewModel.paramSnapshot();

                /* 逐字段回填：只填「已读到（非 0）且当前为空/0」的框。
                 * 关键：不能一读到某个字段就收工——0x0033/0x0034 在轮询第 3 帧到，
                 * 0x0035/0x0036 在第 4 帧、0x003A 在第 5 帧，提前 return 会把还没到的字段留成 0。 */
                if (!touch[0]) {
                    if (q[0] != 0 && isZeroOrBlank(etSlip))  etSlip.setText(String.valueOf(q[0]));
                    if (q[1] != 0 && isZeroOrBlank(etVib))   etVib.setText(String.valueOf(q[1]));
                    if (q[2] != 0 && isZeroOrBlank(etTilt))  etTilt.setText(String.valueOf(q[2]));
                    if (q[3] != 0 && isZeroOrBlank(etAlpha)) etAlpha.setText(String.valueOf(q[3]));
                    if (q[4] != 0 && isZeroOrBlank(etOc))    etOc.setText(String.valueOf(q[4]));
                    if (q[1] != 0 && q[2] != 0) {
                        cbAssist.setChecked(q[5] != 0);
                        cbTune.setChecked(q[6] != 0);
                    }
                }

                boolean allGot = q[0] != 0 && q[1] != 0 && q[2] != 0 && q[3] != 0 && q[4] != 0;
                if (allGot) {
                    hint.setText("实测整定（A2）建议先开统计打印，边跑边看串口 [TUNE] 行。");
                    return;
                }

                tries[0]++;
                if (tries[0] < 15) {
                    hint.setText("读取参数中…（" + tries[0] + "/15）");
                    ui.postDelayed(this, 400L);
                } else {
                    hint.setText("部分参数未读到（振动/倾斜/过流为 0）：检查固件 ≥1.3 后重开本页");
                }
            }
        };
        ui.postDelayed(refill, 400L);

        AlertDialog dialog = new AlertDialog.Builder(this)
                .setTitle("六轴门限 / 参数设置")
                .setView(content)
                .setPositiveButton("下发", (d, w) -> {
                    try {
                        viewModel.applyImuParams(
                                Integer.parseInt(etSlip.getText().toString().trim()),
                                Integer.parseInt(etVib.getText().toString().trim()),
                                Integer.parseInt(etTilt.getText().toString().trim()),
                                Integer.parseInt(etAlpha.getText().toString().trim()),
                                Integer.parseInt(etOc.getText().toString().trim()),
                                cbAssist.isChecked(),
                                cbTune.isChecked());
                    } catch (NumberFormatException e) {
                        Toast.makeText(this, "请输入整数（不要带单位/小数）", Toast.LENGTH_LONG).show();
                    }
                })
                .setNeutralButton("立即打印一行统计", (d, w) -> viewModel.writeParam(
                        ModbusProtocol.REG_TUNE_CTRL,
                        ModbusProtocol.TUNECTL_ONCE
                                | (cbTune.isChecked() ? ModbusProtocol.TUNECTL_PRINT : 0)))
                .setNegativeButton("取消", null)
                .create();

        dialog.show();
        applyDialogSize60(dialog);
    }

    /**
     * 弹窗尺寸统一规范（用户 2026-11-06 定稿，后续所有弹窗照此执行）：
     *   宽度 = 屏幕宽（系统默认边距）
     *   高度 = 屏幕高 × 60%，内容超出时**内部滚动**，不铺满全屏
     * 新增弹窗一律在 show() 之后调用本方法。
     */
    private void applyDialogSize60(Dialog dialog) {
        Window w = dialog.getWindow();
        if (w == null) {
            return;
        }
        int h = (int) (getResources().getDisplayMetrics().heightPixels
                * DEVICE_DIALOG_MAX_HEIGHT_RATIO);
        w.setLayout(ViewGroup.LayoutParams.MATCH_PARENT, h);
    }

    /** 输入框是否还是空的/0（只有这种状态才允许自动回填，避免覆盖用户输入）。 */
    private static boolean isZeroOrBlank(EditText et) {
        String t = et.getText().toString().trim();
        return t.isEmpty() || "0".equals(t);
    }

    /** 只关心“文本被改过”的最小 TextWatcher。 */
    private static class SimpleWatcher implements TextWatcher {
        private final Runnable onEdit;

        SimpleWatcher(Runnable onEdit) {
            this.onEdit = onEdit;
        }

        @Override
        public void beforeTextChanged(CharSequence s, int start, int count, int after) {
        }

        @Override
        public void onTextChanged(CharSequence s, int start, int before, int count) {
            onEdit.run();
        }

        @Override
        public void afterTextChanged(Editable s) {
        }
    }

    /** 启动被下位机联锁拒绝时，说明原因并提供清故障入口。 */
    private void showStartBlockedDialog(String message) {
        new AlertDialog.Builder(this)
                .setTitle("启动被拒绝：下位机保护联锁")
                .setMessage(message)
                .setPositiveButton("清故障并启动", (d, w) -> viewModel.clearFaultAndStart())
                .setNeutralButton("只清故障", (d, w) -> viewModel.clearFault())
                .setNegativeButton("取消", null)
                .show();
    }

    private void toast(String msg) {
        Toast.makeText(this, msg, Toast.LENGTH_SHORT).show();
    }

    private int dp(int v) {
        return Math.round(getResources().getDisplayMetrics().density * v);
    }

    /** 长按连发：按下立即执行一次，600ms 后每 250ms 重复（面板为 80ms，BLE 需留发送间隔）。 */
    private final class HoldRepeatListener implements View.OnTouchListener {
        private final Runnable action;
        private final Runnable repeat = new Runnable() {
            @Override
            public void run() {
                action.run();
                mainHandler.postDelayed(this, 250L);
            }
        };

        HoldRepeatListener(Runnable action) {
            this.action = action;
        }

        @Override
        @SuppressLint("ClickableViewAccessibility")
        public boolean onTouch(View v, MotionEvent e) {
            switch (e.getActionMasked()) {
                case MotionEvent.ACTION_DOWN:
                    v.setPressed(true);
                    action.run();
                    mainHandler.removeCallbacks(repeat);
                    mainHandler.postDelayed(repeat, 600L);
                    return true;
                case MotionEvent.ACTION_UP:
                    v.setPressed(false);
                    mainHandler.removeCallbacks(repeat);
                    v.performClick();
                    return true;
                case MotionEvent.ACTION_CANCEL:
                    v.setPressed(false);
                    mainHandler.removeCallbacks(repeat);
                    return true;
                default:
                    return false;
            }
        }
    }
}
