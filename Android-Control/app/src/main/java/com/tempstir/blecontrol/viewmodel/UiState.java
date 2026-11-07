package com.tempstir.blecontrol.viewmodel;

/**
 * 首页一次性渲染所需的完整 UI 状态（不可变语义：ViewModel 每次刷新都新建一份）。
 * View 层只负责把这里的字符串/布尔量贴到控件上，不再做任何计算。
 */
public final class UiState {

    public String connText = "未连接";
    public String focusLabel = "";
    public String focusValue = "--";
    public String focusHint = "";
    public String liquidText = "--";
    public String speedText = "--";
    public String plateText = "--";
    public String timeText = "--";
    public String percentText = "--";
    public String statusText = "--";
    public String faultText = "--";

    /* P9：倾斜角（= max(|roll|,|pitch|)，超门限时 tiltAlarm=true 用于红色显示） */
    public String tiltText = "--";
    public boolean tiltAlarm;

    /* P9：IMU / 斜坡（六轴仪表区） */
    public String imuAccText = "--";
    public String imuGyroText = "--";
    public String imuAttText = "--";
    public String imuMiscText = "--";

    /* A4/A2：参数页预填值（固件 >= 1.3 才有） */
    public boolean paramSupported;      // 固件是否支持 0x003A~0x003C
    public int heatOcRaw = 0;           // 0x003A 加热过流阈值 ADC raw
    public boolean assistOn;            // 0x003B bit0 振动辅助堵转
    public boolean tuneOn;              // 0x003C bit0 实测统计打印

    public boolean fault;
    public boolean running;

    UiState() {
    }
}
