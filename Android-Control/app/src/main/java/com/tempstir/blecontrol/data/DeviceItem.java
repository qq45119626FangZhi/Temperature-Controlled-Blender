package com.tempstir.blecontrol.data;

/**
 * 一台扫描到的 BLE 设备的展示数据。
 *
 * 名称解析（BluetoothDevice.getName 走 Binder）在子线程完成，
 * 主线程只负责把已经准备好的列表交给 RecyclerView 刷新。
 */
public final class DeviceItem {

    public final String name;
    public final String address;
    public final int rssi;
    public final boolean target;      // 是否本项目下位机（按 MAC 判断）
    public final boolean preset;      // 首行"直连出厂模块"占位项

    public DeviceItem(String name, String address, int rssi, boolean target, boolean preset) {
        this.name = name;
        this.address = address;
        this.rssi = rssi;
        this.target = target;
        this.preset = preset;
    }

    /** 列表第一行：无需等待扫描，直接连出厂 MAC。 */
    public static DeviceItem preset(String mac) {
        return new DeviceItem("直连出厂模块  BT_ECB1A6B01BA2", mac, 0, true, true);
    }

    public String displayName() {
        if (preset) {
            return name;
        }
        return target ? name + "  (目标模块)" : name;
    }
}
