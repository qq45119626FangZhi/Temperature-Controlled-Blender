package com.tempstir.blecontrol.ui;

import android.view.LayoutInflater;
import android.view.ViewGroup;

import androidx.annotation.NonNull;
import androidx.recyclerview.widget.DiffUtil;
import androidx.recyclerview.widget.ListAdapter;
import androidx.recyclerview.widget.RecyclerView;

import com.tempstir.blecontrol.data.DeviceItem;
import com.tempstir.blecontrol.databinding.ItemDeviceBinding;

/**
 * 设备列表适配器：名称 + MAC 双行显示，点击即有波纹反馈并直接连接。
 * 使用 ViewBinding 绑定 item_device.xml，不再 findViewById。
 */
public final class DeviceAdapter extends ListAdapter<DeviceItem, DeviceAdapter.DeviceViewHolder> {

    public interface OnPick {
        void onPick(DeviceItem item);
    }

    private static final DiffUtil.ItemCallback<DeviceItem> DIFF =
            new DiffUtil.ItemCallback<DeviceItem>() {
                @Override
                public boolean areItemsTheSame(@NonNull DeviceItem a, @NonNull DeviceItem b) {
                    return a.address.equals(b.address);
                }

                @Override
                public boolean areContentsTheSame(@NonNull DeviceItem a, @NonNull DeviceItem b) {
                    return a.displayName().equals(b.displayName())
                            && a.rssi == b.rssi;
                }
            };

    private final OnPick onPick;

    public DeviceAdapter(OnPick onPick) {
        super(DIFF);
        this.onPick = onPick;
    }

    @NonNull
    @Override
    public DeviceViewHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {
        ItemDeviceBinding binding = ItemDeviceBinding.inflate(
                LayoutInflater.from(parent.getContext()), parent, false);
        return new DeviceViewHolder(binding);
    }

    @Override
    public void onBindViewHolder(@NonNull DeviceViewHolder holder, int position) {
        DeviceItem item = getItem(position);
        if (item == null) {
            return;
        }
        holder.binding.tvDevName.setText(item.displayName());
        holder.binding.tvDevMac.setText(item.address);
        holder.binding.getRoot().setOnClickListener(v -> onPick.onPick(item));
    }

    static final class DeviceViewHolder extends RecyclerView.ViewHolder {
        final ItemDeviceBinding binding;

        DeviceViewHolder(ItemDeviceBinding binding) {
            super(binding.getRoot());
            this.binding = binding;
        }
    }
}
