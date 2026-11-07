package com.tempstir.blecontrol.viewmodel;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

/**
 * 一次性事件包装：LiveData 会在页面重建时重放最后一个值，
 * 用 Event 保证 Toast / 弹窗这类"只处理一次"的动作不会被重复消费。
 */
public final class Event<T> {

    private final T content;
    private boolean handled;

    public Event(@NonNull T content) {
        this.content = content;
    }

    /** 未消费过则返回内容并标记为已消费，否则返回 null。 */
    @Nullable
    public T getContentIfNotHandled() {
        if (handled) {
            return null;
        }
        handled = true;
        return content;
    }

    @NonNull
    public T peekContent() {
        return content;
    }
}
