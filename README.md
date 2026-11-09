# Temperature-Controlled-Blender
STM32 温控搅拌机，分下位机C项目，上位机android, 由AI辅助编程，使用最多的模型是DeepSeek

下位机

TempStir

产品与功能目标

智能温控搅拌机，核心功能：

1. **温度采集与闭环加热**：板载 NTC + 液体 NTC 双路测温，PWM 驱动加热组件，PID 控温，过温/断线/过流保护。
2. **搅拌电机调速**：编码器测速 + PWM 调速闭环，软启动、限幅、堵转保护。
3. **人机界面**：0.96" OLED（SSD1315，128×64）+ 4 按键，多级菜单设置目标温度/转速/时间。
4. **参数与时间**：M24C02 EEPROM 保存参数；DS3231 RTC 提供实时时间与秒中断。
5. **提示与散热**：蜂鸣器（PWM 4kHz，短鸣/长鸣/告警音）；风扇按温度阈值启停。
6. **通信**：蓝牙（UART4）通信
7. **故障与保护**：硬件故障锁存（74LVC1G74）读取与复位、故障停机与告警。

<img width="1280" height="2837" alt="60d6cbfce376dead50d187aa65770320" src="https://github.com/user-attachments/assets/741d17aa-c808-419e-a0c9-987517cf5896" />


下位机 
Temperature-Controlled-Blender
<img width="1190" height="620" alt="5c0878e454f9890c70e8adebddd8f827" src="https://github.com/user-attachments/assets/309e9157-f4e2-4baa-848c-d3df31182d01" />


1. 产品与硬件要点

- 智能温控搅拌机，MCU = STM32F103RCT6，FreeRTOS + HAL + Keil MDK-ARM。
- 人机界面：0.96" SSD1315 OLED + **4 个物理按键**（PC6/PC7/PC8/PC9，内部上拉、按下为低）。
- 三通道并存：蓝牙 BLE 
- 蓝牙芯片 **ECB01CV2-SOP8**（早期文档写的 ECB01C-TSSOP16 已更正），接 **USART2：PA2(TX→BL-RXD) / PA3(RX←BL-TXD)**，实测波特率 **9600 8N1**（上电 AT 自动识别，非手册默认的 115200）。
- 蓝牙模块出厂名 `BT_ECB1A6B01BA2`，MAC `ECB1A6B01BA2`；不要用系统蓝牙"配对"。

<img width="1260" height="3606" alt="7471e8e7c87160ca152dacf2247e7099" src="https://github.com/user-attachments/assets/30480b59-07d6-4568-a917-ac315c9649a1" />


