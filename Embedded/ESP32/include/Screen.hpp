/**
 * @brief Show information on the OLED screen
 * @file Dispaly.hpp
 * @author Avent
 * @date 2023-05-14
 */
#pragma once
#include <Adafruit_SSD1306.h>   // 加载Adafruit_SSD1306库（OLED库）

/**
 *  不能使用宏定义作为参数传入Adafruit_SSD1306的构造函数，
 *  具体原因未知
 */
// #define OLED_WIDTH  128
// #define OLED_HEIGHT 64

class Screen
{
public:
    // Screen() = delete;   // 禁止默认构造函数
    Screen() = default;
    explicit Screen(TwoWire& wire);
    Screen(const Screen&) = delete;   // 禁止拷贝构造函数
    ~Screen()             = default;

    void showBatteryPower();

public:
    uint8_t battery_power;

private:
    Adafruit_SSD1306 oled;
    char battery_str[12];
};

Screen::Screen(TwoWire& wire)
{
    oled = Adafruit_SSD1306(oled_width, oled_height, &wire);
    oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);   // 初始化OLED
    oled.clearDisplay();                      // 清空屏幕
    oled.setTextSize(2);                      // 设置字体大小，最小为1
    oled.setTextColor(SSD1306_WHITE);         // 设置字体颜色
    oled.setCursor(0, 0);                     // 设置开始显示文字的坐标
    oled.println("hello!");                   // 输出的字符
    oled.display();
}

void Screen::showBatteryPower()
{
    int analogValue = analogRead(ADC_PIN);                  // 读取原始值0-4096
    int analogVolts = analogReadMilliVolts(ADC_PIN);        // 读取模拟电压，单位毫伏
    float realVolts = 5.02 * ((float)analogVolts * 1e-3);   // 计算实际电压值

    battery_power = realVolts * 100.0f / 5.1f;              // 电压映射,将电压值映射到0-100%

    sprintf(battery_str, "Power:%03d%%", battery_power);

    oled.clearDisplay();         // 清空屏幕
    oled.setTextSize(2);         // 设置字体大小，最小为1
    oled.setCursor(0, 0);        // 设置开始显示文字的坐标
    oled.println(battery_str);   // 输出的字符
    oled.display();
}
