#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "RosNode.h"
#include <QMainWindow>
#include <QTimer>
#include <thread>

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

signals:
    //    void show_battery(uint8_t power);

private slots:
    void on_show_fishbotBattery();


private:
    Ui::MainWindow* ui;

    QTimer* timer;

    std::thread mythread;

    std::shared_ptr<RosNode> node = nullptr;
    //    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscripstion = nullptr;

    //    void callback_subscription(const std_msgs::msg::UInt8::SharedPtr msg);

    void task_spin_ros();
};

#endif   // MAINWINDOW_H
