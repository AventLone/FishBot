#include "mainwindow.h"
#include "../ui/ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    timer = new QTimer();

    node = std::make_shared<RosNode>("fishbot_ui");


    mythread = std::thread(&MainWindow::task_spin_ros, this);

    connect(timer, SIGNAL(timeout()), this, SLOT(on_show_fishbotBattery()));
    //    connect(this, SIGNAL(show_battery(uint8_t)), this, SLOT(on_show_battery(uint8_t)));
    ui->progressBar->setValue(100);

    timer->start(500);
}

MainWindow::~MainWindow()
{
    delete ui;
    rclcpp::shutdown();
    mythread.join();
}

void MainWindow::on_show_fishbotBattery()
{
    //    QString qst = QString::number(node->volts, 'f', 3);
    //    QString qst = QString::number(power);

    //    ui->label_batteey->setText(qst);
    ui->progressBar->setValue(node->message_battry);
}

// void MainWindow::callback_subscription(const std_msgs::msg::UInt8::SharedPtr msg)
//{
//     emit show_battery(msg->data);
// }

void MainWindow::task_spin_ros()
{
    rclcpp::spin(node);
}
