#include "mainwindow.h"
#include <QApplication>


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    QApplication a(argc, argv);
    a.setWindowIcon(QIcon(":/kart_1.png"));

    MainWindow w;
    w.show();

    return a.exec();
}
