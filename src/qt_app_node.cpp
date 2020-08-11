#include <ros/ros.h>
#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "triangle_detection_app_node", ros::init_options::AnonymousName );
  }

    QApplication a(argc, argv);
    MainWindow w;
    w.setWindowTitle("三脚架检测");
    w.setWindowState(Qt::WindowMaximized);
    w.show();

    return a.exec();
}
