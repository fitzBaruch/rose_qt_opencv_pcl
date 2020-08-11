#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <iostream>
#include <math.h>
#include <boost/shared_ptr.hpp>
#include <QTimer>
#include "ui_mainwindow.h"
#include  "../include/qt_app/myviz.h"
#include <QLabel>
#include <QImage>



using namespace std;

namespace Ui {
class MainWindow;
}


class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();


signals:



private:
  Ui::MainWindow *ui;
  MyViz *rvizz;
  QImage *img;

private slots:


  void on_pushButton_clicked();
  void on_initSaveBtn_clicked();
  void on_modeSaveBtn_clicked();
};

#endif // MAINWINDOW_H
