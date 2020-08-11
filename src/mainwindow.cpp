#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QDebug>
#include <QHostAddress>
#include <time.h>
#include <QString>
#include <QFile>

//#include "qt_app/myviz.h"
#include <QPixmap>

using namespace std;



MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);
    rvizz = new MyViz;
    img = new QImage;
    img->load("/home/fitz/Pictures/mouse.jpg");
    ui->label_20->setPixmap(QPixmap::fromImage(*img));

    connect(ui->initXSlider,&QSlider::valueChanged,[&](int index){
        ui->ixlabel->setText(QString::number(index));});
    connect(ui->initYSlider,&QSlider::valueChanged,[&](int index){
        ui->iylabel->setText(QString::number(index));});
    connect(ui->initZSlider,&QSlider::valueChanged,[&](int index){
        ui->izlabel->setText(QString::number(index));});
    connect(ui->initrollSlider,&QSlider::valueChanged,[&](int index){
        ui->iRlabel->setText(QString::number(index));});
    connect(ui->initPitchSlider,&QSlider::valueChanged,[&](int index){
        ui->iplabel->setText(QString::number(index));});
    connect(ui->initYawSlider,&QSlider::valueChanged,[&](int index){
        ui->iYawlabel->setText(QString::number(index));});

    connect(ui->modeXSlider,&QSlider::valueChanged,[&](int index){
        ui->mxlabel->setText(QString::number(index));});
    connect(ui->modeYSlider,&QSlider::valueChanged,[&](int index){
        ui->mylabel->setText(QString::number(index));});
    connect(ui->modeZSlider,&QSlider::valueChanged,[&](int index){
        ui->mzlabel->setText(QString::number(index));});
    connect(ui->moderollSlider,&QSlider::valueChanged,[&](int index){
        ui->mRlabel->setText(QString::number(index));});
    connect(ui->modePitchSlider,&QSlider::valueChanged,[&](int index){
        ui->mplabel->setText(QString::number(index));});
    connect(ui->modeYawSlider,&QSlider::valueChanged,[&](int index){
        ui->mYawlabel->setText(QString::number(index));});


}

MainWindow::~MainWindow()
{
  delete ui;
}


void MainWindow::on_pushButton_clicked()
{
    rvizz->EnableRawPointCloud2("/livox/lidar_1HDDH1200101761");
    qDebug()<<"fghjk";

}

void MainWindow::on_initSaveBtn_clicked()
{

    int x = ui->initXSlider->value();
    int y = ui->initYSlider->value();
    int z = ui->initZSlider->value();
    int roll = ui->initrollSlider->value();
    int pitch = ui->initPitchSlider->value();
    int yaw = ui->initYawSlider->value();
    qDebug()<<x<<y<<z<<roll<<pitch<<yaw;
    QFile data("/home/fit/catkin_ws/src/triangle_detection_app/src/init_guess.txt");
    if (data.open(QFile::WriteOnly | QFile::Truncate)) {
        QTextStream out(&data);
        out << qSetFieldWidth(10) << x << y << z << roll << pitch << yaw ;
    }
}

void MainWindow::on_modeSaveBtn_clicked()
{
    int x = ui->modeXSlider->value();
    int y = ui->modeYSlider->value();
    int z = ui->modeZSlider->value();
    int roll = ui->moderollSlider->value();
    int pitch = ui->modePitchSlider->value();
    int yaw = ui->modeYawSlider->value();
    qDebug()<<x<<y<<z<<roll<<pitch<<yaw;
    QFile data("/home/fit/catkin_ws/src/triangle_detection_app/src/model.txt");
    if (data.open(QFile::WriteOnly | QFile::Truncate)) {
        QTextStream out(&data);
        out << qSetFieldWidth(10) << x << y << z << roll << pitch << yaw ;
        qDebug()<<"45678";
    }

}
