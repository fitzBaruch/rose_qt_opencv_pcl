#include "../include/qt_app/LabelPress.h"
#include <iostream>
#include <QPoint>
#include <QLabel>
#include <QPainter>
#include <QPen>
LabelPress::LabelPress( QWidget *parent /*= 0*/ ):QLabel(parent)
{
      init();
}
 
LabelPress::LabelPress(const QString &text, QWidget *parent, 
	Qt::WindowFlags f):QLabel(text,parent,f)
{
      init();
}
 
void LabelPress::init()
{
    mouse_press = false;
	clicked_num = 0;

    this->startPoint = QPoint(0,0);
    this->endPoint = QPoint(0,0);
    this->lineColor = QColor(Qt::red);
    this->lineSize = 1;

	timer = new QTimer(this);
	connect(timer,SIGNAL(timeout()),this,SLOT(SlotTimerOut())); 
    qDebug()<<"this is mouse init fun";
    std::cout<<"hahha"<<std::endl;
}

void LabelPress::paintEvent(QPaintEvent *event)
{
    QLabel::paintEvent(event);
    QPainter painter(this);
    QPen pen;
    pen.setColor(lineColor);
    pen.setWidth(lineSize);
    painter.setPen(pen);
    painter.drawRect(QRect(startPoint.x(), startPoint.y()
                           , endPoint.x()-startPoint.x(), endPoint.y()-startPoint.y()));
}
 
 
LabelPress::~LabelPress()
{
	delete timer;
}
 
void LabelPress::mousePressEvent( QMouseEvent *event )
{
	if (event->button() == Qt::LeftButton)
	{
        startPoint = event->pos();
        isPressed = true;
	}
}
 
void LabelPress::mouseReleaseEvent( QMouseEvent *event )
{
    isPressed = false;
    update();
}

void LabelPress::mouseMoveEvent(QMouseEvent *event)
{
    if(isPressed){
        endPoint = event->pos();
        update();
    }
}
 


