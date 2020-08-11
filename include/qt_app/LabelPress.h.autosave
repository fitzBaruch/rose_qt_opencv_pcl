#ifndef LabelPress_H
#define LabelPress_H
//#include "QLabel"
#include <QtWidgets> 
//#include "qcoreevent.h"
//#include "qevent.h"
//#include "qtimer.h"
#include <QLabel>
#include <QTimer>
#include <QMouseEvent>
#include <QDebug>
#include <QPoint>
#include <QColor>

class LabelPress:public QLabel
{
    //Q_OBJECT
public:
     LabelPress(QWidget *parent = 0);//explicit QLabel(const QString &text, QWidget *parent=0, Qt::WindowFlags f=0);
     LabelPress(const QString &text, QWidget *parent=0, Qt::WindowFlags f=0);
	~LabelPress();
private:
	void init();
public:
    virtual void paintEvent(QPaintEvent *event) override;
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
private:
	bool mouse_press; 
	int clicked_num;
    QPoint startPoint;
    QPoint endPoint;
    QColor lineColor;               //线条颜色
    int lineSize;                  //5种线型
    bool isPressed;


	QTimer* timer;

};
 
 
#endif //LabelPress_H

