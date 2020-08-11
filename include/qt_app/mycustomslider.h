#ifndef MYCUSTOMSLIDER_H  
#define MYCUSTOMSLIDER_H  
#include <QSlider>  
#include <QLabel>  
#include <QMouseEvent>  
class MyCustomSlider : public QSlider  
{  
public:  
    MyCustomSlider(QWidget *parent=0);  
    ~MyCustomSlider();  
  
protected:  
    virtual void mousePressEvent(QMouseEvent *event);  
    virtual void mouseReleaseEvent(QMouseEvent *event);  
    virtual void mouseMoveEvent(QMouseEvent *event);  
  
private:  
    QLabel* m_displayLabel;  
};  
  
#endif // MYCUSTOMSLIDER_H 
