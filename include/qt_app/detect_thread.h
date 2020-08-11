#ifndef DETECT_THREAD
#define DETECT_THREAD

#include <QObject>
#include <QThread>
#include <qt_app/tripod_detection.h>
#include <Eigen/Eigen>
#include <QMutex>
#include <QWaitCondition>


using namespace detection;

class DetectThread : public QThread {
    Q_OBJECT
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DetectThread();
private:

}

#endif
