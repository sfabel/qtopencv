#ifndef OPENGLSCENE_H
#define OPENGLSCENE_H

#include <QGLWidget>
#include <QTimer>
#include <QMouseEvent>

namespace Ui {
    class OpenGLScene;
}

struct TransfMat //rotate first, then translate
{
    GLfloat x;  //translate distance along x axis
    GLfloat y;
    GLfloat z;
    GLfloat rotx; //rad, rotation angle about x axis
    GLfloat roty;
    GLfloat rotz;
};

class OpenGLScene : public QGLWidget
{
    Q_OBJECT

    friend class MainWindow;

public:
    explicit OpenGLScene(QWidget *parent = 0);
    ~OpenGLScene();


private slots:
    void initializeGL();
    void resizeGL(int width, int height);
    void paintGL();

private:
    Ui::OpenGLScene *ui;
    QTimer *timer;

    GLfloat rotationX;
    GLfloat rotationY;
    GLfloat rotationZ;
    GLfloat view_distance;

    int AUTO_CAL_X;
    int AUTO_CAL_Y;

    QPoint lastPos;


    //void drawCylinder();
    //void drawWrist();
    //void drawEndoscope();
    void flushView();
    void drawBox(GLdouble size);
    void drawSphere(GLdouble size, GLfloat x, GLfloat y, GLfloat z);
    void drawManipulator(GLdouble size_factor,  struct TransfMat *t);
    void drawGripper(GLdouble size_factor, GLfloat grip_angle, struct TransfMat *t);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
};

#endif // OPENGLSCENE_H
