#include "openglscene.h"
#include "ui_openglscene.h"
#include <GL/freeglut.h>
#include "Image_Processing.h"
#include "qtopencv.h"
#include "GRtool.h"


OpenGLScene::OpenGLScene(QWidget *parent) :
    QGLWidget(parent),
    ui(new Ui::OpenGLScene)
{
    ui->setupUi(this);

    setFormat(QGLFormat(QGL::DoubleBuffer | QGL::DepthBuffer));

    rotationX=0.0;
    rotationY=0.0;
    rotationZ=0.0;
    view_distance = -100;

    timer = new QTimer(this);
    timer->setInterval(50);
    connect(timer,SIGNAL(timeout()),this,SLOT(updateGL()));
    timer->start();
}

OpenGLScene::~OpenGLScene()
{
    delete ui;
}

void OpenGLScene::initializeGL()
{
    qglClearColor(Qt::gray);
    glEnable(GL_DEPTH_TEST);
    glFrontFace(GL_CCW);
    glEnable(GL_CULL_FACE);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_ALPHA_TEST);


    GLfloat diffuseMaterial[4]={0.5, 0.5, 0.5, 1.0};
    GLfloat mat_specular[]={1.0, 1.0, 1.0, 1.0};
    GLfloat light_position[]={-1.0, 3.0, 3.0, 0.0};
    GLfloat light_ambient[]={1, 1, 1, 0.1};
    GLfloat white_light[] = {0.5f, 0.5f, 0.5f, 1.0f};
    //Enable lighting
    glEnable(GL_LIGHTING);

    //Setup and enable light 0
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, white_light);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuseMaterial);
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);

    glEnable(GL_LIGHT0);
    glMaterialf(GL_FRONT, GL_SHININESS, 50.0);

    glEnable(GL_LIGHTING);

    glColorMaterial(GL_FRONT, GL_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);


    resizeGL(845, 580);
}

void OpenGLScene::resizeGL(int w, int h)
{
GLfloat fAspect;
//Prevent a devide by zero
if (h==0) h=1;
//Set viewport to window dimensions
glViewport(0,0,w,h);
fAspect = (GLfloat)w/(GLfloat)h;
//Reset coordinate system
glMatrixMode(GL_PROJECTION);
glLoadIdentity();
//Produce the perspective projection
gluPerspective(60.0f, fAspect, 1.0, 400.0);

glMatrixMode(GL_MODELVIEW);
glLoadIdentity();


#ifdef __ORTHO__
#define
    GLfloat nRange = 100.0f;
    //Prevent a divide by Zero
    if (0==h)
        h = 1;

    //Set Viewport to window dimensions
    glViewport(0, 0, w, h);
    //Reset projection matrix stack
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //Establish clipping volume (left, right, bottom, top, near, far)
    if (w <= h)
        glOrtho(-nRange, nRange, -nRange*h/w, nRange*h/w, -nRange, nRange);
    else
        glOrtho(-nRange*w/h, nRange*w/h, -nRange, nRange, -nRange, nRange);

    //Reset Model view matrix stack
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
#endif

}

void OpenGLScene::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    flushView(); //change camera view accroding to mouse Events;
    glColor3f(0.1, 0.1, 0.1);
    glutWireCube(80.0f);

    grDrawGround(40, 40, -40);


    GLfloat x, y, z;
    x = (AUTO_CAL_X-320)/6.4; //AUTO_CAL_Y=0;
    y = (AUTO_CAL_Y-240)/4.8;
    z = -80;
    drawSphere(10, x, y, z);

    struct TransfMat t;
    t.rotx = 2; t.roty=0; t.rotz=-30; t.x=-10; t.y=0; t.z=0;
//    for (t.rotz = 0; t.rotz < 360; t.rotz+=30)
//        for (t.roty = 0; t.roty < 360; t.roty+=30)
//      this->drawManipulator(1.0, &t);
      this->drawManipulator(1.0f, &t);

      drawGripper(1.0f, 45.0, &t);
    t.rotx = 0; t.roty=180; t.rotz=-30; t.x=0; t.y=0; t.z=0;
      this->drawManipulator(1.0f, &t);
      drawGripper(1.0f, 10.0, &t);

}

void OpenGLScene::flushView()
{
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(0, 0, view_distance);
        glRotatef(rotationX*3.14, 0.0f, 1.0f, 0.0f);
        glRotatef(rotationY*3.14, 1.0f, 0.0f, 0.0f);
}

void OpenGLScene::drawSphere(GLdouble size, GLfloat x, GLfloat y, GLfloat z)
{
    glPushMatrix();
      glMatrixMode(GL_MODELVIEW);
       glTranslatef(x, y, z);
       glColor3f(0.8, 0.4, 0.2);
       glutSolidSphere(size, 20, 16);
    glPopMatrix();
}

void OpenGLScene::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();
}

void OpenGLScene::mouseMoveEvent(QMouseEvent *event)
{
    GLfloat dx = GLfloat(event->x() - lastPos.x()) / width();
    GLfloat dy = GLfloat(event->y() - lastPos.y()) / height();

    if(event->buttons() & Qt::LeftButton) {
        rotationX += 180 * dx;
        rotationY += 180 * dy;
        updateGL();
    }
    else if(event->buttons() & Qt::RightButton) {
        rotationX += 180 * dy;
        rotationZ += 180 * dx;
    }
    cout << "rotateX, Y = " << rotationX <<", "<< rotationY<<endl;
    lastPos = event->pos();
    cout << "dx, dy = " << dx <<", "<< dy <<endl;
}

void OpenGLScene::wheelEvent(QWheelEvent *event)
{
    view_distance += 4*event->delta()/8.0/15.0;
    cout << "mouseWheet = " << view_distance << endl;
}

void OpenGLScene::drawManipulator(GLdouble size_factor,  struct TransfMat *t)
{
    //define the geometric shape of manipulator in (cm)
    GLfloat shaft_len = 28.5;
    GLfloat shaft_dia = 0.6/2;
    GLfloat motor_group_len = 3+2.95+4.45+0.2;
    GLfloat motot_group_dia = 6/2;
    GLfloat single_motor_len = 4.45;


    glPushMatrix();
      glMatrixMode(GL_MODELVIEW);
      glRotatef(t->rotx, 1, 0, 0);
      glRotatef(t->roty, 0, 1, 0);
      glRotatef(t->rotz, 0, 0, 1);
      glTranslatef(t->x, t->y, t->z);

      glTranslatef(-shaft_len/2, 0, 0);
      //Draw a big cylinder as motor group
      glPushMatrix();
         glColor3f(0.5, 0.5, 0.2);
         glTranslatef(-(motor_group_len/2+shaft_len/2), 0, 0);
         glRotatef(90, 0, 1, 0);
         grDrawSolidCylinder(motot_group_dia*size_factor, motot_group_dia*size_factor, motor_group_len*size_factor);
      glPopMatrix();

      //Draw a single motor
      glPushMatrix();
         glColor3f(0.01, 0.01, 0.01);
         glTranslatef(-(single_motor_len/2+motor_group_len+shaft_len/2), 0, 1.7);
         glRotatef(90, 0, 1, 0);
         grDrawSolidCylinder(1*size_factor, 1*size_factor, single_motor_len*size_factor);
      glPopMatrix();

      //Draw a long thin cylinder as shaft
      glPushMatrix();
        glColor3f(0.5, 0.5, 0.1);
        glTranslatef(0, 0, 0);
        glRotatef(90, 0, 1, 0);
        grDrawSolidCylinder(shaft_dia*size_factor, shaft_dia*size_factor, shaft_len*size_factor);
      glPopMatrix();

    glPopMatrix();
}

void OpenGLScene::drawGripper(GLdouble size_factor, GLfloat grip_angle, struct TransfMat *t)
  {
      //define the geometric shape of gripper and wrist in (cm)
      GLfloat gripper_len = 1.45;
      GLfloat driving_len = 1.1;
      GLfloat wrist_len = 1.28;
      GLfloat driving_dia = 0.6/2;
      GLfloat wrist_dia = 0.6/2;


      glPushMatrix();
        glMatrixMode(GL_MODELVIEW);
        glRotatef(t->rotx, 1, 0, 0);
        glRotatef(t->roty, 0, 1, 0);
        glRotatef(t->rotz, 0, 0, 1);
        glTranslatef(t->x, t->y, t->z);

        glTranslatef(driving_len+wrist_len, 0 ,0);
        //Draw two cube as gripper
        glPushMatrix();
           glRotatef(-grip_angle/2, 0, 1, 0);
           glTranslatef(gripper_len/2, 0, 0.1f*gripper_len/2);
           glColor3f(0.3, 0.03, 0.0);
           glScalef(gripper_len, 0.2f*gripper_len, 0.1f*gripper_len);
           glutSolidCube(1.0f);
        glPopMatrix();
        glPushMatrix();
           glRotatef(grip_angle/2, 0, 1, 0);
           glTranslatef(gripper_len/2, 0, -0.1f*gripper_len/2);
           glScalef(gripper_len, 0.2f*gripper_len, 0.1f*gripper_len);
           glutSolidCube(1.0f);
        glPopMatrix();
        //Draw gripper driving
        glPushMatrix();
           glColor3f(0.5, 0.5, 0.2);
           glTranslatef(-(driving_len/2), 0, 0);
           glRotatef(90, 0, 1, 0);
           grDrawSolidCylinder(driving_dia*size_factor, driving_dia*size_factor, driving_len*size_factor);
        glPopMatrix();
        //Draw wrist
        glPushMatrix();
           glColor3f(0.5, 0.5, 0.2);
           glTranslatef(-((driving_len+wrist_len)/2), 0, 0);
           //glRotatef(90, 0, 1, 0);
           //grDrawSolidCylinder(wrist_dia*size_factor, wrist_dia*size_factor, wrist_len*size_factor);
           glTranslatef(-wrist_len/7, 0, 0);
           glutSolidSphere(wrist_dia, 10, 10);
           glTranslatef(-wrist_len/2.5, 0, 0);
           glutSolidSphere(wrist_dia, 10, 10);
           glTranslatef(-wrist_len/2.5, 0, 0);
           glutSolidSphere(wrist_dia, 10, 10);
        glPopMatrix();

        glPopMatrix();

  }
