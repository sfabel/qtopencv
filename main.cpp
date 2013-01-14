#include <QtGui>
#include <QApplication>
#include <GL/freeglut.h>
#include "mainwindow.h"

//==============================
//   New Project Yang Tony Jon
//==============================
int main(int argc, char *argv[])
{
    system("v4l2-ctl -i 2 -s ntsc"); //set channel to s-video and NTSC
    QApplication a(argc, argv);
    glutInit(&argc,argv);
    glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGB|GLUT_DEPTH|GLUT_MULTISAMPLE);
    MainWindow mw;
    mw.show();
    a.connect(&a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()));
    return a.exec();
}
