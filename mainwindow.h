#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>
#include <QFileDialog>
#include <QMessageBox>

namespace Ui {
    class MainWindow;
}

class qtopencv;
class OpenGLScene;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void loadImage();
    void loadVideo();
    void loadCamera();
    void aboutDlg();
    void updatePrincipalPoint();

private:
    Ui::MainWindow *ui;
    qtopencv *ocv;
    OpenGLScene *ogl;
};

#endif // MAINWINDOW_H
