# QtOpenCV QMake File for Cross-Platform;

# for easy installation under WinXP
# - needs MSVC 2008 (Express) at time of this writing (11/24/2010)
# - adjust library and include paths below

TARGET = qtopencv
QT += core gui xml opengl

unix {
message("Building Linux Port of QtOpenCV")
LIBS += -lopencv_core \
    -lopencv_highgui \
    -llapack \
    -lblas \
    -lopencv_legacy \
    -lglut \
    -L/usr/local/lib

INCLUDEPATH += /usr/local/include
}

win32 {
message("Building Win32 Port of QtOpenCV")
LIBS += -L$$quote(C:/devel/armadillo-1.0.0/examples/lib_win32) \
        -L$$quote(C:/devel/OpenCV2.1/bin)\
        -L$$quote(C:/devel/OpenCV2.1/lib)\
        -lcv \
        -lcxcore \
        -lhighgui \
        -llapack_win32_MT \
        -lblas_win32_MT \
        -lcvaux

INCLUDEPATH += $$quote(C:/devel/armadillo-1.0.0/include) \
               $$quote(C:/devel/OpenCV2.1/include)
}

FORMS += qtopencv.ui \
    mainwindow.ui \
    openglscene.ui
RESOURCES +=
HEADERS += Image_Processing.h \
    qtopencv.h \
    pPOSIT.h \
    PoseLM.h \
    PoseEstimator.h \
    P3P.h \
    FindROI.h \
    mainwindow.h \
    openglscene.h \
    Reg3D.h \
    GRtool.h \
    math3d.h
SOURCES += Image_Processing.cpp \
    main.cpp \
    qtopencv.cpp \
    pPOSIT.cpp \
    PoseLM.cpp \
    PoseEstimator.cpp \
    P3P.cpp \
    mainwindow.cpp \
    openglscene.cpp \
    Reg3D.cpp \
    GRtool.cpp \
    math3d.cpp
