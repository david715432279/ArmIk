#-------------------------------------------------
#
# Project created by QtCreator 2014-02-20T10:11:51
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

INCLUDEPATH += /home/user/Desktop/eigen

TARGET = IkTest1
TEMPLATE = app


SOURCES += main.cpp\
        widget.cpp ikfunc.cpp \
    robotinitform.cpp \
    armstate.cpp \
    common.c \
    bitratetest.cpp \
    arm_joint.c \

HEADERS  += widget.h ikfast.h \
    ikfunc.h \
    ikarmdesc.h \
    robotinitform.h \
    armstate.h \
    common.h \

LIBS +=  -L/home/user/Workplace/IkTest -lik -llapack
LIBS +=  -L/usr/lib -lpcan -DNO_RT

FORMS    += widget.ui \
    robotinitform.ui

RESOURCES += \
    res.qrc
