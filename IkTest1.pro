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

HEADERS  += widget.h ikfast.h \
    ikfunc.h \
    ikarmdesc.h

LIBS +=  -L/home/user/Workspace/IkTest -lik

FORMS    += widget.ui
