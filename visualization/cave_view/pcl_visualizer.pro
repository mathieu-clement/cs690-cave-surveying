#-------------------------------------------------
#
# Project created by QtCreator 2014-05-01T14:24:33
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = pcl_visualizer
TEMPLATE = app


SOURCES += main.cpp\
        pclviewer.cpp \
    fileloader.cpp

HEADERS  += pclviewer.h \
    fileloader.h \
    xyzpoint.h

FORMS    += pclviewer.ui
