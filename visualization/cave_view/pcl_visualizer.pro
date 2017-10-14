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
    mlsparamsdialog.cpp

HEADERS  += pclviewer.h \
    mlsparamsdialog.h \
    mlsparams.h

FORMS    += pclviewer.ui \
    mlsparamsdialog.ui
