#-------------------------------------------------
#
# Project created by QtCreator 2014-05-01T14:24:33
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = "Cave Viewer"
TEMPLATE = app
ICON = duck.icns

SOURCES += main.cpp\
        pclviewer.cpp \
    paramsdialog.cpp \
    poissonparamsdialog.cpp \
    greedyprojectiontriangulationparamsdialog.cpp \
    marchingcubesparamsdialog.cpp

HEADERS  += pclviewer.h \
    params.h \
    paramsdialog.h \
    poissonparamsdialog.h \
    greedyprojectiontriangulationparamsdialog.h \
    marchingcubesparamsdialog.h

FORMS    += pclviewer.ui \
    paramsdialog.ui \
    poissonparamsdialog.ui \
    greedyprojectiontriangulationparamsdialog.ui \
    marchingcubesparamsdialog.ui

RESOURCES += \
    icons.qrc
