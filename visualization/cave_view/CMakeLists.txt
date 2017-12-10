cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 11)
set(QT_QMAKE_EXECUTABLE "/opt/local/libexec/qt5/bin/qmake")

project(pcl-visualizer)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)
find_package(Qt5Widgets REQUIRED)
find_package(VTK 8.0.1 REQUIRED NO_MODULE)
find_package(PCL 1.7.1 REQUIRED)

include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(project_SOURCES main.cpp pclviewer.cpp paramsdialog.cpp poissonparamsdialog.cpp greedyprojectiontriangulationparamsdialog.cpp marchingcubesparamsdialog.cpp paramsloader.cpp params.cpp)
set(project_HEADERS pclviewer.h params.h paramsdialog.h poissonparamsdialog.h greedyprojectiontriangulationparamsdialog.h marchingcubesparamsdialog.h paramsloader.h json.hpp)
set(project_FORMS pclviewer.ui paramsdialog.ui poissonparamsdialog.ui greedyprojectiontriangulationparamsdialog.ui marchingcubesparamsdialog.ui)
set(project_RESOURCES icons.qrc)

include(${VTK_USE_FILE})

QT5_WRAP_CPP(project_HEADERS_MOC ${project_HEADERS})
QT5_WRAP_UI(project_FORMS_HEADERS ${project_FORMS})
QT5_ADD_RESOURCES(project_RESOURCES ${project_RESOURCES})

include_directories(SYSTEM ${Qt5Widgets_INCLUDES})
add_definitions(${Qt5Widgets_DEFINITIONS})

ADD_EXECUTABLE(cave_viewer ${project_SOURCES}
        ${project_FORMS_HEADERS}
        ${project_RESOURCES}
        ${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES(cave_viewer Qt5::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})