cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 11)
#set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-inconsistent-missing-override")
#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-inconsistent-missing-override")
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

#set  (VTK_LIBRARIES     vtkRendering vtkGraphics vtkHybrid QVTK)
include(${VTK_USE_FILE})

QT5_WRAP_CPP(project_HEADERS_MOC ${project_HEADERS})
QT5_WRAP_UI(project_FORMS_HEADERS ${project_FORMS})
QT5_ADD_RESOURCES(project_RESOURCES ${project_RESOURCES})

include_directories(SYSTEM ${Qt5Widgets_INCLUDES})
add_definitions(${Qt5Widgets_DEFINITIONS})

ADD_EXECUTABLE(pcl_visualizer ${project_SOURCES}
        ${project_FORMS_HEADERS}
        ${project_RESOURCES}
        ${project_HEADERS_MOC})

#if(APPLE)
#    set_target_properties (pcl_visualizer PROPERTIES
#        MACOSX_BUNDLE YES
#    )
#endif(APPLE)
TARGET_LINK_LIBRARIES(pcl_visualizer Qt5::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})

#if(APPLE)
#    add_custom_command(
#        TARGET  pcl_visualizer POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E copy
#            ${CMAKE_SOURCE_DIR}/Info.plist
#            ${CMAKE_CURRENT_BINARY_DIR}/pcl_visualizer.app/Contents/Info.plist)
#    add_custom_command(
#        TARGET  pcl_visualizer POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E make_directory
#            ${CMAKE_CURRENT_BINARY_DIR}/pcl_visualizer.app/Contents/Resources/)
#    add_custom_command(
#        TARGET  pcl_visualizer POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E copy
#            ${CMAKE_SOURCE_DIR}/duck.icns
#            ${CMAKE_CURRENT_BINARY_DIR}/pcl_visualizer.app/Contents/duck.icns)
#    add_custom_command(
#        TARGET  pcl_visualizer POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E copy
#            ${CMAKE_SOURCE_DIR}/duck.icns
#            ${CMAKE_CURRENT_BINARY_DIR}/pcl_visualizer.app/Contents/Resources/duck.icns)
#endif(APPLE)
