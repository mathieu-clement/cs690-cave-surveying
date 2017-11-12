cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

set (QT_QMAKE_EXECUTABLE "/opt/local/libexec/qt4/bin/qmake")

project      (pcl-visualizer)
find_package (Qt4 REQUIRED)
find_package (VTK REQUIRED)
find_package (PCL 1.7.1 REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set  (project_SOURCES   main.cpp pclviewer.cpp paramsdialog.cpp poissonparamsdialog.cpp greedyprojectiontriangulationparamsdialog.cpp)
set  (project_HEADERS   pclviewer.h params.h paramsdialog.h poissonparamsdialog.h greedyprojectiontriangulationparamsdialog.h)
set  (project_FORMS     pclviewer.ui paramsdialog.ui poissonparamsdialog.ui greedyprojectiontriangulationparamsdialog.ui)
set  (project_RESOURCES icons.qrc)
set  (VTK_LIBRARIES     vtkRendering vtkGraphics vtkHybrid QVTK)

QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT4_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})
QT4_ADD_RESOURCES ( project_RESOURCES ${project_RESOURCES})

INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (pcl_visualizer ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_RESOURCES}
                                ${project_HEADERS_MOC})

#if(APPLE)
#    set_target_properties (pcl_visualizer PROPERTIES
#        MACOSX_BUNDLE YES
#    )
#endif(APPLE)
TARGET_LINK_LIBRARIES (pcl_visualizer ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES})

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
