QT += core gui widgets

TARGET = DpSim
TEMPLATE = app
CONFIG += c++14

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

HEADERS += \
#    controller_interface.h \
#    mooring_simulation.h \
#    observer_interface.h \
##    pm_setpoint_interface.h \
#    simulation.h \
#    simulation_interface.h \
#    plant.h \
#    controller.h \
#    observer.h\
    env_load_interface.h\


SOURCES += main.cpp \
#    controller_interface.cpp \
#    mooring_simulation.cpp \
#    observer_interface.cpp \
##    pm_setpoint_interface.cpp \
#    simulation.cpp \
#    simulation_interface.cpp \
#    plant.cpp \
#    controller.cpp \
#    observer.cpp\
    env_load_interface.cpp\


INCLUDEPATH+= C:\C++library\boost_1_65_1\boost_1_65_1
INCLUDEPATH+= C:\C++library\eigen3\eigen3\Eigen
INCLUDEPATH += C:\C++library\gsl\include

# LIBS += -L"C:/Program Files/GSL/lib" -lgsl -lgslcblas -lm
LIBS += -L C:\C++library\gsl\lib -llibgsl -llibgslcblas -lm

#win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../DpController/release/ -lDpController0
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../DpController/debug/ -lDpController0

#INCLUDEPATH += $$PWD/../DpController
#DEPENDPATH += $$PWD/../DpController

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../DpEnvironment/release/ -lDpEnvironment0
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../DpEnvironment/debug/ -lDpEnvironment0

INCLUDEPATH += $$PWD/../DpEnvironment
DEPENDPATH += $$PWD/../DpEnvironment

#win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../DpMooring/release/ -lDpMooring0
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../DpMooring/debug/ -lDpMooring0

#INCLUDEPATH += $$PWD/../DpMooring
#DEPENDPATH += $$PWD/../DpMooring

#win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../DpObserver/release/ -lDpObserver0
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../DpObserver/debug/ -lDpObserver0

#INCLUDEPATH += $$PWD/../DpObserver
#DEPENDPATH += $$PWD/../DpObserver

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../DpVessel/release/ -lDpVessel0
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../DpVessel/debug/ -lDpVessel0

INCLUDEPATH += $$PWD/../DpVessel
DEPENDPATH += $$PWD/../DpVessel

#win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../DpMsg/release/ -lDpMsg
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../DpMsg/debug/ -lDpMsg

#INCLUDEPATH += $$PWD/../DpMsg
#DEPENDPATH += $$PWD/../DpMsg

#win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../DpGui/release/ -lDpGui0
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../DpGui/debug/ -lDpGui0

#INCLUDEPATH += $$PWD/../DpGui
#DEPENDPATH += $$PWD/../DpGui


win32 {
    RC_ICONS = skloe.ico
    VERSION = 0.1.2.180911
    QMAKE_TARGET_COPYRIGHT = "Copyright (c) 2018, Bo Li"
}

RESOURCES += \
    dpsim_resource.qrc
