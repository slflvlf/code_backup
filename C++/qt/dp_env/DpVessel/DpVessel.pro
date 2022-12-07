#-------------------------------------------------
#
# Project created by QtCreator 2018-04-30T23:05:15
#
#-------------------------------------------------

QT       -= gui

TARGET = DpVessel
TEMPLATE = lib

DEFINES += DPVESSEL_LIBRARY

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    current_load.cpp \
    marine_structure.cpp \
    qtf_database.cpp \
    radiation_force.cpp \
    rao_database.cpp \
    semi_submersible.cpp \
    wave_excitation.cpp \
    wind_load.cpp

HEADERS += \
    dpvessel_global.h \
    current_load.h \
    marine_structure.h \
    qtf_database.h \
    radiation_force.h \
    rao_database.h \
    semi_submersible.h \
    wave_excitation.h \
    wind_load.h

INCLUDEPATH+= C:\C++library\boost_1_65_1\boost_1_65_1
INCLUDEPATH+= C:\C++library\eigen3\eigen3\Eigen
INCLUDEPATH += C:\C++library\gsl\include

#LIBS += -L"C:/Program Files/GSL/lib" -lgsl -lgslcblas -lm
LIBS += -L C:\C++library\gsl\lib -llibgsl -llibgslcblas -lm

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../DpEnvironment/release/ -lDpEnvironment0
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../DpEnvironment/debug/ -lDpEnvironment0

INCLUDEPATH += $$PWD/../DpEnvironment
DEPENDPATH += $$PWD/../DpEnvironment

win32 {
    VERSION = 0.1.0.180904
    QMAKE_TARGET_COPYRIGHT = "Copyright (c) 2018, Bo Li"
}
