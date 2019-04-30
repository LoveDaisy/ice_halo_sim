QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = FastHalo
TEMPLATE = app

DEFINES += QT_DEPRECATED_WARNINGS

CONFIG += c++11

CPP_PROJ_ROOT = ../

FORMS += \
    mainwindow.ui \
    previewwindow.ui

HEADERS += \
    mainwindow.h \
    $$CPP_PROJ_ROOT/src/context.h \
    $$CPP_PROJ_ROOT/src/crystal.h \
    $$CPP_PROJ_ROOT/src/files.h \
    $$CPP_PROJ_ROOT/src/mymath.h \
    $$CPP_PROJ_ROOT/src/optics.h \
    $$CPP_PROJ_ROOT/src/render.h \
    $$CPP_PROJ_ROOT/src/simulation.h \
    $$CPP_PROJ_ROOT/src/threadingpool.h \
    icons.h \
    iconbutton.h \
    previewwindow.h \
    spinboxdelegate.h

SOURCES += \
    main.cpp \
    $$CPP_PROJ_ROOT/src/context.cpp \
    $$CPP_PROJ_ROOT/src/crystal.cpp \
    $$CPP_PROJ_ROOT/src/files.cpp \
    $$CPP_PROJ_ROOT/src/mymath.cpp \
    $$CPP_PROJ_ROOT/src/optics.cpp \
    $$CPP_PROJ_ROOT/src/render.cpp \
    $$CPP_PROJ_ROOT/src/simulation.cpp \
    $$CPP_PROJ_ROOT/src/threadingpool.cpp \
    mainwindow.cpp \
    icons.cpp \
    iconbutton.cpp \
    previewwindow.cpp \
    spinboxdelegate.cpp

INCLUDEPATH += $$CPP_PROJ_ROOT/src

RESOURCES += \
    ui.qrc

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

# Boost
macx: LIBS += -L/usr/local/lib/ -lboost_filesystem

INCLUDEPATH += /usr/local/include
DEPENDPATH += /usr/local/include

macx: PRE_TARGETDEPS += /usr/local/lib/libboost_filesystem.a

# Rapidjson
INCLUDEPATH += $$CPP_PROJ_ROOT/thirdparty/rapidjson/include
DEPENDPATH += $$CPP_PROJ_ROOT/thirdparty/rapidjson/include

