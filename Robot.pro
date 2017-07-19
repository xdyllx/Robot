TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    tcpserver.cpp \
    robot.cpp \
    knect.cpp

HEADERS += \
    tcpserver.h \
    robot.h \
    knect.h


LIBS += -ldl
LIBS += -lpthread
unix:!macx: LIBS += -L /usr/local/Aria/lib/ -lAria
INCLUDEPATH += /usr/local/Aria/include
DEPENDPATH += /usr/local/Aria/include

unix:!macx: LIBS += -L$$PWD/../freenect2/lib/ -lfreenect2

INCLUDEPATH += $$PWD/../../freenect2/lib
DEPENDPATH += $$PWD/../../freenect2/lib

INCLUDEPATH += /home/xdy/freenect2/include/libfreenect2
INCLUDEPATH += /home/xdy/freenect2/include

LIBS += -L/usr/lib/x86_64-linux-gnu/ -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_gpu

unix:!macx: LIBS += -L$$PWD/usr/lib/x86_64-linux-gnu/ -lboost_system

unix:!macx: LIBS += -L$$PWD/usr/lib/x86_64-linux-gnu/ -lboost_filesystem

INCLUDEPATH += $$PWD/usr/lib/x86_64-linux-gnu
DEPENDPATH += $$PWD/usr/lib/x86_64-linux-gnu
