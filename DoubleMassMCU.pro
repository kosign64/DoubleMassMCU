TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.c

INCLUDEPATH += "/usr/lib/avr/include"

DEFINES += __AVR_ATmega8__
