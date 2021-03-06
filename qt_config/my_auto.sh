#!/bin/sh
./configure \
	-prefix /home/sbc_7109_455/aplex/qt_arm/qt_arm_insatll  \
	-release \
	-opensource \
	-confirm-license \
	-make libs \
	-xplatform linux-arm-gnueabi-g++ \
	-optimized-qmake \
	-linuxfb    \
	-pch \
	-qt-sql-sqlite \
	-qt-zlib \
	-qt-libpng \
	-tslib \
	-no-opengl \
	-no-sse2 \
	-no-openssl \
	-no-nis \
	-no-cups \
	-no-glib \
	-no-dbus \
	-no-xcb \
	-no-xcursor -no-xfixes -no-xrandr -no-xrender \
	-no-separate-debug-info \
	-make examples -nomake tools -nomake tests -no-iconv
