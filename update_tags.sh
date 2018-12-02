#!/bin/bash

CTAGS_FLAGS="--excmd=number --c-kinds=+p --c++-kinds=+p --tag-relative=no --fields=+a+m+n+S -R"

DIRS="src/app \
	src/argon/include \
	src/argon/src \
	src/bootloader \
	src/CMSIS/Include \
	src/fatfs \
	src/sdmmc \
	src/ksdk"

ctags $CTAGS_FLAGS $DIRS
