#!/bin/bash

sudo ib_console make -j24 ARCH=arm CROSS_COMPILE=/opt/toolchains/gcc-linaro-arm-linux-gnueabihf-4.7-2013.04-20130415_linux/bin/arm-linux-gnueabihf- zImage
if [ $? -eq 0 ]; then
	echo "Kernel compilado"
else
	echo "Kernel a la mierda"
	exit 1
fi
sudo ib_console make -j24 ARCH=arm CROSS_COMPILE=/opt/toolchains/gcc-linaro-arm-linux-gnueabihf-4.7-2013.04-20130415_linux/bin/arm-linux-gnueabihf- exynos4412-odroidx2.dtb
if [ $? -eq 0 ]; then
        echo "DT compilado"
else
        echo "DT a la mierda"
        exit 1
fi


sudo ib_console make -j24 ARCH=arm CROSS_COMPILE=/opt/toolchains/gcc-linaro-arm-linux-gnueabihf-4.7-2013.04-20130415_linux/bin/arm-linux-gnueabihf- modules
if [ $? -eq 0 ]; then
        echo "modulos compilados"
else
	echo "modulos a la mierda"
        exit 1
fi
sudo ib_console make -j24 ARCH=arm CROSS_COMPILE=/opt/toolchains/gcc-linaro-arm-linux-gnueabihf-4.7-2013.04-20130415_linux/bin/arm-linux-gnueabihf- modules_install INSTALL_MOD_PATH=./modules
if [ $? -eq 0 ]; then
        echo "modulos instalados"
else
	echo "modulos no instalados"
        exit 1
fi
exit 0
