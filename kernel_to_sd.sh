#/bin/bash

cat arch/arm/boot/zImage arch/arm/boot/dts/exynos4412-odroidx2.dtb > ./zImage
if [ $? -eq 0 ]; then
        echo "zImage+dtb ok"
else
        echo "zImage+dtb no se ha copiado bien"
        exit 1
fi

sudo cp ./zImage /media/$USER/BOOT/zImage_dtb
if [ $? -eq 0 ]; then
	echo "zImage en memoria, ok"
else
        echo "zImage no se ha copiado bien"
        exit 1
fi

#sudo cp arch/arm/boot/dts/exynos4412-odroidx2.dtb /media/$USER/ROOT/odroid-x2-4.8.dtb
#if [ $? -eq 0 ]; then
#        echo "DTB ok"
#else
#        echo "DTB no se ha copiado bien"
#        exit 1
#fi

sudo rsync -avc ./modules/lib/. /media/$USER/rootfs/lib/.
if [ $? -eq 0 ]; then
	echo "modulos listos"
else
	echo "los modulos no estan bien copiados"
	exit 1
fi
exit 0
