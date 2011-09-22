make ARCH=arm htcleo_defconfig
make ARCH=arm CROSS_COMPILE=/root/CodeSourcery/Sourcery_G++_Lite_OLDTOOLCHAIN/bin/arm-none-linux-gnueabi- zImage -j6
make ARCH=arm CROSS_COMPILE=/root/CodeSourcery/Sourcery_G++_Lite_OLDTOOLCHAIN/bin/arm-none-linux-gnueabi- modules -j6
git log > ./currentrelease.txt
./compiledcopy

tar cvzf ../"ACAOmegaHTC35Kernel`date +"%m%d%y%H%M"`".tar.gz ../compiled/
mv ../ACAOmegaSaviorKernel* ../ACAKernels

