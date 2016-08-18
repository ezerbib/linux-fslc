go()
{
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -j8 zImage
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -j20 modules
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-  headers_install  INSTALL_HDR_PATH=`pwd`/release/usr 
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -j8 modules_install INSTALL_MOD_PATH=`pwd`/release/
}

my_go_install()
{
ip=$1
rsync -avvr  -r release/lib/modules root@192.168.$ip:/lib/
scp arch/arm/boot/zImage root@192.168.$ip:/boot/zImage-3.14.59-rt59+.new
}

my_go_install_dec()
{
my_go_install 68.151
}

my_go_install_enc()
{
my_go_install 68.150
}
