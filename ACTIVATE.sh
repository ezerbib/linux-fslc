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

my_go_install_version()
{
dst=$1
dir=/home/ezerbib/workspace/KDS/Codebase/products/KDS-4/debian/kds-$dst-4/
rsync -avvr  -r release/lib/modules ${dir}/lib
cp -v arch/arm/boot/zImage ${dir}/boot/zImage-3.14.59-rt59+.new
#rename dst for dtb filename matching
[ "$dst" = "ken" ] && dst=kenc
#even in encoder the file load is kdec4
cp -v ./arch/arm/boot/dts/imx6q-hummingboard-${dst}4.dtb ${dir}/boot/imx6q-hummingboard-kdec4.dtb
}

my_go_install_version_dec()
{
my_go_install_version kdec
}

my_go_install_version_enc()
{
my_go_install_version ken
}

my_go_install_dec()
{
my_go_install 68.151
}

my_go_install_enc()
{
my_go_install 68.150
}

my_go_install_mnt () 
{ 
    ip=$1;
    rsync -avvr -r release/lib/modules root@192.168.$ip:/mnt/lib;
    scp arch/arm/boot/zImage root@192.168.$ip:/mnt//boot/zImage-3.14.59-rt59+.new
}
