#!/bin/bash

# ---------------------------------------------------------
# Simple kernel build script (c) by EoF Software Labs  
# ---------------------------------------------------------

# if not in kernel directory
if [ -d kernel -a ! -f Kbuild ] ; then
    cd kernel
fi

SRC_DIR=`pwd`
OS_HOST=`uname -m`
MP="-j3"

# Output directory
UPDATE_DIR="/mnt/ssd/boot-update"

if [ ${OS_HOST} == "x86_64" ] ; then
    #using Ubuntu gcc-10-aarch64
    #TOOL_CHAIN="/opt/FriendlyARM/toolchain/6.4-aarch64"
    UPDATE_DIR="/workspace/projects/kernels/output/aio-rk3568j"
    CROSS_COMPILE="aarch64-linux-gnu-"
	CC="${CROSS_COMPILE}gcc-10"
    LD="${CROSS_COMPILE}ld"
    MP="-j8"
	KRNL_ARCH="arm64"
	REMOTE_SSHKEY="${HOME}/.ssh/id_rsa_tux_deploy" 
	REMOTE_HOST="192.168.181.112"
    #dev-sd	
	#REMOTE_HOST="192.168.181.12"
	#prod
	if [ "x$1" == "xprod" ] ; then
	REMOTE_HOST="192.168.181.113"
	fi 
    REMOTE_TARGET="root@${REMOTE_HOST}:/mnt/ssd/boot-update/"
    if [ -e ${TOOL_CHAIN} ] ; then 
        if [[ ! ${PATH} =~ "${TOOL_CHAIN}" ]]; then
            export PATH=${TOOL_CHAIN}/bin/:$PATH
        fi
    fi
else
    KRNL_ARCH="arm64"
    CC="gcc"
fi

# Used kernel branch
#KRNL_BRANCH="nanopi4-linux-v4.4.y"
KRNL_BRANCH=`git branch`

# Get kernel version
KRNL_VER="`make ${MP} CC=${CC} LD=${LD} CROSS_COMPILE=${CROSS_COMPILE} ARCH=${KRNL_ARCH} kernelrelease`"

echo "----------------------------------------------------"
echo "SRC_DIR......: ${SRC_DIR}"
echo "OS_HOST......: ${OS_HOST}"
echo "UPDATE_DIR...: ${UPDATE_DIR}"
echo "KERNEL_VER...: ${KRNL_VER}"
echo "KERNEL_ARCH..: ${KRNL_ARCH}"
echo "KERNEL_BRANCH: ${KRNL_BRANCH}"
echo "TOOLCHAIN....: ${TOOL_CHAIN}"
echo "CROSS_COMPILE: ${CROSS_COMPILE}"
echo "REMOTE_TARGET: ${REMOTE_TARGET}"
echo "----------------------------------------------------"
#echo | ${CC} -dM -E -
#echo "----------------------------------------------------"

mkdir -p ${UPDATE_DIR}

if [ "x$1" == "xmenuconfig" ] ; then
	make ${MP} CC=${CC} LD=${LD} CROSS_COMPILE=${CROSS_COMPILE} ARCH=${KRNL_ARCH} menuconfig || exit 1
    exit 0
fi

if [ ! -f .config ] ; then
    echo "----------------------------------------------------"
    echo "++ Configure kernel from eof_defconfig"
    make ${MP} CC=${CC} LD=${LD} CROSS_COMPILE=${CROSS_COMPILE} ARCH=${KRNL_ARCH} eof_defconfig
    make ${MP} CC=${CC} LD=${LD} CROSS_COMPILE=${CROSS_COMPILE} ARCH=${KRNL_ARCH} kernelrelease || exit 1
fi

# ===============================================================
# Build and installtion
# ===============================================================

echo "----------------------------------------------------"
echo "++ Build kernel targets: Image modules dtbs"
make ${MP} CC=${CC} LD=${LD} CROSS_COMPILE=${CROSS_COMPILE} ARCH=${KRNL_ARCH} Image modules dtbs || exit 1

if [ "x$1" == "xcompile" ] ; then
	exit 0
fi

echo "----------------------------------------------------"
echo "++ Install kernel image to: ${UPDATE_DIR}"
make ${MP} CC=${CC} LD=${LD} CROSS_COMPILE=${CROSS_COMPILE} ARCH=${KRNL_ARCH} \
		INSTALL_PATH=${UPDATE_DIR} install || exit 1

echo "----------------------------------------------------"
echo "++ Install modules to: ${UPDATE_DIR}"
make ${MP} CC=${CC} LD=${LD} CROSS_COMPILE=${CROSS_COMPILE} ARCH=${KRNL_ARCH} \
		INSTALL_MOD_PATH=${UPDATE_DIR} modules_install || exit 1

echo "----------------------------------------------------"
echo "++ Install device tree DTBs to: ${UPDATE_DIR}"
make ${MP} CC=${CC} LD=${LD} CROSS_COMPILE=${CROSS_COMPILE} ARCH=${KRNL_ARCH} \
		INSTALL_DTBS_PATH=${UPDATE_DIR}/dtbs dtbs_install || exit 1

echo "----------------------------------------------------"
echo "++ Install kernel userspace headers to: ${UPDATE_DIR}"
make ${MP} CC=${CC} LD=${LD} CROSS_COMPILE=${CROSS_COMPILE} ARCH=${KRNL_ARCH}  \
        INSTALL_HDR_PATH=${UPDATE_DIR}/usr/src headers_install || exit 1

# change path name into distribution name
rm -fR ${UPDATE_DIR}/usr/src/linux-headers-$KRNL_VER
mv ${UPDATE_DIR}/usr/src/include ${UPDATE_DIR}/usr/src/linux-headers-$KRNL_VER

# FriendlyArm SOM-RK3399 generate EMMC images files
if [ "x${BOARD_TYPE}" == "xRK3399" ] ; then
echo "----------------------------------------------------"
echo "++ Generate nanopi4 kernel images..."
make ${MP} CC=${CC} LD=${LD} CROSS_COMPILE=${CROSS_COMPILE} ARCH=${KRNL_ARCH} nanopi4-images || exit 1
fi

rm -vf ${UPDATE_DIR}/lib/modules/$KRNL_VER/source
rm -vf ${UPDATE_DIR}/lib/modules/$KRNL_VER/build

cp -Rpvu System.map ${UPDATE_DIR}/System.map-${KRNL_VER}
cp -Rpvu .config ${UPDATE_DIR}/config-${KRNL_VER}

# FriendlyArm SOM-RK3399 EMMC image files
if [ ! -e ${REMOTE_TARGET} ] ; then
	# Remote SBC
    #eval $(ssh-agent) # Create agent and environment variables
    ssh-add ${REMOTE_SSHKEY}
    OPTS="-arluvt"
    SOURCE="${UPDATE_DIR}/"
    echo "Enter manual following commands:"
    # SOK-RK3399
    [ -f kernel.img -a -f resource.img ] && \
        echo "scp -r -i ${REMOTE_SSHKEY} resource.img kernel.img ${REMOTE_TARGET}"
    # @ all
    rsync ${OPTS} ${SOURCE} ${REMOTE_TARGET}
else
    # Local SBC
    [ -f kernel.img -a -f resource.img ] && \
        cp -Rpvu resource.img kernel.img ${UPDATE_DIR}/
fi

exit $?

