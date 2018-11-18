################################################################################
# setup Xilinx Vivado FPGA tools
################################################################################

#. ~/Xilinx/SDK/2015.4/settings64.sh
###############################################################################
# setup Linaro toolchain
################################################################################

#export TOOLCHAIN_PATH=/opt/linaro/gcc-linaro-4.9-2015.02-3-x86_64_arm-linux-gnueabihf
export TOOLCHAIN_PATH=/opt/linaro/gcc-linaro-7.3.1-2018.05-x86_64_arm-linux-gnueabihf
export PATH=$PATH:$TOOLCHAIN_PATH/bin
export PATH=$PATH:$TOOLCHAIN_PATH/libexec/gcc/arm-linux-gnueabihf/7.3.1
export PATH=$PATH:$TOOLCHAIN_PATH/include
export PATH=$TOOLCHAIN_PATH/lib/gcc/arm-linux-gnueabihf/7.3.1/include:$PATH
export CROSS_COMPILE=arm-linux-gnueabihf-

################################################################################
# setup Buildroot download cache directory, to avoid downloads
# this path is also used by some other downloads
################################################################################

#export BR2_DL_DIR=$HOME/Workplace/buildroot/dl
export BR2_DL_DIR=dl

#. ~/Xilinx/SDK/2015.4/settings64.sh

################################################################################
# common make procedure, should not be run by this script
################################################################################

#GIT_COMMIT_SHORT=`git rev-parse --short HEAD`
#make REVISION=$GIT_COMMIT_SHORT
