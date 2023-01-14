#!/bin/bash -e

source ${BASH_SOURCE%/*}/HAL_jenkins_common.sh

cd $PARENT_WORKSPACE
export_common_env_vars
print_header "building HAL itself using various flavors"

TOOLCHAIN_PATH=/project/software/tools/toolchain
export PATH=$TOOLCHAIN_PATH/gcc-linaro-7.1.1-2017.08-x86_64_aarch64-linux-gnu/bin:$PATH
export PATH=$TOOLCHAIN_PATH/arm-linux-musl/bin:$PATH
export PATH=$TOOLCHAIN_PATH/gcc-linaro-arm-linux-gnueabihf-4.9-2014.07_linux/bin:$PATH
if [ "$USE_CCACHE" == "1" ]; then
	CCACHE_PREFIX="ccache"
fi

if ! CC="${CCACHE_PREFIX} gcc" ./build_all.sh $PARALLEL_PARAMETER; then
	echo BUILD_HAL_ERROR
	echo native_compiler
	exit 1
fi

if ! CC="${CCACHE_PREFIX} arm-linux-gnueabihf-gcc" ./build_all.sh $PARALLEL_PARAMETER; then
	echo BUILD_HAL_ERROR
	echo arm-linux-gnueabihf-gcc_compiler
	exit 1
fi

if ! CC="${CCACHE_PREFIX} aarch64-linux-gnu-gcc" ./build_all.sh $PARALLEL_PARAMETER; then
	echo BUILD_HAL_ERROR
	echo aarch64-linux-gnu-gcc_compiler
	exit 1
fi

if ! CC="aarch64-unknown-linux-musl-gcc" ./build_all.sh $PARALLEL_PARAMETER; then
	echo BUILD_HAL_ERROR
	echo aarch64-unknown-linux-musl-gcc
	exit 1
fi

