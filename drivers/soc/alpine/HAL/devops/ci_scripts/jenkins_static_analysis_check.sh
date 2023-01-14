#!/bin/bash -e

source ${BASH_SOURCE%/*}/HAL_jenkins_common.sh

gcc_base="-gcc-base-dir `gcc --print-file-name=`"
export PATH=/project/software/tools/sparse/:$PATH

cd $PARENT_WORKSPACE
export_common_env_vars

if [ "$CHECK_SPARSE_V1" == "1" ]; then
	print_header "checking sparse (Alpine V1)"

	CHECK_SPARSE_DEFS="AL_DEV_ID=AL_DEV_ID_ALPINE_V1 AL_DEV_REV_ID=1 AL_HAL_FP_EX=1 AL_HAL_EX=1"
	make $CHECK_SPARSE_DEFS CFLAGS="${gcc_base}" clean
	sparse_errors=$(make CC="sparse" $CHECK_SPARSE_DEFS CFLAGS="${gcc_base}" \
			$PARALLEL_PARAMETER 2>&1 >/dev/null)
	if [ -n "$sparse_errors" ]; then
		echo "$sparse_errors"
		echo SPARSE_ERRORS
		exit 1
	else
		echo SPARSE_PASS
	fi
else
	print_header "skipping sparse (Alpine V1)"
fi
if [ "$CHECK_SPARSE_V2" == "1" ]; then
	print_header "checking sparse (Alpine V2)"

	CHECK_SPARSE_DEFS="AL_DEV_ID=AL_DEV_ID_ALPINE_V2 AL_DEV_REV_ID=0 AL_HAL_FP_EX=1 AL_HAL_EX=1"
	make $CHECK_SPARSE_DEFS CFLAGS="${gcc_base}" clean
	sparse_errors=$(make CC="sparse" $CHECK_SPARSE_DEFS CFLAGS="${gcc_base}" \
			$PARALLEL_PARAMETER 2>&1 >/dev/null)
	if [ -n "$sparse_errors" ]; then
		echo "$sparse_errors"
		echo SPARSE_ERRORS
		exit 1
	else
		echo SPARSE_PASS
	fi
else
	print_header "skipping sparse (Alpine V2)"
fi

if [ "$CHECK_SPARSE_V3" == "1" ]; then
	print_header "checking sparse (Alpine V3-TC)"

	CHECK_SPARSE_DEFS="AL_DEV_ID=AL_DEV_ID_ALPINE_V3 AL_DEV_REV_ID=0 AL_HAL_FP_EX=1 AL_HAL_EX=1"
	make $CHECK_SPARSE_DEFS CFLAGS="${gcc_base}" clean
	sparse_errors=$(make CC="sparse" $CHECK_SPARSE_DEFS CFLAGS="${gcc_base}" \
			$PARALLEL_PARAMETER 2>&1 >/dev/null)
	if [ -n "$sparse_errors" ]; then
		echo "$sparse_errors"
		echo SPARSE_ERRORS
		exit 1
	else
		echo SPARSE_PASS
	fi

	print_header "checking sparse (Alpine V3)"

	CHECK_SPARSE_DEFS="AL_DEV_ID=AL_DEV_ID_ALPINE_V3 AL_DEV_REV_ID=1 AL_HAL_FP_EX=1 AL_HAL_EX=1"
	make $CHECK_SPARSE_DEFS CFLAGS="${gcc_base}" clean
	sparse_errors=$(make CC="sparse" $CHECK_SPARSE_DEFS CFLAGS="${gcc_base}" \
			$PARALLEL_PARAMETER 2>&1 >/dev/null)
	if [ -n "$sparse_errors" ]; then
		echo "$sparse_errors"
		echo SPARSE_ERRORS
		exit 1
	else
		echo SPARSE_PASS
	fi
else
	print_header "skipping sparse (Alpine V3)"
fi

if [ "$CHECK_SPARSE_AL8" == "1" ]; then
	print_header "checking sparse (AL8)"

	CHECK_SPARSE_DEFS="AL_DEV_ID=AL_DEV_ID_AL8 AL_DEV_REV_ID=0 AL_HAL_FP_EX=1 AL_HAL_EX=1"
	make $CHECK_SPARSE_DEFS CFLAGS="${gcc_base}" clean
	sparse_errors=$(make CC="sparse" $CHECK_SPARSE_DEFS CFLAGS="${gcc_base}" \
			$PARALLEL_PARAMETER 2>&1 >/dev/null)
	if [ -n "$sparse_errors" ]; then
		echo "$sparse_errors"
		echo SPARSE_ERRORS
		exit 1
	else
		echo SPARSE_PASS
	fi
else
	print_header "skipping sparse (AL8)"
fi

