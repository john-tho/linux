#!/bin/bash -e

export PARALLEL_PARAMETER="-j $(($(getconf _NPROCESSORS_ONLN) * 2))"
export MAKE="make ${PARALLEL_PARAMETER}"
export PARENT_WORKSPACE=$WORKSPACE
echo PARENT_WORKSPACE=$PARENT_WORKSPACE

function print_header()
{
	echo --------------------------------------------------------------------------------------------
	echo -------------------   $@
	echo --------------------------------------------------------------------------------------------
}

# Prints a small title
#
# @param title
function print_small_title
{
	local title=$1

	echo --------------------   $title
}

function export_common_env_vars()
{
	if [[ -z $PARENT_WORKSPACE ]]; then
		echo "export_common_env_vars: Error! PARENT_WORKSPACE is not set!"
		exit 1
	fi

	export HAL_TOP=${HAL_TOP:-$PARENT_WORKSPACE}

	echo "export_common_env_vars: Config set by this function is as follows"
	echo "export_common_env_vars: HAL_TOP = ${HAL_TOP}"

	export USE_CCACHE=${USE_CCACHE:-1}

	if [ "$USE_CCACHE" == "1" ]; then
		export CCACHE_DIR=${CCACHE_DIR:-/local/jenkins/ccache}
		export CCACHE_COMPILERCHECK="%compiler% -v"
	else
		export CCACHE_DISABLE=1
	fi
}

