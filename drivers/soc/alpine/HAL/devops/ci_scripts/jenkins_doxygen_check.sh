#!/bin/bash -e

source ${BASH_SOURCE%/*}/HAL_jenkins_common.sh

export PATH="/usr/bin/:${PATH}"

cd $PARENT_WORKSPACE
export_common_env_vars

print_header "checking doxygen"

rm -rf hal_doc
rsync -Rr . hal_doc
pushd hal_doc
if ! make doc; then
	echo BUILD_DOC_ERROR
	exit 1
fi

popd

