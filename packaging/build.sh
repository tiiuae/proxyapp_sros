#!/bin/bash

set -euo pipefail

build_dir=$1
dest_dir=$2

cp ${build_dir}/packaging/debian/* ${dest_dir}/DEBIAN/

cd ${build_dir}

set +u
source /opt/ros/galactic/setup.bash
set -u

if [ ! -e ${build_dir}/proxyapp_build ]; then
    mkdir ${build_dir}/proxyapp_build
fi
pushd ${build_dir}/proxyapp_build
cmake ..
make
popd

if [ ! -e ${build_dir}/proxyapp_server_client_build ]; then
    mkdir ${build_dir}/proxyapp_server_client_build
fi
pushd ${build_dir}/proxyapp_server_client_build
cmake ../tools
make
popd

mkdir -p ${dest_dir}/usr/bin
cp -f ${build_dir}/proxyapp_build/proxyapp_ros ${dest_dir}/usr/bin/ || exit
chmod +x ${dest_dir}/usr/bin/proxyapp_ros || exit

cp -f ${build_dir}/proxyapp_server_client_build/proxyapp_server_client ${dest_dir}/usr/bin/ || exit
chmod +x ${dest_dir}/usr/bin/proxyapp_server_client || exit
