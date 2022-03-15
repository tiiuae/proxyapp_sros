#!/bin/bash

set -euo pipefail

build_dir=$1
dest_dir=$2

cp ${build_dir}/packaging/debian/* ${dest_dir}/DEBIAN/

cd ${build_dir}
mkdir -p ${dest_dir}/usr/bin
cp -f proxyapp_py_ros2/secure_server.py ${dest_dir}/usr/bin/ || exit
cp -f proxyapp_py_ros2/unsecure_client.py ${dest_dir}/usr/bin/ || exit

mkdir -p ${dest_dir}/etc/systemd/system
cp -f ${build_dir}/systemd/proxyapp_py_secure_srv.service ${dest_dir}/etc/systemd/system/ || exit
mkdir -p ${dest_dir}/etc/systemd/system
cp -f ${build_dir}/systemd/proxyapp_py_unsecure_client.service ${dest_dir}/etc/systemd/system/ || exit
