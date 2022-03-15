#!/bin/bash

set -euxo pipefail

output_dir=$1

git_commit_hash=${2:-$(git rev-parse HEAD)}

git_version_string=${3:-$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)}

build_number=${GITHUB_RUN_NUMBER:=0}

iname=${PACKAGE_NAME:=proxyapp_py_ros2}

iversion=${PACKAGE_VERSION:=latest}

docker build \
  --build-arg BUILD_NUMBER="${build_number}" \
  --pull \
  -t "${iname}_build:${iversion}" .

container_id=$(docker create "${iname}_build" "")
docker cp "${container_id}":/packages .
docker rm "${container_id}"
mkdir -p "$output_dir"
cp packages/*.deb "$output_dir"
rm -Rf packages
