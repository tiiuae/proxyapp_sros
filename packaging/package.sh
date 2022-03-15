#!/bin/bash

set -euo pipefail

usage() {
	echo "
Usage: $(basename "$0") [-h] [-b nbr] [-d dist]
 -- Generate debian package from fog_sw module.
Params:
    -h  Show help text.
    -b  Build number. This will be tha last digit of version string (x.x.N).
    -d  Distribution string in debian changelog.
    -g  Git commit hash.
    -v  Git version string
"
	exit 0
}

check_arg() {
	if [ "$(echo $1 | cut -c1)" = "-" ]; then
		return 1
	else
		return 0
	fi
}

error_arg() {
	echo "$0: option requires an argument -- $1"
	usage
}

mod_dir="$(realpath $(dirname $0)/..)"
build_nbr=0
distr=""
version=""
git_commit_hash=""
git_version_string=""

while getopts "hb:d:g:v:" opt
do
	case $opt in
		h)
			usage
			;;
		b)
			check_arg $OPTARG && build_nbr=$OPTARG || error_arg $opt
			;;
		d)
			check_arg $OPTARG && distr=$OPTARG || error_arg $opt
			;;
		g)
			check_arg $OPTARG && git_commit_hash=$OPTARG || error_arg $opt
			;;
		v)
			check_arg $OPTARG && git_version_string=$OPTARG || error_arg $opt
			;;
		\?)
			usage
			;;
	esac
done

if [[ "$git_commit_hash" == "0" || -z "$git_commit_hash" ]]; then
	git_commit_hash="$(git rev-parse HEAD)"
fi
if [[ "$git_version_string" == "0" || -z "$git_version_string" ]]; then
	git_version_string="$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)"
fi

## Remove trailing '/' mark in module dir, if exists
mod_dir=$(echo $mod_dir | sed 's/\/$//')

## Debug prints
echo 
echo "[INFO] mod_dir: ${mod_dir}."
echo "[INFO] build_nbr: ${build_nbr}."
echo "[INFO] distr: ${distr}."
echo "[INFO] git_commit_hash: ${git_commit_hash}."
echo "[INFO] git_version_string: ${git_version_string}."

cd $mod_dir

## Generate package
echo "[INFO] Creating deb package..."
set +u
[ "$arch" = "" ] && arch="amd64"
[ "$package_version" = "" ] && package_version=2.0.0
set -u

## Generate package
echo "Creating deb package..."
build_dir=$(mktemp -d)
mkdir ${build_dir}/DEBIAN

## Build the module
##   module build.sh contains actions:
##    - building binaries from sources
##    - copy artifacts to the build_dir
echo "Build the module..."
./packaging/build.sh $PWD ${build_dir} || exit 1

echo "INFO: Use default packaging."
### Create version string
version="$package_version-${build_nbr}${git_version_string}"
sed -i "s/VERSION/${version}/" ${build_dir}/DEBIAN/control
cat ${build_dir}/DEBIAN/control
echo "version: ${version}"
### create changelog
pkg_name=$(grep -oP '(?<=Package: ).*' ${build_dir}/DEBIAN/control)
mkdir -p ${build_dir}/usr/share/doc/${pkg_name}
cat << EOF > ${build_dir}/usr/share/doc/${pkg_name}/changelog.Debian
${pkg_name} (${version}) ${distr}; urgency=high

  * commit: ${git_commit_hash}

 -- $(grep -oP '(?<=Maintainer: ).*' ${build_dir}/DEBIAN/control)  $(date +'%a, %d %b %Y %H:%M:%S %z')

EOF
gzip ${build_dir}/usr/share/doc/${pkg_name}/changelog.Debian
### create debian package
debfilename=${pkg_name}_${version}_${arch}.deb
echo "${debfilename}"
fakeroot dpkg-deb --build ${build_dir} $mod_dir/${debfilename}

rm -rf ${build_dir}
echo "Done"
