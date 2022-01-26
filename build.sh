#!/bin/bash

set -euxo pipefail

output_dir=$1

git_commit_hash=${2:-$(git rev-parse HEAD)}

git_version_string=${3:-$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)}

build_number=${GITHUB_RUN_NUMBER:=0}

ros_distro=${ROS_DISTRO:=galactic}

iname=${PACKAGE_NAME:=octomap_server2}

iversion=${PACKAGE_VERSION:=latest}

docker build \
  --build-arg UID=$(id -u) \
  --build-arg GID=$(id -g) \
  --build-arg ROS_DISTRO=${ros_distro} \
  --build-arg PACKAGE_NAME=${iname} \
  --pull \
  -f Dockerfile.build_env -t "${iname}_build:${iversion}" .

docker run \
  --rm \
  -v $(pwd):/${iname}/sources \
  ${iname}_build:${iversion} \
  ./packaging/package.sh \
  -b ${build_number} \
  -g ${git_commit_hash} \
  -v ${git_version_string}

mkdir -p ${output_dir}
cp *.deb *.ddeb ${output_dir}
rm -Rf *.deb *.ddeb

exit 0
