#!/bin/bash

set -euxo pipefail

output_dir=$1

build_number=${GITHUB_RUN_NUMBER:=0}

ros_distro=${ROS_DISTRO:=foxy}

iname=${PACKAGE_NAME:=octomap_server2}

docker build \
  --build-arg UID=$(id -u) \
  --build-arg GID=$(id -g) \
  --build-arg ROS_DISTRO=${ros_distro} \
  --build-arg PACKAGE_NAME=${iname} \
  --pull \
  -f Dockerfile.build_env -t "${iname}:latest" .

docker run \
  --rm \
  -v $(pwd):/${iname}/sources \
  ${iname}:latest \
  ./packaging/package.sh \
  -b ${build_number}

mkdir -p ${output_dir}
cp *.deb *.ddeb ${output_dir}
rm -Rf *.deb *.ddeb

exit 0
