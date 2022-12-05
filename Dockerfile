FROM ghcr.io/tiiuae/fog-ros-baseimage:builder-7072ebc AS builder

# TODO: use the same libvtk7-qt-dev-hack_1.0_all.deb hack here to make build faster
#       (currently fails build, don't know the reason)

# Workaround for rosdep issue with libpcl-dev install
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    libpcl-dev

COPY . /main_ws/src/

# this:
# 1) builds the application
# 2) packages the application as .deb & writes it to /main_ws/
#
# SKIP_BUILD_UNDERLAY_STEPS because otherwise build fails for some reason
RUN SKIP_BUILD_UNDERLAY_STEPS=true /packaging/build.sh

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:sha-7072ebc

ENTRYPOINT /entrypoint.sh

COPY entrypoint.sh /entrypoint.sh

COPY misc/libvtk7-qt-dev-hack_1.0_all.deb /tmp/libvtk7-qt-dev-hack_1.0_all.deb

# prevent libpcl-dev from pulling in a full graphical environment.
# produced with these instructions: https://askubuntu.com/a/656153
RUN dpkg -i /tmp/libvtk7-qt-dev-hack_1.0_all.deb

COPY --from=builder /main_ws/ros-*-octomap-server2_*_amd64.deb /octomap-server.deb

RUN apt update && apt install -y --no-install-recommends ./octomap-server.deb \
	&& rm /octomap-server.deb

