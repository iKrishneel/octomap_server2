FROM ghcr.io/tiiuae/fog-ros-baseimage:builder-3dcb78d AS builder

# TODO: use the same libvtk7-qt-dev-hack_1.0_all.deb hack here to make build faster
#       (currently fails build, don't know the reason)

# Workaround for rosdep issue with libpcl-dev install
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    libpcl-dev

COPY . /main_ws/src/
# RUN ls /etc/ros/rosdep/sources.list.d
# RUN ls /packaging/
# RUN cat /etc/ros/rosdep/sources.list.d/51-fogsw-module.list
# RUN cat /etc/ros/rosdep/sources.list.d/20-default.list
ARG rosdepYamlPath=/packaging/rosdep.yaml
RUN  if [ -e ${rosdepYamlPath} ]; then \ 
    echo "fog_lib:" >> $rosdepYamlPath && \
    echo "    ubuntu: ros-galactic-fog-lib" >> $rosdepYamlPath && \
    echo "yaml file://${rosdepYamlPath}" > /etc/ros/rosdep/sources.list.d/51-fogsw-module.list; \
    fi;
RUN cat $rosdepYamlPath
# this:
# 1) builds the application
# 2) packages the application as .deb & writes it to /main_ws/
#
# SKIP_BUILD_UNDERLAY_STEPS because otherwise build fails for some reason
RUN SKIP_BUILD_UNDERLAY_STEPS=true /packaging/build.sh

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:sha-3dcb78d

ENTRYPOINT /entrypoint.sh

COPY entrypoint.sh /entrypoint.sh

# prevent libpcl-dev from pulling in a full graphical environment.
# produced with these instructions: https://askubuntu.com/a/656153
RUN dpkg -i /tmp/libvtk7-qt-dev-hack_1.0_all.deb
RUN dpkg -i /tmp/ros-galactic-fog-lib_0.1.0-0focal_amd64.deb

COPY --from=builder /main_ws/ros-*-octomap-server2_*_amd64.deb /octomap-server.deb

RUN apt update && apt install -y --no-install-recommends ./octomap-server.deb \
	&& rm /octomap-server.deb

