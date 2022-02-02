# fog-sw BUILDER
ARG FROM_IMAGE
FROM $FROM_IMAGE as fog-sw-builder
ARG ROS_DISTRO="galactic"
ARG UID=1000
ARG GID=1000
ARG PACKAGE_NAME

WORKDIR /$PACKAGE_NAME/main_ws
USER root
ADD . /$PACKAGE_NAME/main_ws/src
RUN chown -R builder:builder /$PACKAGE_NAME/main_ws

USER builder

RUN if [ -e /$PACKAGE_NAME/deps_ws ]; then \
        . /$PACKAGE_NAME/deps_ws/install/setup.sh && \
        colcon build; \
    elif [ -e /opt/ros/${ROS_DISTRO}/setup.sh ]; then \
        . /opt/ros/${ROS_DISTRO}/setup.sh && \
        colcon build; \
    fi

RUN sed --in-place \
      's|^source .*|source "/'$PACKAGE_NAME'/main_ws/install/setup.bash"|' \
      /$PACKAGE_NAME/entrypoint.sh && \
        chmod +x /$PACKAGE_NAME/entrypoint.sh

ENV PACKAGE_NAME $PACKAGE_NAME
ENV RMW_IMPLEMENTATION rmw_fastrtps_cpp

WORKDIR /$PACKAGE_NAME
ENTRYPOINT "/"$PACKAGE_NAME"/entrypoint.sh"
