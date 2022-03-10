# fog-sw BUILDER
FROM ros:galactic-ros-base as fog-sw-builder

ARG BUILD_NUMBER
ARG GIT_VERSION_STRING
ARG GIT_COMMIT_HASH

# workaround for ROS GPG Key Expiration Incident
RUN rm /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get update && \
    apt-get install -y curl && \
    curl http://repo.ros2.org/repos.key | sudo apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get update

RUN echo "deb [trusted=yes] https://ssrc.jfrog.io/artifactory/ssrc-debian-public-remote focal fog-sw" >> /etc/apt/sources.list

# Install build dependencies
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    debhelper \
    dh-make \
    fakeroot \
    ros-galactic-fastcdr=1.0.20-4~git20220225.c0e0fa2 \
    ros-galactic-fastrtps=2.5.0-6~git20220308.617c8fe \
    ros-galactic-fastrtps-cmake-module=1.2.1-5~git20220308.f68fbd2 \
    ros-galactic-fog-msgs=0.0.8-42~git20220104.1d2cf3f \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build

COPY . .

RUN params= \
    && [ ! "${BUILD_NUMBER}" = "" ] && params="$params -b ${BUILD_NUMBER}" || : \
    && [ ! "${GIT_VERSION_STRING}" = "" ] && params="$params -v ${GIT_VERSION_STRING}" || : \
    && [ ! "${GIT_COMMIT_HASH}" = "" ] && params="$params -g ${GIT_COMMIT_HASH}" || : \
    && ./packaging/package.sh $params

FROM scratch
COPY --from=fog-sw-builder /build/*.deb /packages/
