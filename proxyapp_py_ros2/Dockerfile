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
    wget \
    libssl-dev \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://ssrc.jfrog.io/artifactory/ssrc-debian-public-remote/ros-galactic-foonathan-memory-vendor_1.1.0-4~git20220310.bbb8a5c_amd64.deb \
    && wget https://ssrc.jfrog.io/artifactory/ssrc-debian-public-remote/ros-galactic-fastcdr_1.0.20-5~git20220310.f65f034_amd64.deb \
    && wget https://ssrc.jfrog.io/artifactory/ssrc-debian-public-remote/ros-galactic-fastrtps_2.5.0-7~git20220310.4ca1f95_amd64.deb \
    && wget https://ssrc.jfrog.io/artifactory/ssrc-debian-public-remote/ros-galactic-fastrtps-cmake-module_1.2.1-6~git20220310.67ed436_amd64.deb \
    && wget https://ssrc.jfrog.io/artifactory/ssrc-debian-public-remote/ros-galactic-fog-msgs_0.0.8-42~git20220104.1d2cf3f_amd64.deb \
    && dpkg -i ros-galactic-foonathan-memory-vendor*.deb ros-galactic-fastcdr*.deb ros-galactic-fastrtps_*.deb ros-galactic-fastrtps-cmake-module*.deb ros-galactic-fog-msgs*.deb

WORKDIR /build

COPY . .

RUN params= \
    && [ ! "${BUILD_NUMBER}" = "" ] && params="$params -b ${BUILD_NUMBER}" || : \
    && [ ! "${GIT_VERSION_STRING}" = "" ] && params="$params -v ${GIT_VERSION_STRING}" || : \
    && [ ! "${GIT_COMMIT_HASH}" = "" ] && params="$params -g ${GIT_COMMIT_HASH}" || : \
    && ./packaging/package.sh $params

FROM scratch
COPY --from=fog-sw-builder /build/*.deb /packages/
