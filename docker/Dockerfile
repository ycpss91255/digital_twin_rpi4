# syntax = docker/dockerfile:experimental
FROM ros:noetic-ros-base

# Arguments
ARG USER=initial
ARG GROUP=initial
ARG UID=1000
ARG GID=${UID}
ARG SHELL=/bin/bash
# Setup users and groups
RUN groupadd --gid ${GID} ${GROUP} \
  && useradd --gid ${GID} --uid ${UID} -ms ${SHELL} ${USER} \
  && mkdir -p /etc/sudoers.d \
  && echo "${USER}:x:${UID}:${UID}:${USER},,,:$HOME:${shell}" >> /etc/passwd \
  && echo "${USER}:x:${UID}:" >> /etc/group \
  && echo "${USER} ALL=(ALL) NOPASSWD: ALL" > "/etc/sudoers.d/${USER}" \
  && chmod 0440 "/etc/sudoers.d/${USER}"

# Replace apt urls
RUN sed -i 's@archive.ubuntu.com@ftp.tku.edu.tw/@g' /etc/apt/sources.list
# Install packages
RUN apt-get update && apt-get install -y --no-install-recommends \
      sudo net-tools htop git rsync wget zip unzip \
      byobu zsh terminator \
      nano vim \
      python3-pip python3-catkin-tools python3-setuptools \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /home/${USER}/.tmp

# setup custom configuration
COPY config .
# shell
RUN bash ./script/bash_setup.bash \
  && bash ./script/terminator_setup.bash \
  && bash ./script/zsh_setup.bash \
  && bash ./script/byobu_setup.bash \
  # Dependent
  && bash ./script/rule_setup.bash \
  && bash ./script/pip_setup.bash \
  && bash ./script/pigpio_setup.bash \
  && rm -rf /home/${USER}/.tmp

# copy entrypoint
COPY entrypoint.bash /entrypoint.bash
RUN chmod 777 /entrypoint.bash

# RUN pigpiod
WORKDIR /home/${USER}

# Switch user to ${USER}
USER ${USER}

RUN sudo mkdir work

# Make SSH available
EXPOSE 22

# Switch to user's HOME folder
WORKDIR /home/${USER}/work

ENTRYPOINT ["/entrypoint.bash","byobu"]
# ENTRYPOINT ["byobu"]