##############################################################################
##                                Base Image                                ##
##############################################################################
ARG RENDER=nvidia
FROM python:3.12.7 as python
USER root
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

##############################################################################
##                          Rendering Dependencies                          ##
##############################################################################
FROM python as render-base
USER root
RUN apt update \
  && apt install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
  && rm -rf /var/lib/apt/lists/*# Env vars for the nvidia-container-runtime.

RUN DEBIAN_FRONTEND=noninteractive \
	apt update && \
	apt install -y mesa-utils libgl1-mesa-glx libglu1-mesa-dev freeglut3-dev mesa-common-dev libopencv-dev python3-opencv python3-tk
RUN DEBIAN_FRONTEND=noninteractive apt update && apt install -y python3-pip

##############################################################################
##                                  Nvidia                                  ##
##############################################################################
FROM render-base as render-nvidia

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

##############################################################################
##                                   User                                   ##
##############################################################################
FROM render-${RENDER} as user

# install sudo
RUN apt-get update && apt-get install -y sudo

# Create user
ARG USER=jovyan
ARG PASSWORD=automaton
ARG UID=1000
ARG GID=1000
ENV USER=$USER
RUN groupadd -g $GID $USER \
    && useradd -m -u $UID -g $GID -p "$(openssl passwd -1 $PASSWORD)" \
    --shell $(which bash) $USER -G sudo
RUN echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudogrp
RUN usermod -a -G video $USER
USER $USER
RUN mkdir -p /home/$USER/workspace/src
RUN mkdir -p /home/$USER/data

##############################################################################
##                               Dependencies                               ##
##############################################################################
FROM user as dependencies
RUN pip install --no-cache-dir numpy opencv-python opencv-contrib-python
RUN pip install --no-cache-dir pybullet
RUN pip install --no-cache-dir loguru
RUN pip install --no-cache-dir scipy
RUN pip install --no-cache-dir pandas
RUN pip install --no-cache-dir openpyxl

ENV PYTHONPATH=/home/$USER/workspace/src:$PYTHONPATH

WORKDIR /home/$USER/workspace/src
CMD ["bash"]