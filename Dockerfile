FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y git lsb-release sudo wget gcc-arm-none-eabi python3 python3-pip
RUN git clone https://github.com/rl-tools/px4-autopilot.git
RUN cd px4-autopilot && git checkout rl-tools/1.14
RUN cd px4-autopilot && ./Tools/setup/ubuntu.sh || true
RUN pip3 install --upgrade pip && pip3 install empy==3.3.4
RUN pip3 install --upgrade pip && pip3 install kconfiglib && echo test
RUN git config --global safe.directory '*'

RUN apt-get update && apt-get install -y curl lsb-release gnupg
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && apt-get -y install gz-garden
RUN apt-get update && apt-get -y install libgz-transport12-dev