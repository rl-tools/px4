FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y git lsb-release sudo wget gcc-arm-none-eabi python3 python3-pip
RUN git clone https://github.com/rl-tools/px4-autopilot.git
RUN cd px4-autopilot && ./Tools/setup/ubuntu.sh || true
RUN pip3 install --upgrade pip && pip3 install empy==3.3.4
RUN pip3 install --upgrade pip && pip3 install kconfiglib && echo test
RUN git config --global safe.directory '*'
