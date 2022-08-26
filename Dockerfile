FROM arm64v8/ubuntu

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update
RUN apt-get install -y build-essential
RUN apt-get install -y python3
RUN apt-get install -y cmake
RUN apt-get install -y gcc-arm-none-eabi libnewlib-arm-none-eabi

VOLUME [ "/project" ]

WORKDIR /project
