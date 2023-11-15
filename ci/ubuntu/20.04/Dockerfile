FROM ubuntu:20.04

RUN mkdir -p /opt/built

# prevent failures due to interactive apt transactions
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get -y update
RUN apt-get -y upgrade --no-install-recommends

# Copy and install packages from the latest opentestpoint build
WORKDIR /opt

# Build tools
RUN apt-get -y install git autoconf automake libtool dh-python debhelper pkg-config python3-setuptools

RUN git clone https://github.com/adjacentlink/waveform-resource -b develop

WORKDIR waveform-resource
RUN ./autogen.sh && ./configure && make deb
RUN cp $(find .debbuild -name "*\.deb") /opt/built
RUN dpkg -i /opt/built/*; apt-get -y install -f

RUN echo 'complete'

