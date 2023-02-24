FROM fedora:37

RUN mkdir -p /opt/built

RUN dnf -y update

# Copy and install packages from the latest opentestpoint build
WORKDIR /opt

# Build tools
RUN dnf -y install git make autoconf automake libtool rpm-build python3-setuptools

RUN git clone https://github.com/adjacentlink/waveform-resource -b develop

WORKDIR waveform-resource
RUN ./autogen.sh && ./configure && make rpm
RUN cp $(find .rpmbuild/RPMS -name "*\.rpm") /opt/built
RUN dnf -y install /opt/built/*

RUN dnf clean all

RUN echo 'complete'

