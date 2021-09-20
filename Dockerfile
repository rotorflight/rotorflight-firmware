FROM ubuntu:bionic
RUN apt-get -y update
RUN apt-get -y upgrade
RUN apt-get -y install \
    build-essential \
    curl \
    git \
    python

RUN mkdir /rotorflight
WORKDIR /rotorflight