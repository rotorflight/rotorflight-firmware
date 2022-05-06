FROM ubuntu:bionic

RUN apt-get -y update
RUN apt-get -y upgrade
RUN apt-get -y install \
    build-essential \
    python \
    curl \
    git

RUN mkdir /rotorflight
WORKDIR /rotorflight
