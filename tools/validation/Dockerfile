FROM ubuntu:24.04
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential clang llvm lld libclang-rt-18-dev libblocksruntime-dev python3 git curl ca-certificates \
    bzip2 xz-utils file make && rm -rf /var/lib/apt/lists/*
ARG TARGETARCH=arm64
RUN set -eu; case "$TARGETARCH" in arm64) ARM_ARCH=aarch64; NODE_ARCH=arm64 ;; amd64) ARM_ARCH=x86_64; NODE_ARCH=x64 ;; *) exit 1 ;; esac; \
    curl -fsSL "https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2020q2/gcc-arm-none-eabi-9-2020-q2-update-${ARM_ARCH}-linux.tar.bz2" | tar -xj -C /opt; \
    curl -fsSL "https://nodejs.org/dist/v22.17.0/node-v22.17.0-linux-${NODE_ARCH}.tar.xz" | tar -xJ -C /opt; \
    ln -s "/opt/node-v22.17.0-linux-${NODE_ARCH}" /opt/node
ENV ARM_SDK_DIR=/opt/gcc-arm-none-eabi-9-2020-q2-update
ENV PATH=/opt/node/bin:/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:$PATH
WORKDIR /work/firmware
