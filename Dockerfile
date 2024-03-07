FROM ubuntu:22.04

RUN apt-get update \
    && apt-get install -y tzdata \
    && apt-get install -y \
        build-essential \
        fakeroot \
        git \
        make \
        cmake \
        lcov \
        valgrind \
        doxygen \
        python-all-dev \
        python3-dev \
        python3-pip \
        python3-setuptools \
        sudo \
        curl \
        wget \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2
RUN tar -xf gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2
RUN rm gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2
RUN ln -s gcc-arm-none-eabi-10.3-2021.10/bin/arm-* /usr/local/bin/
RUN pip install --upgrade jinja2 pygments pros-cli

ENV LC_ALL C.UTF-8
ENV LANG C.UTF-8

WORKDIR /
