FROM ubuntu:22.04

# Set environment variables to avoid user interaction during install
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    unzip \
    libeigen3-dev \
    libopencv-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    libboost-all-dev \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

# Build and install Ceres Solver
RUN git clone https://github.com/ceres-solver/ceres-solver.git /ceres && \
    cd /ceres && \
    git submodule update --init --recursive && \
    mkdir build && cd build && \
    cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF && \
    make -j$(nproc) && make install && \
    ldconfig

# Set working directory
WORKDIR /workspace

# Default command
CMD ["/bin/bash"]