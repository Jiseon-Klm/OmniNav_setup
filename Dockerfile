# =============================================================================
# OmniNav GPU Dockerfile - aarch64 (ARM64) + CUDA 13.0
# =============================================================================
# 이 Dockerfile은 NVIDIA DGX Spark GB10 (Grace Blackwell) ARM64 GPU 시스템용입니다.
# 테스트 환경: NVIDIA GB10, Compute Capability 12.0 (sm_120), Driver 580.x, CUDA 13.0
#
# ⚠️  중요: GB10 (Blackwell) 지원을 위해 NGC PyTorch 25.09+ 이미지 사용
#     공식 PyTorch wheel은 GB10/Blackwell을 지원하지 않습니다!
#
# 빌드: docker build -f Dockerfile.aarch64 -t omninav:aarch64 .
# 실행: docker run --gpus all --ipc=host --ulimit memlock=-1 --ulimit stack=67108864 -it omninav:aarch64
# =============================================================================

# NGC PyTorch 이미지 사용 (GB10 Blackwell + CUDA 13.0 지원)
# aarch64용 최신 버전 확인: https://catalog.ngc.nvidia.com/orgs/nvidia/containers/pytorch
# GB10 지원이 추가된 25.09 버전 사용 (CUDA 13.0, PyTorch 2.9.0, Driver 580.x 호환)
FROM nvcr.io/nvidia/pytorch:25.09-py3

# 환경 변수 설정
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=${CUDA_HOME}/bin:${PATH}
ENV LD_LIBRARY_PATH=${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}

# 병렬 빌드를 위한 코어 수 계산 (시스템이 터지지 않도록 90% 사용, 최소 2개)
# Docker 빌드 시 --build-arg BUILD_JOBS=N 으로 오버라이드 가능
ARG BUILD_JOBS=0
RUN CORES=$(nproc) && \
    if [ "$BUILD_JOBS" = "0" ] || [ -z "$BUILD_JOBS" ]; then \
        BUILD_JOBS=$(python3 -c "import math; print(max(2, int($CORES * 0.9)))"); \
    fi && \
    echo "export BUILD_JOBS=$BUILD_JOBS" >> /root/.bashrc && \
    echo "$BUILD_JOBS" > /tmp/build_jobs.txt && \
    echo "BUILD_JOBS set to: $BUILD_JOBS (cores: $CORES)"

# 한국 미러로 변경 (aarch64 ports - Kakao 미러)
RUN sed -i 's|http://ports.ubuntu.com/ubuntu-ports|http://mirror.kakao.com/ubuntu-ports|g' /etc/apt/sources.list /etc/apt/sources.list.d/*.list 2>/dev/null || \
    sed -i 's|http://ports.ubuntu.com|http://mirror.kakao.com|g' /etc/apt/sources.list /etc/apt/sources.list.d/*.list 2>/dev/null || true

# 기본 패키지 + ROS2 의존성 + RealSense 의존성 통합 설치 (apt-get update 1회만)
# Ubuntu 24.04 (Noble) 호환: libasound2 → libasound2t64
RUN apt-get update && apt-get install -y \
    # 기본 빌드 도구
    build-essential \
    cmake \
    git \
    wget \
    curl \
    vim \
    ninja-build \
    pkg-config \
    # OpenGL/GUI 라이브러리
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgomp1 \
    libegl1 \
    libegl-dev \
    libxrandr2 \
    libxss1 \
    libxcursor1 \
    libxinerama1 \
    libxi6 \
    libpangocairo-1.0-0 \
    libatk1.0-0 \
    libcairo-gobject2 \
    libgtk-3-0 \
    libgdk-pixbuf2.0-0 \
    libglfw3-dev \
    # ROS2 의존성
    locales \
    software-properties-common \
    gnupg \
    lsb-release \
    # RealSense SDK 의존성
    libssl-dev \
    libusb-1.0-0-dev \
    libgtk-3-dev \
    && (apt-get install -y libasound2t64 || apt-get install -y libasound2 || true) \
    && rm -rf /var/lib/apt/lists/*

# pip 업그레이드 (NGC 이미지에 이미 PyTorch가 설치되어 있음)
RUN pip install --no-cache-dir --upgrade pip setuptools wheel

# 공통 Python 의존성 설치 (aarch64 + CUDA 13.0 호환)
RUN pip install --no-cache-dir \
    numpy==1.26.4 \
    opencv-python-headless \
    pillow \
    matplotlib \
    scipy \
    tqdm \
    omegaconf \
    hydra-core \
    numba \
    qwen-vl-utils==0.0.10 \
    accelerate \
    datasets \
    safetensors \
    sentencepiece \
    einops \
    peft \
    trl==0.24.0 \
    diffusers \
    "modelscope[datasets]>=1.19" \
    "gradio>=4.0.0" \
    fastapi \
    uvicorn \
    tensorboard \
    pandas \
    nltk \
    rouge \
    requests \
    aiohttp \
    addict \
    attrdict \
    dacite \
    charset_normalizer \
    binpacking \
    importlib_metadata \
    jieba \
    openai \
    oss2 \
    tiktoken \
    jsonlines \
    transformers_stream_generator \
    zstandard \
    blake3 \
    pydantic>=2.0.0 \
    pydantic_core>=2.0.0

# deepspeed - NGC 이미지에 이미 포함되어 있을 수 있음
RUN pip install --no-cache-dir deepspeed || echo "deepspeed installation skipped (may already exist)"

# liger-kernel - aarch64에서 빌드 필요할 수 있음
RUN pip install --no-cache-dir liger-kernel || echo "liger-kernel installation skipped on aarch64"

# triton은 NGC 이미지에 이미 포함됨 (aarch64 + CUDA 13.0 호환)
RUN pip install --no-cache-dir --upgrade triton || echo "triton upgrade skipped on aarch64"

# open3d는 aarch64에서 공식 wheel이 없을 수 있음 - 설치 시도
RUN pip install --no-cache-dir open3d || echo "open3d installation skipped on aarch64"

# cpm_kernels는 x86 전용일 수 있음 - 설치 시도
RUN pip install --no-cache-dir cpm_kernels || echo "cpm_kernels installation skipped on aarch64"

# 작업 디렉토리 설정
WORKDIR /workspace

# transformers-main 설치 (editable mode) - NGC 이미지의 transformers와 버전 맞춤
COPY train_code/transformers-main /workspace/train_code/transformers-main
RUN cd /workspace/train_code/transformers-main && pip install --no-cache-dir -e . || true

# =============================================================================
# ROS2 Jazzy Jalisco 설치 (Ubuntu 24.04 호환)
# =============================================================================
ENV ROS_DISTRO=jazzy
ENV ROS_PYTHON_VERSION=3

# ROS2 GPG 키 추가 및 저장소 설정
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list

    
# ROS2 Jazzy 설치 (ros-base: GUI 도구 제외, 용량 1/3)
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-base \
    ros-jazzy-realsense2-camera \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    ros-jazzy-compressed-image-transport
    && rm -rf /var/lib/apt/lists/*

# rosdep 초기화
RUN rosdep init || true && \
    rosdep update

# ROS2 환경 변수 설정
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "export ROS_DISTRO=jazzy" >> /root/.bashrc

# ROS2 디렉토리 생성
RUN mkdir -p /when2reason_ws/src

# =============================================================================
# Intel RealSense SDK 설치 (Python 바인딩 포함)
# =============================================================================
WORKDIR /when2reason_ws
RUN git clone https://github.com/IntelRealSense/librealsense.git realsense-sdk

WORKDIR /when2reason_ws/realsense-sdk
RUN mkdir build && cd build && \
    BUILD_JOBS=$(cat /tmp/build_jobs.txt 2>/dev/null || python3 -c "import math; print(max(2, int($(nproc) * 0.9)))") && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_PYTHON_BINDINGS:bool=true \
        -DPYTHON_EXECUTABLE=$(which python3) \
        -DBUILD_EXAMPLES:bool=true \
        -DBUILD_GRAPHICAL_EXAMPLES:bool=true \
        -DFORCE_RSUSB_BACKEND=true && \
    make -j$BUILD_JOBS && \
    make install && \
    ldconfig && \
    PY_SITE=$(python3 -c "import site; print(site.getsitepackages()[0])") && \
    find . -name "pyrealsense2*.so" -exec cp {} $PY_SITE/ \; || \
    find /usr/local/lib -name "pyrealsense2*.so" -exec cp {} $PY_SITE/ \; || true

# =============================================================================
# ROS2 Control 및 추가 의존성 설치
# =============================================================================
RUN apt-get update && apt-get install -y \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-twist-mux \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-diff-drive-controller \
    && rm -rf /var/lib/apt/lists/*
# =============================================================================
# Scout Mini ROS2 설치 및 빌드
# =============================================================================
WORKDIR /when2reason_ws/src
RUN git clone https://github.com/roasinc/scout_mini_ros2.git && \
    vcs import . < scout_mini_ros2/requirement.rosinstall

WORKDIR /when2reason_ws
RUN . /opt/ros/jazzy/setup.sh && \
    BUILD_JOBS=$(cat /tmp/build_jobs.txt 2>/dev/null || python3 -c "import math; print(max(2, int($(nproc) * 0.9)))") && \
    apt-get update && \
    rosdep init || true && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro jazzy && \
    # 먼저 메시지 패키지 빌드 (의존성 순서 보장)
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --packages-select scout_mini_msgs ros2_socketcan_msgs \
        --parallel-workers $BUILD_JOBS && \
    # 설치 디렉토리 소스하여 다른 패키지가 찾을 수 있도록 함
    . install/setup.sh && \
    # 나머지 패키지 빌드
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --parallel-workers $BUILD_JOBS

# Scout Mini 및 RealSense 환경 변수 설정
RUN echo "source /when2reason_ws/install/setup.bash" >> /root/.bashrc

# 프로젝트 코드 복사 (data/, models/ 제외 - 마운트로 사용)
# 큰 폴더들은 .dockerignore로 제외됨

# # 설정 파일들
# COPY *.md /workspace/OmniNav/
# COPY *.sh /workspace/OmniNav/
# COPY *.yml /workspace/OmniNav/
# COPY *.yaml /workspace/OmniNav/
# data/, models/는 런타임에 마운트로 사용
WORKDIR /workspace/OmniNav

# 기본 명령어
CMD ["/bin/bash"]
