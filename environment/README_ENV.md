# Môi trường phát triển - Lite3 RL Deploy (CMPC)

## Tổng quan

Dự án C++ (CMake) điều khiển robot Lite3, với Python scripts hỗ trợ mô phỏng và phân tích.

## Cấu trúc môi trường

```
environment/
├── apt-packages-list.txt     # Danh sách apt packages (dpkg -l)
├── requirements.txt          # Danh sách pip packages (pip freeze)
├── README_ENV.md             # File này
└── setup_env.sh              # Script check + tải + build

third_party/
├── onnxruntime/              # [PHẢI TẢI] x86/ + arm/
│   └── [BUILD_PLATFORM]/
│       ├── include/
│       └── lib/
├── mujoco/                   # [CÓ SẴN] x86/ + arm/ (prebuilt libraries)
├── eigen/                    # [CÓ SẴN] Header-only
├── gamepad/                  # [CÓ SẴN] SDK gamepad
├── Lite3_MotionSDK/          # [CÓ SẴN] SDK robot Lite3
├── deep_robotics_model/      # [git submodule] https://github.com/DeepRoboticsLab/deep_robotics_model.git
├── pinocchio/                # [CÓ SẴN] Header-only
├── boost/                    # [CÓ SẴN] Boost headers
└── boost_1_84_0.tar.gz       # [CÓ SẴN] Boost source archive
```

## Cách setup trên máy mới

### 1. Clone repo + submodules

```bash
git clone <repo-url>
cd Lite3_rl_deploy
git submodule update --init --recursive
```

### 2. Chạy setup script

```bash
chmod +x environment/setup_env.sh
./environment/setup_env.sh
```

Script sẽ tự động:
- Cài đặt apt packages còn thiếu (sudo)
- Kiểm tra third_party/ dependencies
- Tải onnxruntime nếu thiếu
- Tạo Python venv + cài pip packages
- Build project (cmake + make)

### 3. Hoặc làm thủ công từng bước

#### System packages
```bash
sudo apt install -y \
  build-essential cmake \
  libeigen3-dev libglfw3-dev \
  libboost-all-dev
```

#### third_party/onnxruntime
```bash
cd third_party
# x86_64
wget https://github.com/microsoft/onnxruntime/releases/download/v1.15.1/onnxruntime-linux-x64-1.15.1.tgz
tar -xzf onnxruntime-linux-x64-1.15.1.tgz
mkdir -p onnxruntime/x86
mv onnxruntime-linux-x64-1.15.1 onnxruntime/x86

# aarch64 (cross-compile cho robot)
wget https://github.com/microsoft/onnxruntime/releases/download/v1.15.1/onnxruntime-linux-aarch64-1.15.1.tgz
tar -xzf onnxruntime-linux-aarch64-1.15.1.tgz
mkdir -p onnxruntime/arm
mv onnxruntime-linux-aarch64-1.15.1 onnxruntime/arm
```

#### Python venv
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install numpy mujoco pybullet matplotlib pandas colorama
```

#### Build
```bash
mkdir -p build && cd build
cmake .. -DBUILD_PLATFORM=x86 -DBUILD_SIM=ON
make -j$(nproc)
```

## Build options

| Option | Giá trị | Mô tả |
|--------|---------|-------|
| BUILD_PLATFORM | x86 / arm | Nền tảng build |
| BUILD_SIM | ON / OFF | Build với simulation |
| USE_PYBULLET | ON | Dùng PyBullet simulation (mặc định) |
| USE_MJCPP | OFF | Dùng MuJoCo C++ simulation |
| SEND_REMOTE | OFF | Build để deploy lên robot |
| USE_MOLD_LINKER | OFF | Dùng mold linker (apt install mold) |

## Cấu hình máy đang dùng

- OS: Ubuntu 24.04
- Compiler: GCC 13
- C++ Standard: 17
- CMake: 3.28.3
- Python: 3.12