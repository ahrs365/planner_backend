# Planner Backend (planner_backend)

![bspline](https://github.com/user-attachments/assets/bc85f2c3-53c3-424f-9a8b-94badbf39b09)

[English](#english) | [中文](#中文)

---

<a name="english"></a>
## English

Local planning algorithms backend for the web robot simulation.

### Planners

| Planner | Description | Port |
|---------|-------------|------|
| DWA | Dynamic Window Approach | 8081 |
| DDR-OPT | JPS + Trajectory Optimization | 8082 |
| TMPC | Timed Model Predictive Control | 8083 |
| BSpline | B-spline lattice planner | 8084 |

### Prerequisites

- CMake 3.20+
- C++17 compiler (GCC 9+ / Clang 10+)
- Git
- libgsl-dev
- boost
Install libgsl-dev and boost:

```bash
sudo apt-get update
sudo apt-get install libgsl-dev
sudo apt-get install libboost-all-dev
```

### Build

```bash
# Download and extract, then:
cd planner_backend
mkdir build && cd build
cmake ..
make -j$(nproc)
```

> All dependencies are included. No extra downloads needed.

### Run

```bash
./src/servers/dwa_server        # Port 8081
./src/servers/ddr_opt_server    # Port 8082
./src/servers/tmpc_server       # Port 8083
./src/servers/bspline_server    # Port 8084
```

Then open the web simulation - planner will auto-connect.

### Structure

```
backend/
├── CMakeLists.txt
├── cmake/
├── third_party/        # Dependencies
└── src/
    ├── dwa/            # DWA algorithm
    ├── ddr-opt/        # DDR optimization
    ├── tmpc/           # TMPC algorithm
    ├── bspline_planner/ # B-spline planner
    └── servers/        # WebSocket servers
```
https://github.com/ZJU-FAST-Lab/DDR-opt

https://github.com/tud-amr/mpc_planner

https://github.com/ahrs365/tmpc

---

<a name="中文"></a>
## 中文

Web 机器人仿真的本地规划算法后端。

### 规划器

| 规划器 | 说明 | 端口 |
|--------|------|------|
| DWA | 动态窗口法 | 8081 |
| DDR-OPT | JPS + 轨迹优化 | 8082 |
| TMPC | 时序模型预测控制 | 8083 |
| BSpline | B 样条格点规划 | 8084 |

### 环境要求

- CMake 3.20+
- C++17 编译器 (GCC 9+ / Clang 10+)
- Git
- libgsl-dev（唯一需要安装的系统依赖）
- boost(如果是纯净的新系统，也需要安装)
安装 libgsl-dev和boost：

```bash
sudo apt-get update
sudo apt-get install libgsl-dev
sudo apt-get install libboost-all-dev
```

### 编译

```bash
# 下载解压后:
cd planner_backend
mkdir build && cd build
cmake ..
make -j$(nproc)
```

> 所有依赖已内置，无需额外下载。

### 运行

```bash
./src/servers/dwa_server        # 端口 8081
./src/servers/ddr_opt_server    # 端口 8082
./src/servers/tmpc_server       # 端口 8083
./src/servers/bspline_server    # 端口 8084
```

启动后打开网页仿真，规划器会自动连接。

### 目录结构

```
backend/
├── CMakeLists.txt
├── cmake/
├── third_party/        # 依赖库
└── src/
    ├── dwa/            # DWA 算法
    ├── ddr-opt/        # DDR 优化
    ├── tmpc/           # TMPC 算法
    ├── bspline_planner/ # B 样条规划器
    └── servers/        # WebSocket 服务器
```

---

https://github.com/ZJU-FAST-Lab/DDR-opt

https://github.com/tud-amr/mpc_planner

https://github.com/ahrs365/tmpc

## License

MIT
