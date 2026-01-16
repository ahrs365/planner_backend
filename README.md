# Planner Backend

[English](#english) | [中文](#中文)

---

<a name="english"></a>
## English

Local planning algorithms backend for the web robot simulation.

### Planners

| Planner | Description | Port |
|---------|-------------|------|
| DWA | Dynamic Window Approach | 9001 |
| DDR-OPT | JPS + Trajectory Optimization | 9002 |
| TMPC | Timed Model Predictive Control | 9003 |

### Prerequisites

- CMake 3.20+
- C++17 compiler (GCC 9+ / Clang 10+)
- Git

### Build

```bash
git clone <repo_url>
cd planner_backend

mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Run

```bash
./src/servers/dwa_server        # Port 9001
./src/servers/ddr_opt_server    # Port 9002
./src/servers/tmpc_server       # Port 9003
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
    └── servers/        # WebSocket servers
```

---

<a name="中文"></a>
## 中文

Web 机器人仿真的本地规划算法后端。

### 规划器

| 规划器 | 说明 | 端口 |
|--------|------|------|
| DWA | 动态窗口法 | 9001 |
| DDR-OPT | JPS + 轨迹优化 | 9002 |
| TMPC | 时序模型预测控制 | 9003 |

### 环境要求

- CMake 3.20+
- C++17 编译器 (GCC 9+ / Clang 10+)
- Git

### 编译

```bash
git clone <repo_url>
cd planner_backend

mkdir build && cd build
cmake ..
make -j$(nproc)
```

### 运行

```bash
./src/servers/dwa_server        # 端口 9001
./src/servers/ddr_opt_server    # 端口 9002
./src/servers/tmpc_server       # 端口 9003
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
    └── servers/        # WebSocket 服务器
```

---

## License

MIT
