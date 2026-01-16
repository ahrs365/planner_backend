# Planner Backend

Local planning algorithms backend for the web robot simulation.

## Planners

| Planner | Description | Port |
|---------|-------------|------|
| DWA | Dynamic Window Approach | 9001 |
| DDR-OPT | JPS + Trajectory Optimization | 9002 |
| TMPC | Timed Model Predictive Control | 9003 |

## Build

### Prerequisites

- CMake 3.20+
- C++17 compiler (GCC 9+ or Clang 10+)
- Git (for submodules)

### Clone & Build

```bash
git clone <repo_url>
cd planner_backend

# Initialize submodules
git submodule update --init --recursive

# Build
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Build Options

```bash
cmake .. -DBUILD_DWA_SERVER=ON -DBUILD_DDR_OPT_SERVER=ON -DBUILD_TMPC_SERVER=ON
```

## Run

Start any planner server:

```bash
./build/src/servers/dwa_server        # Port 9001
./build/src/servers/ddr_opt_server    # Port 9002
./build/src/servers/tmpc_server       # Port 9003
```

Then open the web simulation and the planner will auto-connect.

## Project Structure

```
backend/
├── CMakeLists.txt          # Top-level CMake
├── cmake/                  # CMake modules
├── third_party/            # Dependencies (git submodules)
│   ├── acados/
│   ├── asio/
│   ├── eigen/
│   ├── json/
│   ├── websocketpp/
│   └── yaml-cpp/
└── src/
    ├── dwa/                # DWA algorithm
    ├── ddr-opt/            # DDR optimization
    ├── tmpc/               # TMPC algorithm
    └── servers/            # WebSocket servers
```

## License

MIT

