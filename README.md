# Bundle Adjustment Project

This repository contains a C++ Bundle Adjustment implementation using the Ceres Solver, OpenCV, Eigen, and PCL libraries.

---

## Requirements

- Docker (recommended for easy setup)
- Or a Linux system with dependencies installed manually

---

## Build and Run Instructions

### Using Docker (recommended)

1. **Build the Docker image**

```bash
./build_docker.sh
```

2. **Run the Docker container**

```bash
./run_docker.sh
```

3. **Build the project inside the container**

```bash
./build_project.sh
```
- To clean and rebuild, use:
```bash
./build_project.sh --clean
```

4. **Run the executable**

```bash
./build/ba_main
```


---

### Without Docker (manual build)

1. Install dependencies: Ceres, OpenCV, Eigen, PCL (depending on your system).

2. Run the build script:

```bash
./build_project.sh
```

3. **Run the executable**

```bash
./build/ba_main
```
---