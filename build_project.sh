#!/bin/bash

BUILD_DIR="build"
BUILD_TYPE="Release"  # Default

# Help message
show_help() {
    echo "Usage: ./build_project.sh [--clean] [--debug|--release] [--help|-h]"
    echo ""
    echo "Options:"
    echo "  --clean        Delete the build/ directory before building"
    echo "  --debug        Build with debugging flags (default is Release)"
    echo "  --release      Build with optimizations (default)"
    echo "  -h, --help     Show this help message"
}

# Parse flags
for arg in "$@"; do
    case "$arg" in
        --clean)
            echo "Cleaning build directory..."
            rm -rf "$BUILD_DIR"
            ;;
        --debug)
            BUILD_TYPE="Debug"
            ;;
        --release)
            BUILD_TYPE="Release"
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown option: $arg"
            show_help
            exit 1
            ;;
    esac
done

# Configure and build
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR" || exit 1

echo "Configuring project (${BUILD_TYPE})..."
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE ..

echo "Building project..."
make -j$(nproc)
