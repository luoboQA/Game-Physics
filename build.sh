#!/bin/bash

echo "========================================"
echo "Game Physics Engine"
echo "========================================"

# 清理功能
if [ "$1" == "clean" ]; then
    echo "Cleaning build directory..."
    rm -rf build
    echo "Done."
    exit 0
fi

# 创建build目录
mkdir -p build
cd build

# 配置和编译
cmake ..
make -j4

echo ""
echo "========================================"
echo "Build Complete!"
echo "========================================"
echo ""
echo "Run: cd build && ./stage*"
echo "Clean: ./build.sh clean"