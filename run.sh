#!/bin/bash
# 运行 MotionFlow 程序

cd "$(dirname "$0")/build/bin" || exit 1
./motionflow "$@"
