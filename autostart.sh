#!/bin/bash

# ============================================================================
# 视觉系统启动脚本 (Vision System Autostart Script)
# ============================================================================
# 此脚本在系统启动时自动运行，必须立即返回，不能阻塞！
# ============================================================================

set -e

# 配置
WORK_DIR="/home/rmul/DUST_Infantry_Vision"
PROGRAM="infantry_debug"
CONFIG_FILE="configs/standard3.yaml"
LOG_DIR="/tmp/sp_vision_logs"
LOG_FILE="$LOG_DIR/autostart_$(date +%Y%m%d_%H%M%S).log"

# 创建日志目录
mkdir -p "$LOG_DIR"

{
    echo "=========================================="
    echo "启动时间: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "系统架构: $(uname -m)"
    echo "工作目录: $WORK_DIR"
    echo "程序: $PROGRAM"
    echo "配置文件: $CONFIG_FILE"
    echo "=========================================="
    
    # 等待系统就绪
    echo "[INFO] 等待系统就绪 (5 秒)..."
    sleep 10
    
    # 进入工作目
    cd "$WORK_DIR" || {
        echo "[ERROR] 无法进入工作目录: $WORK_DIR"
        exit 1
    }
    
    # 检查程序是否存在
    if [ ! -f "build/$PROGRAM" ]; then
        echo "[ERROR] 程序不存在: build/$PROGRAM"
        ls -la build/ | head -20
        exit 1
    fi
    
    # 检查配置文件
    if [ ! -f "$CONFIG_FILE" ]; then
        echo "[ERROR] 配置文件不存在: $CONFIG_FILE"
        exit 1
    fi
    
    echo "[INFO] 启动程序: ./build/$PROGRAM $CONFIG_FILE"
    
    # ============================================================================
    # 🔴 关键：设置库路径以找到依赖库（libMVSDK.so 等）
    # ============================================================================
    # 检测系统架构并设置相应的库路径
    ARCH=$(uname -m)
    if [ "$ARCH" = "x86_64" ]; then
        LIB_PATH="$WORK_DIR/io/mindvision/lib/amd64"
    elif [ "$ARCH" = "aarch64" ]; then
        LIB_PATH="$WORK_DIR/io/mindvision/lib/arm64"
    else
        LIB_PATH="$WORK_DIR/io/mindvision/lib"
    fi
    
    echo "[INFO] 检测架构: $ARCH"
    echo "[INFO] 库路径: $LIB_PATH"
    
    if [ ! -d "$LIB_PATH" ]; then
        echo "[WARNING] 库路径不存在: $LIB_PATH"
        echo "[WARNING] 使用默认库路径"
        export LD_LIBRARY_PATH="$WORK_DIR/io/mindvision/lib:$LD_LIBRARY_PATH"
    else
        export LD_LIBRARY_PATH="$LIB_PATH:$WORK_DIR/io/mindvision/lib:$LD_LIBRARY_PATH"
    fi
    
    echo "[INFO] LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
    
    # ============================================================================
    # ⚠️ 重要：在后台启动程序并立即返回
    # 这样脚本会立即完成，不会阻塞自启进程
    # ============================================================================
    nohup ./build/$PROGRAM "$CONFIG_FILE" >> "$LOG_FILE" 2>&1 &
    
    PROGRAM_PID=$!
    echo "[INFO] 程序已启动 (PID: $PROGRAM_PID)"
    echo "[INFO] 日志文件: $LOG_FILE"
    echo "[SUCCESS] 启动完成！"
    
} >> "$LOG_FILE" 2>&1

# ✓ 立即退出（脚本完成），不阻塞自启进程
exit 0
