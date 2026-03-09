#!/bin/bash

# 1. 预先获取并缓存 sudo 权限
sudo -v

echo "--- Startup Process ---"

# 使用重定向将输出存入日志，方便调试
sudo ./dji_sdk_demo_linux_cxx > dji.log 2>&1 &
echo "Started DJI SDK Demo."

sudo ./yesense_main > yesense.log 2>&1 &
echo "Started Yesense Main."

echo "---------------------------------------"
echo "All programs are running. The system will auto-shutdown in 15 minutes."
echo "Time: $(date)"W

# 4. 倒计时 15 分钟 (900秒)
sleep 900

echo "---------------------------------------"
echo "Time is up! Starting Shutdown Sequence..."

# 给程序 5 秒钟处理退出逻辑（保存数据、关闭视频流）
sleep 5

# 6. 第二阶段：强制清理 (针对所有进程)
# 使用 -9 (SIGKILL) 和 -f (全命令行匹配) 确保彻底杀死
echo "Force cleaning up all remaining processes..."

# 杀掉主要程序
sudo pkill -9 -f "dji_sdk_demo_linux_cxx"
sudo pkill -9 -f "yesense_main"

echo "Auto-shutdown complete. All processes terminated."
echo "Time: $(date)"
