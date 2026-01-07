#!/bin/bash

# 1. 预先获取并缓存 sudo 权限
sudo -v

# 2. 为 dvsense_recorder 创建命名管道
FIFO_PATH="/tmp/dvsense_fifo"
[ -p $FIFO_PATH ] || mkfifo $FIFO_PATH

echo "--- Startup Process ---"

# 3. 启动各个应用并放入后台
# 使用重定向将输出存入日志，方便调试
sudo ./dji_sdk_demo_linux_cxx > dji.log 2>&1 &
echo "Started DJI SDK Demo."

sudo ./yesense_main > yesense.log 2>&1 &
echo "Started Yesense Main."

./infrared_driver 1 0 > infrared.log 2>&1 &
echo "Started Infrared Driver."

# 通过管道运行 dvsense_recorder，以便能接收 'q'
( tail -f $FIFO_PATH ) | ./dvsense_recorder > dvsense.log 2>&1 &
echo "Started DVsense Recorder (with command pipe)."

echo "---------------------------------------"
echo "All programs are running. The system will auto-shutdown in 15 minutes."
echo "Time: $(date)"

# 4. 倒计时 15 分钟 (900秒)
sleep 900

echo "---------------------------------------"
echo "Time is up! Starting Shutdown Sequence..."

# 5. 第一阶段：尝试温和退出 (针对 dvsense_recorder)
if [ -p "$FIFO_PATH" ]; then
    echo "Sending 'q' to dvsense_recorder..."
    echo "q" > $FIFO_PATH
fi

# 给程序 5 秒钟处理退出逻辑（保存数据、关闭视频流）
sleep 5

# 6. 第二阶段：强制清理 (针对所有进程)
# 使用 -9 (SIGKILL) 和 -f (全命令行匹配) 确保彻底杀死
echo "Force cleaning up all remaining processes..."

# 杀掉后台辅助进程
sudo pkill -9 -f "tail -f $FIFO_PATH" > /dev/null 2>&1

# 杀掉主要程序
sudo pkill -9 -f "dji_sdk_demo_linux_cxx"
sudo pkill -9 -f "yesense_main"
sudo pkill -9 -f "infrared_driver"
sudo pkill -9 -f "dvsense_recorder"

# 7. 清理临时管道文件
sudo rm -f $FIFO_PATH

echo "Auto-shutdown complete. All processes terminated."
echo "Time: $(date)"