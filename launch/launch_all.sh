#!/bin/bash

# 1. 自动获取并持续更新 sudo 权限
sudo -v
# 启动一个后台进程，每 1 分钟更新一次 sudo 凭据，防止 15 分钟内权限过期
while true; do sudo -n true; sleep 60; kill -0 "$$" || exit; done 2>/dev/null &

# 2. 准备工作：创建日志目录并确保权限
LOG_DIR="Logs"
if [ ! -d "$LOG_DIR" ]; then
    mkdir -p "$LOG_DIR"
fi
# 确保当前用户和 root 都能读写日志目录
sudo chmod 777 "$LOG_DIR"

# 3. 准备 dvsense_recorder 的命令命名管道 (FIFO)
FIFO_PATH="/tmp/dvsense_fifo"
rm -f $FIFO_PATH  # 先清理旧的
mkfifo $FIFO_PATH
chmod 666 $FIFO_PATH # 确保所有用户都能写入该管道

echo "--- Startup Process ---"

# 4. 运行各个核心模块
# 使用 sudo sh -c "命令 > 日志" 的格式，确保重定向和参数传递万无一失

sudo sh -c "./dji_sdk_demo_linux_cxx> $LOG_DIR/dji.log 2>&1" &
echo "Started DJI SDK Demo."

sudo sh -c "./yesense_main > $LOG_DIR/yesense.log 2>&1" &
echo "Started Yesense Main."

# 关键修正：确保参数 1 0 包含在 sh -c 的字符串里
sudo sh -c "./infrared_driver 1 0 > $LOG_DIR/infrared.log 2>&1" &
echo "Started Infrared Driver with Params 1 0."

# 启动 dvsense_recorder，通过管道接收退出指令 'q'
# tail -f 会持续保持打开状态，直到被 pkill
( tail -f $FIFO_PATH ) | sudo ./dvsense_recorder 1 1 > $LOG_DIR/dvsense.log 2>&1 &
echo "Started DVsense Recorder (with command pipe)."

echo "---------------------------------------"
echo "All programs are running. The system will auto-shutdown in 15 minutes."
echo "Time: $(date)"

# 5. 等待 15 分钟 (900秒)
sleep 900

echo "---------------------------------------"
echo "Time is up! Starting Shutdown Sequence..."

# 6. 安全退出交互式程序 (向管道发送 'q')
if [ -p "$FIFO_PATH" ]; then
    echo "Sending 'q' to dvsense_recorder..."
    echo "q" > $FIFO_PATH
fi

# 等待 5 秒让程序处理退出逻辑并保存文件
sleep 5

# 7. 强制清理剩余进程 (防止有些程序没响应 'q')
echo "Cleaning up all remaining processes..."

# 杀掉后台所有的相关进程
sudo pkill -9 -f "tail -f $FIFO_PATH" > /dev/null 2>&1
sudo pkill -9 -f "dji_sdk_demo_linux_cxx"
sudo pkill -9 -f "yesense_main"
sudo pkill -9 -f "infrared_driver"
sudo pkill -9 -f "dvsense_recorder"

# 8. 清理管道文件
rm -f $FIFO_PATH

echo "Auto-shutdown complete. All processes terminated."
echo "Time: $(date)"
