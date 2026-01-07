#!/bin/bash

# 1. 立即获取并提升 sudo 权限
sudo -v

echo "--- Starting Emergency Stop ---"

# 使用 -9 信号 (SIGKILL)，这是 Linux 中权限最高的杀进程信号，进程无法忽略
echo "[Step 2] Force killing all processes..."

# 杀掉目标程序 (使用 -f 匹配完整命令行，防止漏杀)
sudo pkill -9 -f "dji_sdk_demo_linux_cxx"
sudo pkill -9 -f "yesense_main"

#验证是否还活着
echo "---------------------------------------"
echo "Verifying..."
# 检查是否还有残留进程
CHECK=$(pgrep -f "dji_sdk_demo|yesense|infra|dvsense")
if [ -z "$CHECK" ]; then
    echo "SUCCESS: All processes have been terminated."
else
    echo "WARNING: Some processes are still running:"
    ps -ef | grep -E "dji_sdk_demo|yesense|infra|dvsense" | grep -v grep
fi
