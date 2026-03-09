#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rosbag
import sys
import os

def main():
    if len(sys.argv) != 4:
        print("用法: python downsample_bag.py <input.bag> <output.bag> <topic_name>")
        print("示例: python downsample_bag.py raw.bag filtered.bag /camera/image_raw")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]
    target_topic = sys.argv[3]
    target_hz = 4.0
    min_interval = 1.0 / target_hz

    if not os.path.exists(input_file):
        print(f"错误: 文件 {input_file} 不存在")
        sys.exit(1)

    print(f"开始处理...")
    print(f"输入: {input_file}")
    print(f"输出: {output_file}")
    print(f"目标话题: {target_topic} -> {target_hz} Hz")

    kept_count = 0
    total_count = 0
    last_time = -1.0

    with rosbag.Bag(output_file, 'w') as outbag:
        with rosbag.Bag(input_file, 'r') as inbag:
            for topic, msg, t in inbag.read_messages():
                total_count += 1
                
                if topic == target_topic:
                    current_time = t.to_sec()
                    if last_time < 0 or (current_time - last_time) >= min_interval:
                        outbag.write(topic, msg, t)
                        last_time = current_time
                        kept_count += 1
                    # 否则丢弃该消息 (降频)
                else:
                    # 非目标话题原样保留
                    outbag.write(topic, msg, t)

    print(f"处理完成!")
    print(f"原始消息总数: {total_count}")
    print(f"目标话题保留消息数: {kept_count}")
    if total_count > 0:
        # 估算一下该话题原本的数量（这里简化计算，假设其他话题数量未知）
        print(f"降频比例约为: {kept_count / (total_count if kept_count==total_count else kept_count+1):.2f} (仅供参考)")

if __name__ == '__main__':
    main()
