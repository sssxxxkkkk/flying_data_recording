import pandas as pd
import numpy as np

def calculate_timestamp_diffs(input_file, output_file=None):
    """
    计算相机时间戳、系统时间戳、delta_t与前一帧的差值
    
    参数:
        input_file: 输入文件路径，每行格式为 "cam_timestamp system_time delta_t"
    """
    # 读取数据
    data = []
    with open(input_file, 'r') as f:
            # 跳过第一行（标题行）
            next(f)  # 跳过标题行 "cam_timestamp system_time delta_t"
            for line in f:
                line = line.strip()
                if not line:
                    continue
                parts = list(map(int, line.split()))
                if len(parts) != 3:
                    print(f"警告: 无效行格式，跳过: {line}")
                    continue
                data.append(parts)
    
    if not data:
        print("错误: 文件中没有有效数据")
        return
    
    # 转换为DataFrame方便处理
    df = pd.DataFrame(data, columns=['cam_timestamp', 'system_time', 'delta_t'])
    
    # 计算与前一帧的差值
    df['cam_diff'] = df['cam_timestamp'].diff().fillna(0)
    df['system_diff'] = df['system_time'].diff().fillna(0)
    df['delta_diff'] = df['delta_t'].diff().fillna(0)
    
    # 计算统计信息
    stat_info = {
        "相机间隔均值(μs)": df['cam_diff'][1:].mean(),
        "相机间隔标准差(μs)": df['cam_diff'][1:].std(),
        "系统间隔均值(μs)": df['system_diff'][1:].mean(),
        "系统间隔标准差(μs)": df['system_diff'][1:].std(),
        "delta_t均值(μs)": df['delta_diff'][1:].mean(),
        "delta_t标准差(μs)": df['delta_diff'][1:].std()
    }
    
    # 打印结果
    print("="*80)
    print("\n时间戳差值计算结果:")
    print(df[['cam_diff', 'system_diff', 'delta_diff']].to_string(index=False))
    print("\n统计信息:")
    for key, value in stat_info.items():
        print(f"{key}: {value:.2f}")
    print("\n" + "="*80)
    
    
    return df, stat_info

def analyze_time_jitter(df, threshold=10000):
    """
    分析系统时间戳抖动情况
    
    参数:
        df: 包含diff数据的DataFrame
        threshold: 认为是异常抖动的阈值(μs)
    """
    print("\n时间戳抖动分析:")
    
    # 找出异常大的系统间隔
    abnormal = df[df['system_diff'] > threshold]
    if not abnormal.empty:
        print(f"\n检测到 {len(abnormal)} 个异常大的系统间隔 (> {threshold}μs):")
        for idx, row in abnormal.iterrows():
            print(f"行#{idx+1}: {row['system_diff']:.0f}μs, 相机间隔: {row['cam_diff']:.0f}μs")
    else:
        print(f"所有系统间隔均在阈值 {threshold}μs 以内")
    
    # 分析间隔模式(检测双峰分布)
    system_diffs = df['system_diff'].values[1:]
    if len(system_diffs) < 10:
        return
    

# 示例使用
if __name__ == "__main__":
    import sys

    input_file = "../save_data/event_data/sync_signal.txt"
    df, stats = calculate_timestamp_diffs(input_file)
    analyze_time_jitter(df)
