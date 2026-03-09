import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R
import re
from scipy.spatial.transform import Slerp
import cv2
from scipy.signal import correlate
from scipy.ndimage import gaussian_filter1d
def parse_uav_data(file_path):
    """解析 UAV 数据: R_UAV_W (Body to World)"""
    data = []
    with open(file_path, 'r') as f:
        content = f.read()
    
    # 使用正则表达式找到所有"receive quaternion data."之间的内容
    pattern = r"receive quaternion data\.\s*\n(timestamp:\s+\d+\.\d+)\s*\n(quaternion:\s+[\d\.-]+\s+[\d\.-]+\s+[\d\.-]+\s+[\d\.-]+)"
    matches = re.findall(pattern, content)
    
    # 处理时间戳对
    last_efficient_timestamp = 0
    for i in range(len(matches) - 1):  # 避免超出索引
        # 解析第一个数据点
        first_match = matches[i]
        first_ts_str = first_match[0]
        first_quat_str = first_match[1]
        
        first_ts_match = re.search(r"timestamp:\s+(\d+\.\d+)", first_ts_str)
        if not first_ts_match:
            continue
        first_ts = float(first_ts_match.group(1))
        
        first_quat_match = re.search(r"quaternion:\s+([\d\.-]+)\s+([\d\.-]+)\s+([\d\.-]+)\s+([\d\.-]+)\.", first_quat_str)
        if not first_quat_match:
            continue
        
        first_quat = [float(first_quat_match.group(2)), float(first_quat_match.group(3)), 
                     float(first_quat_match.group(4)), float(first_quat_match.group(1))]
        
        # 解析第二个数据点
        second_match = matches[i + 1]
        second_ts_str = second_match[0]
        
        second_ts_match = re.search(r"timestamp:\s+(\d+\.\d+)", second_ts_str)
        if not second_ts_match:
            continue
        second_ts = float(second_ts_match.group(1))
        
        # 检查条件：第二个时间戳比第一个大，且差值小于0.05
        time_diff = second_ts - first_ts
        if time_diff > 0 and time_diff < 0.05 and first_ts > last_efficient_timestamp:
            # 检查四元数是否有效并添加第一个数据点
            if np.linalg.norm(first_quat) > 0:
                rot = R.from_quat(first_quat)
                data.append({'ts': first_ts, 'rot': rot})
                last_efficient_timestamp  = first_ts
        else:
            # 不满足条件，继续处理后续数据
            continue
    
    return data

def parse_iner_data(file_path):
    """解析惯导数据: R^IMU_W_Body (Body to World)"""
    data = []
    prv_ts = 0
    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):  # 跳过空行和注释
                continue
            parts = line.split(',')
            if len(parts) < 13: 
                continue
            try:
                ts = int(parts[1]) + int(parts[2]) / 1e9

                if ts < prv_ts:
                    continue
                
                prv_ts = ts
                # 按照数据格式: num, time_stample_sec, time_stample_nsec, angle_rate.x, angle_rate.y, 
                # angle_rate.z, accel.x, accel.y, accel.z, quaternion0, quaternion1, quaternion2, quaternion3
                # 通常q0是标量部分w，q1,q2,q3是x,y,z向量部分
                # scipy使用[x,y,z,w]格式，所以需要从[w,x,y,z]转换为[x,y,z,w]
                quat_w = float(parts[9])   # q0
                quat_x = float(parts[10])  # q1
                quat_y = float(parts[11])  # q2
                quat_z = float(parts[12])  # q3
                
                # 转换为scipy期望的格式 [x, y, z, w]
                quat = [quat_x, quat_y, quat_z, quat_w]
                
                # 检查四元数是否有效
                if np.linalg.norm(quat) > 0:
                    rot = R.from_quat(quat)
                    data.append({'ts': ts, 'rot': rot})
            except (ValueError, IndexError):
                # 如果解析失败，跳过该行
                continue
    return data

def create_rotation_animation(uav_list, iner_list, estimated_delay, R_x=np.eye(3)):
    # 最近邻插值对齐
    iner_ts = np.array([d['ts'] for d in iner_list])
    iner_rots = R.from_quat([d['rot'].as_quat() for d in iner_list])
    slerp = Slerp(iner_ts, iner_rots)
    
    # 创建时间对齐的数据
    aligned_uav_rots = []
    aligned_iner_rots = []
    common_ts = []

    for uav_data in uav_list:
        ts = uav_data['ts'] - estimated_delay
        if iner_ts[0] <= ts <= iner_ts[-1]:
            R_uav_in_w = uav_data['rot'].as_matrix()
            R_body_in_c = slerp(ts).as_matrix()
            #R_iner_in_uav = np.linalg.inv(R_uav_in_w) @ R_x @ R_body_in_c
            #aligned_uav_rots.append(np.eye(3))
            
            R_iner_in_uav = R_x @ R_body_in_c
            aligned_uav_rots.append(R_uav_in_w)
            aligned_iner_rots.append(R_iner_in_uav)
            common_ts.append(ts)
            
    # 定义坐标轴向量（单位向量）
    axis_x = np.array([1, 0, 0])
    axis_y = np.array([0, 1, 0])
    axis_z = np.array([0, 0, 1])
    
    # 计算旋转后的坐标轴 - 现在使用矩阵乘法代替apply方法
    uav_x_axes = [rot @ axis_x for rot in aligned_uav_rots]
    uav_y_axes = [rot @ axis_y for rot in aligned_uav_rots]
    uav_z_axes = [rot @ axis_z for rot in aligned_uav_rots]
    
    iner_x_axes = [rot @ axis_x for rot in aligned_iner_rots]
    iner_y_axes = [rot @ axis_y for rot in aligned_iner_rots]
    iner_z_axes = [rot @ axis_z for rot in aligned_iner_rots]

    # 创建图形
    fig = plt.figure(figsize=(12, 8))
    
    # 创建3D子图
    ax = fig.add_subplot(111, projection='3d')
    
    # 初始化函数
    def init():
        ax.set_xlim([-1.5, 1.5])
        ax.set_ylim([-1.5, 1.5])
        ax.set_zlim([-1.5, 1.5])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('UAV vs Inertial Rotation Comparison')
        ax.grid(True)
        
        return []
    
    # 动画更新函数
    def update(frame):
        # 清除之前的绘图
        ax.clear()
        
        # 设置坐标轴范围和标签
        ax.set_xlim([-1.5, 1.5])
        ax.set_ylim([-1.5, 1.5])
        ax.set_zlim([-1.5, 1.5])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f'UAV vs Inertial Rotation (Frame {frame})')
        ax.grid(True)
        
        # 添加时间戳文本
        if frame < len(common_ts):
            timestamp_text = f'Time: {common_ts[frame]:.3f}s'
            ax.text2D(0.02, 0.95, timestamp_text, transform=ax.transAxes, fontsize=10, 
                      verticalalignment='top', bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8))
        
        # 绘制UAV坐标轴
        if frame < len(uav_x_axes):
            origin = np.array([0, 0, 0])
            # 绘制UAV的X轴（红色，较细）
            ax.quiver(*origin, *uav_x_axes[frame], color='red', alpha=1.0, arrow_length_ratio=0.1, linewidths=2)
            # 绘制UAV的Y轴（绿色，较细）
            ax.quiver(*origin, *uav_y_axes[frame], color='green', alpha=1.0, arrow_length_ratio=0.1, linewidths=2)
            # 绘制UAV的Z轴（蓝色，较细）
            ax.quiver(*origin, *uav_z_axes[frame], color='blue', alpha=1.0, arrow_length_ratio=0.1, linewidths=2)
            
            # 添加UAV轴标签
            ax.text(uav_x_axes[frame][0], uav_x_axes[frame][1], uav_x_axes[frame][2], 'UAV-X', color='red', fontsize=10)
            ax.text(uav_y_axes[frame][0], uav_y_axes[frame][1], uav_y_axes[frame][2], 'UAV-Y', color='green', fontsize=10)
            ax.text(uav_z_axes[frame][0], uav_z_axes[frame][1], uav_z_axes[frame][2], 'UAV-Z', color='blue', fontsize=10)
        
        # 绘制惯导坐标轴
        if frame < len(iner_x_axes):
            origin = np.array([0, 0, 0])
            # 绘制惯导的X轴（青色，较粗）
            ax.quiver(*origin, *iner_x_axes[frame], color='cyan', alpha=1.0, arrow_length_ratio=0.1, linewidths=4)
            # 绘制惯导的Y轴（黄色，较粗）
            ax.quiver(*origin, *iner_y_axes[frame], color='yellow', alpha=1.0, arrow_length_ratio=0.1, linewidths=4)
            # 绘制惯导的Z轴（洋红色，较粗）
            ax.quiver(*origin, *iner_z_axes[frame], color='magenta', alpha=1.0, arrow_length_ratio=0.1, linewidths=4)
            
            # 添加惯导轴标签
            ax.text(iner_x_axes[frame][0], iner_x_axes[frame][1], iner_x_axes[frame][2], 'IN-X', color='cyan', fontsize=10)
            ax.text(iner_y_axes[frame][0], iner_y_axes[frame][1], iner_y_axes[frame][2], 'IN-Y', color='yellow', fontsize=10)
            ax.text(iner_z_axes[frame][0], iner_z_axes[frame][1], iner_z_axes[frame][2], 'IN-Z', color='magenta', fontsize=10)
        
        return []
    
    # 创建动画
    ani = FuncAnimation(fig, update, frames=min(len(aligned_uav_rots), len(aligned_iner_rots)), 
                        init_func=init, blit=False, interval=50, repeat=True)
    
    plt.tight_layout()
    plt.show()
    
    try:
        # 尝试使用plt.waitforbuttonpress()等待用户交互
        print("动画正在显示，按任意键或关闭窗口以退出...")
        plt.waitforbuttonpress()
    except:
        # 如果waitforbuttonpress不可用，使用input()阻塞
        input("按Enter键退出...")
        
    return ani

def estimate_time_delay(uav_list, iner_list, max_delay_seconds=1.0, sample_rate_guess=200):
    """
    鲁棒的两个旋转系统时延估计函数
    """
    # --- 1. 提取时间戳和旋转 ---
    def clean_data(data_list):
        ts = np.array([d['ts'] for d in data_list])
        rots = [d['rot'] for d in data_list]
        
        return ts, rots

    uav_ts, uav_rots = clean_data(uav_list)
    iner_ts, iner_rots = clean_data(iner_list)

    # --- 2. 确定公共时间窗口 ---
    # 为了避免边界效应，我们在重叠区域两头各缩减一点时间
    overlap_start = max(uav_ts[0], iner_ts[0]) + 0.2
    overlap_end = min(uav_ts[-1], iner_ts[-1]) - 0.2

    if overlap_end - overlap_start < 2.0:
        print("错误: 重叠时间太短，无法可靠估计时延。")
        return 0.0, 0.0

    print(f"分析时间窗口: {overlap_end - overlap_start:.2f} 秒")

    # --- 3. 统一重采样 (Resampling) ---
    dt = 1.0 / sample_rate_guess
    common_time = np.arange(overlap_start, overlap_end, dt)

    # 构建插值器
    # 注意：SciPy 的 Slerp 需要旋转对象
    uav_slerp = Slerp(uav_ts, R.from_quat([r.as_quat() for r in uav_rots]))
    iner_slerp = Slerp(iner_ts, R.from_quat([r.as_quat() for r in iner_rots]))

      # 插值得到对齐后的旋转（现在时间轴是均匀的）
    uav_rots_interp = uav_slerp(common_time)
    iner_rots_interp = iner_slerp(common_time)

    # --- 4. 改进的角速度计算（使用中心差分） ---
    def get_angular_speed_magnitude_centered(rots, time_array):
        """
        使用中心差分计算角速度，更平滑
        """
        speeds = []
        for i in range(1, len(rots)-1):
            r_prev = rots[i-1]
            r_curr = rots[i]
            r_next = rots[i+1]
            
            # 使用中心差分：R_diff = R_{i-1}^{-1} * R_{i+1}
            # 这对应的时间跨度是 t_{i+1} - t_{i-1}
            r_diff = r_prev.inv() * r_next
            angle = r_diff.magnitude()
            
            # 时间跨度（中心差分的时间间隔是2*dt）
            dt_span = time_array[i+1] - time_array[i-1]
            
            if dt_span > 0:
                speed = angle / dt_span
            else:
                speed = 0.0
            
            speeds.append(speed)
        
        # 在两端补值（使用相邻值）
        if len(speeds) > 0:
            speeds = [speeds[0]] + speeds + [speeds[-1]]
        else:
            speeds = [0.0] * len(rots)
        
        return np.array(speeds)

    # 使用中心差分计算角速度
    uav_speed = get_angular_speed_magnitude_centered(uav_rots_interp, common_time)
    iner_speed = get_angular_speed_magnitude_centered(iner_rots_interp, common_time)

    # --- 5. 低通滤波 ---
    # 使用Savitzky-Golay滤波器，既能平滑又能保持信号特征
    from scipy.signal import savgol_filter
    
    # 窗口大小应根据采样率调整，一般取奇数
    window_length = min(11, len(uav_speed) - 1)  # 确保窗口长度小于数据长度
    if window_length % 2 == 0:  # 确保窗口长度为奇数
        window_length -= 1
    if window_length < 3:
        window_length = 3
        
    polyorder = 3  # 多项式阶数
    
    if len(uav_speed) > window_length and len(iner_speed) > window_length:
        uav_speed_smooth = savgol_filter(uav_speed, window_length, polyorder)
        iner_speed_smooth = savgol_filter(iner_speed, window_length, polyorder)
    else:
        # 如果数据太短，使用高斯滤波
        uav_speed_smooth = gaussian_filter1d(uav_speed, sigma=2)
        iner_speed_smooth = gaussian_filter1d(iner_speed, sigma=2)

    # --- 6. 归一化 ---
    def normalize(v):
        return (v - np.mean(v)) / (np.std(v) + 1e-10)

    uav_norm = normalize(uav_speed_smooth)
    iner_norm = normalize(iner_speed_smooth)

    # --- 7. 互相关分析 ---
    correlation = correlate(uav_norm, iner_norm, mode='full')
    lags = np.arange(-len(iner_norm) + 1, len(uav_norm))
    
    # 转换为时间偏移
    time_lags = lags * dt

    # 限制搜索范围
    valid_mask = (time_lags >= -max_delay_seconds) & (time_lags <= max_delay_seconds)
    if not np.any(valid_mask):
         print("警告: 指定的最大时延范围内没有数据点")
         return 0.0, 0.0
         
    valid_lags = time_lags[valid_mask]
    valid_corr = correlation[valid_mask]

    # 找到最大值
    max_idx = np.argmax(valid_corr)
    estimated_delay = valid_lags[max_idx]
    max_corr_val = valid_corr[max_idx]

    print(f"估计时延: {estimated_delay:.4f} s (相关系数: {max_corr_val:.4f})")
    print(f"解释: 如果值为正 (例如 0.1s)，意味着 UAV 信号波形出现在 Inertial 之后 0.1s。")
    print(f"      即 Inertial 的时间戳比 UAV 早。为对齐，需将 Inertial 时间戳 +0.1s。")

    # --- 可视化验证 ---
    # import matplotlib.pyplot as plt
    plt.figure(figsize=(12, 4))
    plt.subplot(1, 2, 1)
    plt.plot(uav_speed, label='UAV Angular Velocity')
    plt.plot(iner_speed, label='Iner Angular Velocity')
    plt.legend()
    plt.title('Angular Velocity Comparison')
    
    plt.subplot(1, 2, 2)
    plt.plot(valid_lags, valid_corr)
    plt.axvline(x=estimated_delay, color='r', linestyle='--', label=f'Delay: {estimated_delay:.4f}s')
    plt.xlabel('Time Lag (s)')
    plt.ylabel('Cross-correlation')
    plt.title('Cross-correlation vs Time Lag')
    plt.legend()
    plt.tight_layout()
    plt.show()

    return estimated_delay, max_corr_val

def solve_hand_eye(uav_list,iner_list,estimated_delay):
    # 插值对齐
    iner_ts = np.array([d['ts'] for d in iner_list])
    iner_rots = R.from_quat([d['rot'].as_quat() for d in iner_list])
    slerp = Slerp(iner_ts, iner_rots)

    R_A, R_B = [], []
    index_list = []
    step = 10 # 增加步长以获得明显的旋转量，提高标定鲁棒性

    for i in range(0, len(uav_list) - step, step):
        t1, t2 = uav_list[i]['ts'], uav_list[i+step]['ts']
        
        if iner_ts[0] <= t1 and t2 <= iner_ts[-1]:
            index_list.append(i)
            
            # R_UAV^W
            R_uav1_in_w = uav_list[i]['rot']
            R_uav2_in_w = uav_list[i+step]['rot']
            
            # R_IMU_W^Body
            R_imu1_in_c = slerp(t1 - estimated_delay)
            R_imu2_in_c = slerp(t2 - estimated_delay)
                      
            # 根据推导: A = R^w_uav2 * R^uav1_w
            # R^uav1_w =  (R_uav1_in_w)-1 
            mat_A = (R_uav2_in_w * R_uav1_in_w.inv()).as_matrix()

            # 根据推导: B = (R^c_body2) * R^body1_c
            #  R^body1_c =  (R_imu1_in_c)-1
            mat_B = (R_imu2_in_c * R_imu1_in_c.inv()).as_matrix()

            R_A.append(mat_A)
            R_B.append(mat_B)
            
           # 提取旋转向量 (Axis * Angle)
            vec_A = R.from_matrix(mat_A).as_rotvec()
            vec_B = R.from_matrix(mat_B).as_rotvec()

            # 计算旋转向量的模长（角度）
            angle_A = np.linalg.norm(vec_A)
            angle_B = np.linalg.norm(vec_B)

            # 计算 A 和 B 旋转轴的夹角余弦值
            # 注意：虽然 A 和 B 在不同坐标系，但 AX=XB 意味着它们描述的是同一个物理旋转
            # 它们的点积在整个序列中应该保持一个相对稳定的值（因为 Rx 是固定的）
            cos_sim = np.dot(vec_A, vec_B) / (angle_A * angle_B)

            print(f"Angle A: {angle_A:.6f}, Angle B: {angle_B:.6f}, Dot: {cos_sim:.6f}")
 
    if len(R_A) < 5:
        print("Insufficient overlapping data.")
        return

    # 构造零平移向量
    zero_T = [np.zeros((3, 1)) for _ in R_A]

        # 验证: 取一组数据看结果是否为常数矩阵
    method_list = [cv2.CALIB_HAND_EYE_TSAI, cv2.CALIB_HAND_EYE_PARK]
                  

    # 使用 Park-Martin 算法求解 AX=XB, X = R^w_c
    rotation_std = []
    results = []
    
    for method in method_list:
        R_x, _ = cv2.calibrateHandEye(R_A, zero_T, R_B, zero_T, method=method)
        print(f"Method: {method}, X:\n", R_x, "R_x det: ", np.linalg.det(R_x))
  
        rotation_angles = []
        for idx in index_list:
            R_uav_in_w = uav_list[idx]['rot'].as_matrix()
            R_body_in_c = slerp(uav_list[idx]['ts']-estimated_delay).as_matrix()
            R_const = np.linalg.inv(R_uav_in_w) @ R_x @ R_body_in_c
            
            trace = np.trace(R_const)
            angle_rad = np.arccos(np.clip((trace - 1) / 2, -1, 1))  # 限制在[-1,1]范围内
            angle_deg = np.degrees(angle_rad)
            rotation_angles.append(angle_deg)
            
        if rotation_angles:
            mean_angle = np.mean(rotation_angles)
            std_angle = np.std(rotation_angles)
            var_angle = np.var(rotation_angles)
            rotation_std.append(std_angle)
            
            print(f"\nRotation Angles Statistics:")
            print(f"Mean: {mean_angle:.4f} degrees")
            print(f"Standard Deviation: {std_angle:.4f} degrees")
            print(f"Variance: {var_angle:.4f} degrees²")
            print(f"Max: {np.max(rotation_angles):.4f} degrees")
            print(f"Min: {np.min(rotation_angles):.4f} degrees")
            
            results.append((method, R_x, std_angle))
    
    # 选择方差最小的方法
    if results:
        best_method_idx = np.argmin(rotation_std)
        best_method, best_R_x, best_std = results[best_method_idx]
        
        print(f"\nBest method: {best_method} with std: {best_std:.4f}")
        print(f"Best rotation matrix:\n{best_R_x}")
        
        # 更新R_x为最佳结果
        R_x = best_R_x
        
    # 保存 XML
    fs = cv2.FileStorage("extrinsic.xml", cv2.FILE_STORAGE_WRITE)
    fs.write("R_imu_to_uav", R_x)
    fs.release()
    
    return R_x

if __name__ == "__main__":
    uav_list = parse_uav_data('../save_data/navigation_log.txt')
    iner_list = parse_iner_data('../save_data/inertial_data/inertial_data.txt')
    estimated_delay, _ = estimate_time_delay(uav_list, iner_list)
    
    #create_rotation_animation(uav_list,iner_list,estimated_delay)
    R_x = solve_hand_eye(uav_list,iner_list,estimated_delay) #R_x : R^uav_imu 或者说imu坐标系在uav坐标系下的姿态
    
    create_rotation_animation(uav_list,iner_list,estimated_delay,R_x)