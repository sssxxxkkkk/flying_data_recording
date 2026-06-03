import re
import cv2
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from scipy.signal import correlate, savgol_filter
from scipy.ndimage import gaussian_filter1d


def is_valid_rotation_matrix(R_mat, tolerance=1e-6):
    """
    Check whether a matrix is a valid rotation matrix.
    """
    R_mat = np.asarray(R_mat, dtype=np.float64)

    if R_mat.shape != (3, 3):
        return False

    orthogonal_error = np.linalg.norm(R_mat.T @ R_mat - np.eye(3))
    determinant_error = abs(np.linalg.det(R_mat) - 1.0)

    return orthogonal_error < tolerance and determinant_error < tolerance


def rotation_angle_deg(R_mat):
    """
    Convert rotation matrix error to angle in degrees.
    """
    return np.degrees(R.from_matrix(R_mat).magnitude())


def parse_uav_data(file_path, output_frame="dji"):
    """
    Parse DJI UAV quaternion data and return body-to-world rotation.

    DJI PSDK quaternion convention:
        q0, q1, q2, q3 = w, x, y, z

    DJI attitude meaning (Confirmed by Hand-Eye Calibration):
        World: NED
        Body : FRD
        Raw quaternion represents Body(FRD) -> World(NED)

    Parameters
    ----------
    file_path : str
        Path to the UAV log file.

    output_frame : str
        "dji":
            Return Body(FRD) -> World(NED)

        "ros":
            Return Body(FLU) -> World(ENU)

    Returns
    -------
    data : list of dict
        {
            "ts": timestamp,
            "rot": scipy Rotation object, body-to-world,
            "R": 3x3 matrix, body-to-world
        }
    """
    if output_frame not in ["dji", "ros"]:
        raise ValueError("output_frame must be 'dji' or 'ros'.")

    data = []

    with open(file_path, "r") as f:
        content = f.read()

    pattern = (
        r"receive quaternion data\.\s*\n"
        r"(timestamp:\s+\d+\.\d+)\s*\n"
        r"(quaternion:\s+"
        r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?\s+"
        r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?\s+"
        r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?\s+"
        r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?\.?)"
    )

    matches = re.findall(pattern, content)

    last_efficient_timestamp = 0.0

    # Body frame conversion: DJI FRD -> ROS FLU
    C_frd_to_flu = np.array([
        [1.0,  0.0,  0.0],
        [0.0, -1.0,  0.0],
        [0.0,  0.0, -1.0]
    ])

    # World frame conversion: ROS ENU -> DJI NED
    C_enu_to_ned = np.array([
        [0.0, 1.0,  0.0],
        [1.0, 0.0,  0.0],
        [0.0, 0.0, -1.0]
    ])

    for i in range(len(matches) - 1):
        first_ts_str, first_quat_str = matches[i]
        second_ts_str, _ = matches[i + 1]

        first_ts_match = re.search(r"timestamp:\s+(\d+\.\d+)", first_ts_str)
        second_ts_match = re.search(r"timestamp:\s+(\d+\.\d+)", second_ts_str)

        if not first_ts_match or not second_ts_match:
            continue

        first_ts = float(first_ts_match.group(1))
        second_ts = float(second_ts_match.group(1))

        time_diff = second_ts - first_ts

        if not (time_diff > 0.0 and time_diff < 0.05):
            continue

        if first_ts <= last_efficient_timestamp:
            continue

        quat_match = re.search(
            r"quaternion:\s+"
            r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s+"
            r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s+"
            r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s+"
            r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\.?",
            first_quat_str
        )

        if not quat_match:
            continue

        # DJI order: w, x, y, z
        q_w = float(quat_match.group(1))
        q_x = float(quat_match.group(2))
        q_y = float(quat_match.group(3))
        q_z = float(quat_match.group(4))

        # SciPy order: x, y, z, w
        q_xyzw = np.array([q_x, q_y, q_z, q_w], dtype=np.float64)

        q_norm = np.linalg.norm(q_xyzw)
        if q_norm <= 1e-12:
            continue

        q_xyzw = q_xyzw / q_norm

        # DJI raw rotation represents Body(FRD) -> World(NED)
        rot_body_to_world_raw = R.from_quat(q_xyzw)
        R_body_to_world_raw = rot_body_to_world_raw.as_matrix()

        if not is_valid_rotation_matrix(R_body_to_world_raw):
            continue

        if output_frame == "dji":
            R_body_to_world = R_body_to_world_raw
            rot_body_to_world = rot_body_to_world_raw
        else:
            # Convert Body(FRD)->World(NED) to Body(FLU)->World(ENU)
            # R_enu_flu = C_enu_ned.T @ R_ned_frd @ C_frd_flu.T
            R_body_to_world = C_enu_to_ned.T @ R_body_to_world_raw @ C_frd_to_flu.T
            rot_body_to_world = R.from_matrix(R_body_to_world)

        if not is_valid_rotation_matrix(R_body_to_world):
            continue

        data.append({
            "ts": first_ts,
            "rot": rot_body_to_world,
            "R": R_body_to_world
        })

        last_efficient_timestamp = first_ts

    return data


def parse_iner_data(file_path):
    """
    Parse inertial data.
    Output: Body(IMU) -> World(IMU world)
    """
    data = []
    previous_ts = 0.0

    with open(file_path, "r") as f:
        for line in f:
            line = line.strip()

            if not line or line.startswith("#"):
                continue

            parts = line.split(",")

            if len(parts) < 13:
                continue

            try:
                ts = int(parts[1]) + int(parts[2]) / 1e9

                if ts < previous_ts:
                    continue

                previous_ts = ts

                quat_w = float(parts[9])
                quat_x = float(parts[10])
                quat_y = float(parts[11])
                quat_z = float(parts[12])

                q_xyzw = np.array([quat_x, quat_y, quat_z, quat_w], dtype=np.float64)

                q_norm = np.linalg.norm(q_xyzw)
                if q_norm <= 1e-12:
                    continue

                q_xyzw = q_xyzw / q_norm

                rot_body_to_world = R.from_quat(q_xyzw)
                R_body_to_world = rot_body_to_world.as_matrix()

                if not is_valid_rotation_matrix(R_body_to_world):
                    continue

                data.append({
                    "ts": ts,
                    "rot": rot_body_to_world,
                    "R": R_body_to_world
                })

            except (ValueError, IndexError):
                continue

    return data


def estimate_time_delay(uav_list, iner_list, max_delay_seconds=1.0, sample_rate_guess=500):
    """
    Estimate time delay between UAV and IMU attitude sequences.
    """
    uav_ts = np.array([d["ts"] for d in uav_list], dtype=np.float64)
    iner_ts = np.array([d["ts"] for d in iner_list], dtype=np.float64)

    uav_rots = R.from_quat([d["rot"].as_quat() for d in uav_list])
    iner_rots = R.from_quat([d["rot"].as_quat() for d in iner_list])

    overlap_start = max(uav_ts[0], iner_ts[0]) + 0.2
    overlap_end = min(uav_ts[-1], iner_ts[-1]) - 0.2

    if overlap_end - overlap_start < 2.0:
        print("Error: overlapping time range is too short.")
        return 0.0, 0.0

    print(f"Analysis time window: {overlap_end - overlap_start:.2f} s")

    dt = 1.0 / sample_rate_guess
    common_time = np.arange(overlap_start, overlap_end, dt)

    uav_slerp = Slerp(uav_ts, uav_rots)
    iner_slerp = Slerp(iner_ts, iner_rots)

    uav_interp = uav_slerp(common_time)
    iner_interp = iner_slerp(common_time)

    def angular_speed_magnitude(rots, time_array):
        speeds = []
        for i in range(1, len(rots) - 1):
            r_prev = rots[i - 1]
            r_next = rots[i + 1]
            r_diff = r_prev.inv() * r_next
            angle = r_diff.magnitude()
            dt_span = time_array[i + 1] - time_array[i - 1]
            speeds.append(angle / dt_span if dt_span > 0 else 0.0)

        if len(speeds) > 0:
            speeds = [speeds[0]] + speeds + [speeds[-1]]
        else:
            speeds = [0.0] * len(rots)
        return np.array(speeds)

    uav_speed = angular_speed_magnitude(uav_interp, common_time)
    iner_speed = angular_speed_magnitude(iner_interp, common_time)

    window_length = min(11, len(uav_speed) - 1)
    if window_length % 2 == 0:
        window_length -= 1

    if window_length >= 5:
        polyorder = min(3, window_length - 1)
        uav_speed_smooth = savgol_filter(uav_speed, window_length, polyorder)
        iner_speed_smooth = savgol_filter(iner_speed, window_length, polyorder)
    else:
        uav_speed_smooth = gaussian_filter1d(uav_speed, sigma=2)
        iner_speed_smooth = gaussian_filter1d(iner_speed, sigma=2)

    def normalize(v):
        return (v - np.mean(v)) / (np.std(v) + 1e-10)

    uav_norm = normalize(uav_speed_smooth)
    iner_norm = normalize(iner_speed_smooth)

    correlation = correlate(uav_norm, iner_norm, mode="full")
    lags = np.arange(-len(iner_norm) + 1, len(uav_norm))
    time_lags = lags * dt

    valid_mask = (time_lags >= -max_delay_seconds) & (time_lags <= max_delay_seconds)

    if not np.any(valid_mask):
        print("Warning: no valid lag in the specified delay range.")
        return 0.0, 0.0

    valid_lags = time_lags[valid_mask]
    valid_corr = correlation[valid_mask]

    max_idx = np.argmax(valid_corr)
    estimated_delay = valid_lags[max_idx]
    max_corr_val = valid_corr[max_idx]

    print(f"Estimated delay: {estimated_delay:.4f} s")
    print(f"Correlation score: {max_corr_val:.4f}")

    plt.figure(figsize=(12, 4))
    plt.subplot(1, 2, 1)
    plt.plot(common_time, uav_speed_smooth, label="UAV angular speed")
    plt.plot(common_time, iner_speed_smooth, label="IMU angular speed")
    plt.xlabel("Time (s)")
    plt.ylabel("Angular speed (rad/s)")
    plt.title("Angular speed comparison")
    plt.legend()

    plt.subplot(1, 2, 2)
    plt.plot(valid_lags, valid_corr)
    plt.axvline(x=estimated_delay, color="r", linestyle="--", label=f"Delay: {estimated_delay:.4f} s")
    plt.xlabel("Time lag (s)")
    plt.ylabel("Cross-correlation")
    plt.title("Cross-correlation")
    plt.legend()
    plt.tight_layout()
    plt.show()

    return estimated_delay, max_corr_val


def solve_hand_eye(uav_list, iner_list, estimated_delay, step=10):
    """
    Solve UAV-IMU rotation extrinsic using OpenCV calibrateHandEye.
    
    Returns
    -------
    dict or None:
        包含外参 R_uav_imu 和 世界系对齐矩阵 R_imuworld_to_uavworld 的字典。
    """
    iner_ts = np.array([d["ts"] for d in iner_list], dtype=np.float64)

    sort_idx = np.argsort(iner_ts)
    iner_ts = iner_ts[sort_idx]
    iner_quats = [iner_list[i]["rot"].as_quat() for i in sort_idx]
    iner_rots = R.from_quat(iner_quats)

    unique_ts, unique_idx = np.unique(iner_ts, return_index=True)
    iner_ts = unique_ts
    iner_rots = R.from_quat(iner_rots.as_quat()[unique_idx])

    if len(iner_ts) < 2:
        print("Insufficient IMU data for interpolation.")
        return None

    slerp = Slerp(iner_ts, iner_rots)
    raw_pairs = []

    for i in range(0, len(uav_list), step):
        t_uav = uav_list[i]["ts"]
        t_imu = t_uav - estimated_delay

        if not (iner_ts[0] <= t_imu <= iner_ts[-1]):
            continue

        R_uav_raw = uav_list[i]["rot"].as_matrix()     # R_world_uav
        R_imu_raw = slerp(t_imu).as_matrix()           # R_imuworld_imu

        raw_pairs.append((R_uav_raw, R_imu_raw, t_uav, t_imu))

    if len(raw_pairs) < 5:
        print("Insufficient overlapping data.")
        return None

    zero_T = [np.zeros((3, 1), dtype=np.float64) for _ in raw_pairs]
    
    best = None
    R_gripper2base = []
    R_target2cam = []
    used_pairs = []

    for R_uav_raw, R_imu_raw, t_uav, t_imu in raw_pairs:
        R_world_uav = R_uav_raw
        R_imu_imuworld = R_imu_raw.T
        R_gripper2base.append(R_world_uav)
        R_target2cam.append(R_imu_imuworld)
        used_pairs.append((R_world_uav, R_imu_imuworld))

    methods = [
        cv2.CALIB_HAND_EYE_PARK,
        cv2.CALIB_HAND_EYE_TSAI,
        cv2.CALIB_HAND_EYE_HORAUD,
    ]
    
    for method in methods:
        try:
            R_x, _ = cv2.calibrateHandEye(
                R_gripper2base, zero_T, R_target2cam, zero_T, method=method
            )
        except cv2.error as e:
            print(f"Method {method} failed:\n{e}")
            continue

        world_align_list = []
        for R_world_uav, R_imu_imuworld in used_pairs:
            R_world_imuworld = R_world_uav @ R_x @ R_imu_imuworld
            world_align_list.append(R_world_imuworld)

        R_mean = R.from_matrix(np.stack(world_align_list)).mean().as_matrix()

        residuals = []
        for R_world_imuworld in world_align_list:
            delta = R_mean.T @ R_world_imuworld
            angle_deg = np.degrees(R.from_matrix(delta).magnitude())
            residuals.append(angle_deg)

        residuals = np.array(residuals)
        mean_err = float(np.mean(residuals))
        std_err = float(np.std(residuals))
        median_err = float(np.median(residuals))
        max_err = float(np.max(residuals))

        score = (median_err, mean_err, std_err)

        if best is None or score < best["score"]:
            best = {
                "score": score,
                "method": method,
                "R_x": R_x,
                "R_mean": R_mean,  # 保存对齐矩阵
                "mean": mean_err,
                "std": std_err,
                "median": median_err,
                "max": max_err,
            }

    if best is None:
        print("Hand-eye calibration failed for all cases.")
        return None

    print("\n==================== BEST RESULT ====================")
    print(f"Best method: {best['method']}")
    print("Best R_x = R^uav_imu:\n", best["R_x"])
    print("Best R_imuworld_to_uavworld:\n", best["R_mean"])
    print(f"Best residual mean  : {best['mean']:.4f} deg")
    print(f"Best residual median: {best['median']:.4f} deg")

    # 保存至文件
    fs = cv2.FileStorage("extrinsic.xml", cv2.FILE_STORAGE_WRITE)
    fs.write("R_imu_to_uav", best["R_x"])
    fs.write("R_imuworld_to_uavworld", best["R_mean"])
    fs.write("method", int(best["method"]))
    fs.write("residual_mean_deg", best["mean"])
    fs.release()

    return {
        "R_uav_imu": best["R_x"],
        "R_imuworld_to_uavworld": best["R_mean"]
    }


def create_rotation_animation(uav_list, iner_list, estimated_delay, calib_result, anim_step=5):
    """
    完善后的三维动画可视化：验证外参解算一致性。
    
    通过动态绘制 IMU 机体系在 UAV 世界坐标系下的两个三维姿态轴线：
    1. 预测轴（红绿蓝细线）：由 UAV 姿态和解算出的外参矩阵计算得到： R_pred = R_world_uav @ R_uav_imu
    2. 测量轴（青黄品粗线）：由 IMU 原始姿态和世界系对齐矩阵计算得到： R_meas = R_imuworld_to_uavworld @ R_imu_raw
    """
    R_uav_imu = calib_result["R_uav_imu"]
    R_imuworld_to_uavworld = calib_result["R_imuworld_to_uavworld"]

    iner_ts = np.array([d["ts"] for d in iner_list], dtype=np.float64)
    sort_idx = np.argsort(iner_ts)
    iner_ts = iner_ts[sort_idx]
    iner_quats = [iner_list[i]["rot"].as_quat() for i in sort_idx]
    iner_rots = R.from_quat(iner_quats)

    unique_ts, unique_idx = np.unique(iner_ts, return_index=True)
    iner_ts = unique_ts
    iner_rots = R.from_quat(iner_rots.as_quat()[unique_idx])

    slerp = Slerp(iner_ts, iner_rots)

    predicted_imu_axes = []
    measured_imu_axes = []
    common_ts = []

    # 引入 anim_step 降采样，防止高频数据导致动画卡死
    for i in range(0, len(uav_list), anim_step):
        uav_data = uav_list[i]
        t_uav = uav_data["ts"]
        t_imu = t_uav - estimated_delay

        if not (iner_ts[0] <= t_imu <= iner_ts[-1]):
            continue

        R_world_uav = uav_data["rot"].as_matrix()
        R_imu_raw = slerp(t_imu).as_matrix()  # Body(IMU) -> World(IMU)

        # 1. 从无人机姿态通过外参预测 IMU 姿态
        R_pred = R_world_uav @ R_uav_imu
        # 2. 从 IMU 姿态通过世界系统一转换为无人机世界下的姿态
        R_meas = R_imuworld_to_uavworld @ R_imu_raw

        predicted_imu_axes.append(R_pred)
        measured_imu_axes.append(R_meas)
        common_ts.append(t_uav)

    if len(predicted_imu_axes) == 0:
        print("No overlapping data for animation.")
        return None

    # 定义标准直角坐标轴基向量
    axis_x = np.array([1.0, 0.0, 0.0])
    axis_y = np.array([0.0, 1.0, 0.0])
    axis_z = np.array([0.0, 0.0, 1.0])

    # 提取序列中每一帧的三维基向量方向
    pred_x = [rot @ axis_x for rot in predicted_imu_axes]
    pred_y = [rot @ axis_y for rot in predicted_imu_axes]
    pred_z = [rot @ axis_z for rot in predicted_imu_axes]

    meas_x = [rot @ axis_x for rot in measured_imu_axes]
    meas_y = [rot @ axis_y for rot in measured_imu_axes]
    meas_z = [rot @ axis_z for rot in measured_imu_axes]

    # 初始化 3D 画布
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    def set_axes_properties():
        ax.set_xlim([-1.2, 1.2])
        ax.set_ylim([-1.2, 1.2])
        ax.set_zlim([-1.2, 1.2])
        ax.set_xlabel("X (World)")
        ax.set_ylabel("Y (World)")
        ax.set_zlabel("Z (World)")
        ax.grid(True)

    # 创建虚假的隐藏线条用以生成标准的图例 Legend
    dummy_pred = ax.plot([0], [0], [0], color="red", linestyle="-", linewidth=1.5, label="Predicted IMU Frame (from UAV + Extrinsic)")[0]
    dummy_meas = ax.plot([0], [0], [0], color="cyan", linestyle="-", linewidth=3, label="Measured IMU Frame (from IMU + World Align)")[0]
    ax.legend(handles=[dummy_pred, dummy_meas], loc="upper left")

    def init():
        set_axes_properties()
        return []

    def update(frame):
        # 彻底清空画布重绘三维向量，保证箭头更新稳定
        ax.clear()
        set_axes_properties()
        
        # 重新补上图例
        ax.legend(handles=[dummy_pred, dummy_meas], loc="upper left")
        ax.set_title(
            f"IMU Frame Consistency Verification [Frame {frame}/{len(predicted_imu_axes)-1}]", 
            y=0, 
            fontsize=12, 
            weight="bold"
        )
        
        # 打印当前帧对应的时间戳
        ax.text2D(0.02, 0.90, f"Time: {common_ts[frame]:.3f} s", transform=ax.transAxes, fontsize=11, weight="bold")

        origin = np.array([0.0, 0.0, 0.0])

        # 1. 绘制预测的 IMU 坐标轴（细线：红、绿、蓝）
        ax.quiver(*origin, *pred_x[frame], color="red", alpha=0.9, arrow_length_ratio=0.12, linewidths=1.5)
        ax.quiver(*origin, *pred_y[frame], color="green", alpha=0.9, arrow_length_ratio=0.12, linewidths=1.5)
        ax.quiver(*origin, *pred_z[frame], color="blue", alpha=0.9, arrow_length_ratio=0.12, linewidths=1.5)

        # 2. 绘制实际测量的 IMU 坐标轴（粗线半透明包裹：青、黄、品红）
        ax.quiver(*origin, *meas_x[frame], color="cyan", alpha=0.5, arrow_length_ratio=0.1, linewidths=4)
        ax.quiver(*origin, *meas_y[frame], color="yellow", alpha=0.5, arrow_length_ratio=0.1, linewidths=4)
        ax.quiver(*origin, *meas_z[frame], color="magenta", alpha=0.5, arrow_length_ratio=0.1, linewidths=4)

        # 动态添加轴向文字标签
        ax.text(*(pred_x[frame]*1.1), "X", color="red", fontsize=10, weight="bold")
        ax.text(*(pred_y[frame]*1.1), "Y", color="green", fontsize=10, weight="bold")
        ax.text(*(pred_z[frame]*1.1), "Z", color="blue", fontsize=10, weight="bold")

        return []

    ani = FuncAnimation(
        fig,
        update,
        frames=len(predicted_imu_axes),
        init_func=init,
        blit=False,
        interval=40,  # 约 25 帧/秒 
        repeat=True
    )

    plt.tight_layout()
    plt.show()
    return ani


if __name__ == "__main__":
    # 解析无人机数据
    uav_list = parse_uav_data(
        "../../save_data/navigation_log.txt",
        output_frame="dji"
    )

    # 解析惯导数据
    iner_list = parse_iner_data(
        "../../save_data/inertial_data/inertial_data.txt"
    )

    # 延迟估计
    estimated_delay, _ = estimate_time_delay(
        uav_list,
        iner_list,
        max_delay_seconds=1.0,
        sample_rate_guess=500
    )

    # 手眼标定解算
    calib_result = solve_hand_eye(
        uav_list,
        iner_list,
        estimated_delay,
        step=10
    )

    # 动画一致性验证
    if calib_result is not None:
        ani = create_rotation_animation(
            uav_list,
            iner_list,
            estimated_delay,
            calib_result,
            anim_step=5  # 每隔 5 帧画一次，保证流畅度
        )
