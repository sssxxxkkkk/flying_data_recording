import numpy as np
import re
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import cv2

def parse_uav_data(file_path):
    """解析 UAV 数据: R_UAV_W (Body to World)"""
    data = []
    with open(file_path, 'r') as f:
        content = f.read()
    ts_pattern = re.compile(r"timestamp:\s+(\d+\.\d+)")
    quat_pattern = re.compile(r"quaternion:\s+([\d\.-]+)\s+([\d\.-]+)\s+([\d\.-]+)\s+([\d\.-]+)")
    sections = content.split("receive quaternion data.")
    for section in sections:
        ts_m = ts_pattern.search(section)
        q_m = quat_pattern.search(section)
        if ts_m and q_m:
            # 假设输入顺序 x y z w
            rot = R.from_quat([float(q_m.group(1)), float(q_m.group(2)), 
                               float(q_m.group(3)), float(q_m.group(4))])
            data.append({'ts': float(ts_m.group(1)), 'rot': rot})
    return data

def parse_iner_data(file_path):
    """解析惯导数据: R_IMU_W_Body (World to Body)"""
    data = []
    with open(file_path, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            if len(parts) < 13: continue
            ts = int(parts[1]) + int(parts[2]) / 1e9
            # 根据 fprintf, parts[9-12] 是 q0-q3. 
            # 假设 q0=w, q1=x, q2=y, q3=z -> 转换为 scipy 的 x,y,z,w
            rot = R.from_quat([float(parts[10]), float(parts[11]), float(parts[12]), float(parts[9])])
            data.append({'ts': ts, 'rot': rot})
    return data

def solve_hand_eye():
    uav_list = parse_uav_data('uav_data.txt')
    iner_list = parse_iner_data('iner_data.txt')

    # 插值对齐
    iner_ts = np.array([d['ts'] for d in iner_list])
    iner_rots = R.from_quat([d['rot'].as_quat() for d in iner_list])
    slerp = Slerp(iner_ts, iner_rots)

    R_A, R_B = [], []
    step = 10 # 增加步长以获得明显的旋转量，提高标定鲁棒性

    for i in range(0, len(uav_list) - step, step):
        t1, t2 = uav_list[i]['ts'], uav_list[i+step]['ts']
        
        if iner_ts[0] <= t1 and t2 <= iner_ts[-1]:
            # R_UAV^W
            R_u1 = uav_list[i]['rot']
            R_u2 = uav_list[i+step]['rot']
            
            # R_IMU_W^Body
            R_i1_inv = slerp(t1) 
            R_i2_inv = slerp(t2)

            # 根据推导: A = (R_u2_W)^-1 * R_u1_W
            # 代表 Body2 到 Body1 的转换 (在 Body2 系下看 Body1)
            mat_A = (R_u2.inv() * R_u1).as_matrix()

            # 根据推导: B = R_i2_Body * (R_i1_Body)^-1
            # 注意: 这里的 R_i 是从 World 到 Body 的旋转
            mat_B = (R_i2_inv * R_i1_inv.inv()).as_matrix()

            R_A.append(mat_A)
            R_B.append(mat_B)

    if len(R_A) < 5:
        print("Insufficient overlapping data.")
        return

    # 构造零平移向量
    zero_T = [np.zeros((3, 1)) for _ in R_A]

    # 使用 Park-Martin 算法求解 AX=XB
    # R_x 是我们求的 X = R_Body_IMU^Body_UAV
    R_x, _ = cv2.calibrateHandEye(R_A, zero_T, R_B, zero_T, method=cv2.CALIB_HAND_EYE_PARK)

    print("Extrinsic Rotation Matrix (IMU Body -> UAV Body):\n", R_x)

    # 验证: 取一组数据看结果是否为常数矩阵
    test_idx = 0
    R_u_test = uav_list[test_idx]['rot'].as_matrix()
    R_i_test = slerp(uav_list[test_idx]['ts']).as_matrix()
    R_const = R_u_test @ R_x @ R_i_test
    print("\nVerification - Constant Matrix (R_world_rel):\n", R_const)

    # 保存 XML
    fs = cv2.FileStorage("extrinsic.xml", cv2.FILE_STORAGE_WRITE)
    fs.write("R_imu_to_uav", R_x)
    fs.write("R_world_offset", R_const)
    fs.release()

if __name__ == "__main__":
    solve_hand_eye()