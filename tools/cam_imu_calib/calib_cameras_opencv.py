#!/usr/bin/env python3
"""
智能相机标定脚本 - 自动过滤低质量图像
功能：
  ✅ 读取文件夹所有图像（支持 jpg/png/bmp）
  ✅ 检测棋盘格角点 + 亚像素优化
  ✅ 智能过滤：模糊度检测 + 角点分布质量
  ✅ 标定 + 重投影误差分析
  ✅ 保存标定参数 + 可视化报告
  ✅ 生成标定质量报告（JSON）
"""
import os
import cv2
import numpy as np
import glob
import json
from pathlib import Path
import argparse
from datetime import datetime

def is_blurry(gray_img, threshold=300.0):
    """Laplacian 方差检测模糊度（值越小越模糊）"""
    laplacian_var = cv2.Laplacian(gray_img, cv2.CV_64F).var()
    return laplacian_var < threshold, laplacian_var

def check_corner_distribution(corners, img_shape, min_coverage=0.2):
    """
    检查角点是否覆盖图像关键区域（避免集中在中心）
    :param min_coverage: 角点需覆盖图像宽度/高度的最小比例
    """
    xs = corners[:, 0, 0]
    ys = corners[:, 0, 1]
    x_span = (np.max(xs) - np.min(xs)) / img_shape[1]
    y_span = (np.max(ys) - np.min(ys)) / img_shape[0]
    return (x_span >= min_coverage) and (y_span >= min_coverage), (x_span, y_span)

def calibrate_camera(image_dir, pattern_size=(9, 6), square_size=1.0, 
                     blur_threshold=80.0, min_valid_images=10, output_dir="calibration_results"):
    """
    主标定流程
    :param image_dir: 图像文件夹路径
    :param pattern_size: 棋盘格内角点 (columns, rows) 例如 (9,6) 表示9列6行角点
    :param square_size: 方格物理尺寸（单位：毫米），用于缩放内参
    :param blur_threshold: 模糊度阈值（Laplacian方差），低于此值视为模糊
    :param min_valid_images: 最小有效图像数量
    :param output_dir: 结果输出目录
    """
    # ============= 1. 准备物点坐标（3D点） =============
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size  # 应用物理尺寸
    
    objpoints = []  # 3D点
    imgpoints = []  # 2D点
    valid_images = []  # 有效图像路径
    rejected_info = []  # 记录被拒绝原因
    
    # ============= 2. 读取并筛选图像 =============
    image_paths = sorted(glob.glob(os.path.join(image_dir, "*.[jJ][pP][gG]")) + 
                         glob.glob(os.path.join(image_dir, "*.[pP][nN][gG]")) +
                         glob.glob(os.path.join(image_dir, "*.[bB][mM][pP]")))
    
    if not image_paths:
        raise ValueError(f"❌ 未在 {image_dir} 中找到图像文件！支持格式: jpg, png, bmp")
    
    print(f"🔍 扫描到 {len(image_paths)} 张图像，开始筛选...\n")
    
    # 角点检测参数（亚像素优化）
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # 创建窗口用于显示角点检测结果
    cv2.namedWindow('Detected Corners', cv2.WINDOW_AUTOSIZE)
    
    for idx, img_path in enumerate(image_paths, 1):
        img = cv2.imread(img_path)
        if img is None:
            rejected_info.append({"file": Path(img_path).name, "reason": "无法读取图像"})
            continue
            
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape[:2]
        
        # 检查模糊度
        blurry, lap_var = is_blurry(gray, blur_threshold)
        if blurry:
            print({
                "file": Path(img_path).name, 
                "reason": f"图像模糊 (Laplacian方差={lap_var:.1f} < {blur_threshold})"
            })
            continue
        
        # 检测角点
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        if not ret:
            print({"file": Path(img_path).name, "reason": "未检测到完整棋盘格"})
            continue
        
        # 亚像素优化
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        # 保存有效数据
        objpoints.append(objp)
        imgpoints.append(corners_refined)
        valid_images.append(img_path)
        
        # 可视化角点（调试用）
        img_with_corners = img.copy()
        cv2.drawChessboardCorners(img_with_corners, pattern_size, corners_refined, ret)
        
        # 显示带有角点的图像
        cv2.imshow('Detected Corners', img_with_corners)
        key = cv2.waitKey(500)  # 显示500毫秒
        if key == 27:  # ESC键退出
            break
    
    cv2.destroyAllWindows()  # 处理完所有图像后关闭窗口
    
    # ============= 3. 验证有效图像数量 =============
    if len(valid_images) < min_valid_images:
        print(f"❌ 有效图像不足！至少需要 {min_valid_images} 张（当前: {len(valid_images)}）")
        if rejected_info:
            print("\n🚫 被拒绝图像示例:")
            for item in rejected_info[:5]:
                print(f"   • {item['file']}: {item['reason']}")
        cv2.destroyAllWindows()  # 即使失败也要关闭窗口
        return False
    
    # ============= 4. 执行标定 =============
    print("\n⚙️  开始相机标定...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, (w, h), None, None,flags=cv2.CALIB_FIX_K3
    )
    
    # 计算重投影误差（每张图）
    reprojection_errors = []
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        reprojection_errors.append(float(error))
    
    mean_error = np.mean(reprojection_errors)
    max_error = np.max(reprojection_errors)
    
    # ============= 5. 保存结果 =============
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    
    # 保存标定参数（YAML格式，ROS友好）
    calib_data = {
        "camera_matrix": mtx.tolist(),
        "distortion_coefficients": dist.flatten().tolist(),
        "image_width": w,
        "image_height": h,
        "pattern_size": {"cols": pattern_size[0], "rows": pattern_size[1]},
        "square_size_mm": square_size,
        "reprojection_error_mean_px": float(mean_error),
        "reprojection_error_max_px": float(max_error),
        "valid_image_count": len(valid_images),
        "calibration_date": datetime.now().isoformat()
    }
    
    # 保存为YAML（OpenCV标准格式）
    fs = cv2.FileStorage(f"{output_dir}/camera_calibration.yaml", cv2.FILE_STORAGE_WRITE)
    fs.write("camera_matrix", mtx)
    fs.write("distortion_coefficients", dist)
    fs.write("image_width", w)
    fs.write("image_height", h)
    fs.write("reprojection_error_mean", mean_error)
    fs.release()
    
    # 保存JSON报告（人类可读）
    with open(f"{output_dir}/calibration_report.json", 'w') as f:
        json.dump(calib_data, f, indent=2)
    
    # ============= 6. 可视化质量报告 =============
    # 保存带角点的示例图像
    for i, img_path in enumerate(valid_images[:3]):  # 保存前3张
        img = cv2.imread(img_path)
        cv2.drawChessboardCorners(img, pattern_size, imgpoints[i], True)
        cv2.putText(img, f"Reproj Err: {reprojection_errors[i]:.2f}px", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imwrite(f"{output_dir}/valid_sample_{i+1}.jpg", img)
    
    # 生成筛选报告
    report = f"""
╔════════════════════════════════════════════════════════════╗
║              📷 相机标定质量报告                           ║
╠════════════════════════════════════════════════════════════╣
║ 有效图像数量      : {len(valid_images):3d} / {len(image_paths):3d}                         
║ 平均重投影误差    : {mean_error:6.3f} 像素 (优秀: <0.5)              
║ 最大重投影误差    : {max_error:6.3f} 像素                          
║ 内参矩阵 (fx,fy)  : ({mtx[0,0]:.1f}, {mtx[1,1]:.1f})                   
║ 畸变系数 (k1,k2)  : ({dist[0,0]:.4f}, {dist[0,1]:.4f})                 
╠════════════════════════════════════════════════════════════╣
║ ✅ 标定成功！结果已保存至: {output_dir}              
║    • camera_calibration.yaml (OpenCV/ROS 标准格式)        
║    • calibration_report.json (详细参数)                   
║    • valid_sample_*.jpg (有效图像示例)                    
╚════════════════════════════════════════════════════════════╝
"""
    print(report)
    
    # 保存筛选日志
    if rejected_info:
        with open(f"{output_dir}/rejected_images.log", 'w') as f:
            f.write(f"共拒绝 {len(rejected_info)} 张图像:\n")
            for item in rejected_info:
                f.write(f"{item['file']}: {item['reason']}\n")
        print(f"📝 详细拒绝原因已保存至: {output_dir}/rejected_images.log")
    
    return True

# ============= 主程序 =============
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="智能相机标定工具 - 自动过滤低质量图像")
    parser.add_argument("--image_dir", type=str,default="../../save_data/image_data/", help="标定图像文件夹路径")
    parser.add_argument("--cols", type=int, default=6, help="棋盘格内角点列数 (默认: 9)")
    parser.add_argument("--rows", type=int, default=7, help="棋盘格内角点行数 (默认: 6)")
    parser.add_argument("--square_size", type=float, default=50.0, help="方格物理尺寸 (mm, 默认: 25.0)")
    parser.add_argument("--blur_thresh", type=float, default=80.0, help="模糊度阈值 (Laplacian方差, 默认: 80.0)")
    parser.add_argument("--min_images", type=int, default=10, help="最小有效图像数量 (默认: 10)")
    parser.add_argument("--output", type=str, default="calibration_results", help="输出目录 (默认: calibration_results)")
    
    args = parser.parse_args()
    
    success = calibrate_camera(
        image_dir=args.image_dir,
        pattern_size=(args.cols, args.rows),
        square_size=args.square_size,
        blur_threshold=args.blur_thresh,
        min_valid_images=args.min_images,
        output_dir=args.output
    )
    
    exit(0 if success else 1)
