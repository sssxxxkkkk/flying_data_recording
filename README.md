# 项目文档

## 驱动源码位置

- **事件相机驱动**（含录像与图像传输功能）  
  路径：`event_cam_driver/DvsenseSyncViewerSample`

- **事件数据文件读取器**  
  路径：`event_cam_driver/DvsenseFileReaderSample`

- **惯性传感器驱动**  
  路径：`inertial_driver`

- **红外传感器驱动**  
  红外相机驱动的还待完善。

---

## 远程调试支持

启动 VNC 服务器以支持远程调试：
```
启动命令 sh ~/start_vnc.sh 
```

远程服务器可以通过以下指令接收事件相机推送的图片（图传功能）：
```
python /launch/receiver.py 
```
## 数据录制
- **录制事件相机数据**  
```
./launch/dvsense_recorder or ./launch/dvsense_recorder dest_ip dest_port（dest_ip dest_port分别为上位机的ip和端口号，用于图传）
```
- **录制惯导数据**  
```
./launch/yesense_main your_file_name.txt （请替换 your_file_name.txt 为实际文件名）
```

- **录制红外相机数据**  
```
./launch/** （待完善）
```

- **订阅无人机姿态信息**  
```
./launch/dji_sdk_demo_linux_cxx 
```

- **一键启动所有模块**  
```
chmod +x ./launch/launch_all.sh
sh ./launch/launch_all.sh
```

## 数据保存与回放
- **回放事件数据**  
```
./launch/dvsense_file_reader your_file_name.raw （请替换 your_file_name.raw 为实际文件名）
```
- **已保存数据目录**  
```
事件数据：./saved_data/event_data
图像数据：./saved_data/image_data
红外数据：./saved_data/infrared_data
惯性数据：./saved_data/inertial_data
无人机数据：./saved_data/drone_data
```
