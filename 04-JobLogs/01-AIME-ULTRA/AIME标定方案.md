1. # 标定参数和坐标系定义
2. 外参：各传感器相对于机器人坐标系的位置（x,y,z）和姿态(roll, pitch, yaw)。包括前后向的3dtof，RGB相机，轮速计，IMU等传感器。
3. 内参：各传感器自身参数。RGB相机内参（fx, fy, cx, cy, k1, k2, k3,p1 p2），点tof内参(xtalk_pos, xtalk_maxratio, xtalk_peak, offset)，轮速计内参（左右轮半径，轮距）。
4. 外参精度需求：位置精度（< 2mm），角度精度（< 1 deg）
5. 机器人坐标系：两驱动轮轴线**中点**在水平地面上的投影为坐标系原点O, 坐标系方向为前-X，左-Y，上-Z。水平面为XOY平面，Z轴垂直向上。
6. # 各传感器标定方案

强烈建议要求传感器供应商（相机、TOF、IMU）在出厂前完成内参标定并将数据写入传感器Flash中。产线标定**只做外参标定**及**轮速计参数修正** 。

1. ## 3D TOF标定方案

目的：标定前3D TOF+RGB组成的RGBD模组的外参，后3D TOF的外参。

**方案1： 实验室方案**。

![](https://tcl-eaglelab.feishu.cn/space/api/box/stream/download/asynccode/?code=Zjg5YmJjNmExZjg0YjJiZjRlZWRjYzM5OGE5ZGQyM2FfdnlhRnVsSFlOSzZNTEhvRUU2OFIxdGY1YW5ncGNkQ2dfVG9rZW46TjBlU2JOODA3b2lBRDl4VGlGcmNkU0lLbkFmXzE3ODE2NjA0NjI6MTc4MTY2NDA2Ml9WNA&add_watermark=true&scene_type=CCM)

**方案2: 量产标定方案。**需要主动发光标定板，标定滑轨支架和转台。

![](https://tcl-eaglelab.feishu.cn/space/api/box/stream/download/asynccode/?code=YjRmMDA4ODJlZDQwYjEwNjljZjI2M2UzNTAxMjYwNzVfS2dsU1J2T0xCbml4cGxlb2V6WkdEVXVDN3I5YjZFZHBfVG9rZW46RXNzVGJxSXdjb1lmRW94QmRqeGMycVBTbmtmXzE3ODE2NjA0NjI6MTc4MTY2NDA2Ml9WNA&add_watermark=true&scene_type=CCM)

![](https://tcl-eaglelab.feishu.cn/space/api/box/stream/download/asynccode/?code=OWU5MmJhMzA1NjFkNDc2NTgyZGRkMGNmNGY2MWM3MjlfbmJxV3hScnFBTWtkSzlJQjZuWHNlOVdPSmdQblN1UHVfVG9rZW46WXVheWJrc1dkb2FWYmR4aUhUY2NWOTkyblYzXzE3ODE2NjA0NjI6MTc4MTY2NDA2Ml9WNA&add_watermark=true&scene_type=CCM)

2. ## 点TOF标定

目的：标定各个点tof的Xtalk和offset。

需求：一体机硬件给出标定需求，距离，色卡和测试范围。两组远近多面体，配合一个气缸

![](https://tcl-eaglelab.feishu.cn/space/api/box/stream/download/asynccode/?code=MTM3NmRlMDlmMDBjZmRhYWM1OTEzZDliMTNhZDU5MWJfbk5OUjQ1M1VFUEg1Qlh5c3NXU2t1UDFuY0ZFRmtWVm1fVG9rZW46RGlYamJxVkc0bzUxVUJ4QVI2YmN2Q051bkhmXzE3ODE2NjA0NjI6MTc4MTY2NDA2Ml9WNA&add_watermark=true&scene_type=CCM)

![](https://tcl-eaglelab.feishu.cn/space/api/box/stream/download/asynccode/?code=MmI3YjI4YjM4ODUzZTUwYzVmNWViYTU0MzRmZWNjZTNfazVma212YUxseGd6M2dqT0N2NTZpRzM5ZHdYZDZseG1fVG9rZW46V25OMGJVY29Mb0c5akN4U0xXc2NlMWpnbkhkXzE3ODE2NjA0NjI6MTc4MTY2NDA2Ml9WNA&add_watermark=true&scene_type=CCM)

![](https://tcl-eaglelab.feishu.cn/space/api/box/stream/download/asynccode/?code=YWQ4ZTk3ZmE5MTk0MjBkYjY2YTNjMzViMTNiNDRmMTRfYnZvVXFCcDRYdWZQZHJMNTRmOTZ5SkZzZnRTazJ5OXBfVG9rZW46V2tMZWJlRDA2b2t4aVB4TzVtNGNCbFQ2bndlXzE3ODE2NjA0NjI6MTc4MTY2NDA2Ml9WNA&add_watermark=true&scene_type=CCM)

![](https://tcl-eaglelab.feishu.cn/space/api/box/stream/download/asynccode/?code=MjBhZDMwMGIwMThmMDBmN2FjZWVlNzY3NzcyZTcwMmRfYWJNWUoyb3NhODhxZ20ydkIzWnFIN3dISGNTNXozYVhfVG9rZW46VzBlamJST0c0b2lGMTd4anphT2NpbnhMbnliXzE3ODE2NjA0NjI6MTc4MTY2NDA2Ml9WNA&add_watermark=true&scene_type=CCM)

![](https://tcl-eaglelab.feishu.cn/space/api/box/stream/download/asynccode/?code=NDEwNDJjYThlNDRmYzE5YjkxMjJkMjlhMzJhNzM3ZGZfWVRaZmZRUjR3Q3dpZ2Y0dEQzRGNQcnlzWTU3Vk5uNVVfVG9rZW46RVA1VmIyY0N1b0o3TzZ4dHphbmNOM1M5bld3XzE3ODE2NjA0NjI6MTc4MTY2NDA2Ml9WNA&add_watermark=true&scene_type=CCM)

3. ## 轮毂标定

建立一个专用的标定间，内部布置已知几何位置的高精度标靶。机器人驶入后，自动执行一段预设的动作序列，采集数据，后台解算标定参数。

1. **搭建标定间：**地面极度平整（水磨石或环氧地坪），摩擦系数恒定，无打滑。地面**设计机械限位槽**或者贴高精度二维码（位置误差小于1mm）**。**标定间长度大于3m，宽度大于2米，高度大于2.5米。标定场地顶部安装相机俯视拍摄。墙面安装标定板。**墙角垂直，或有直角形状标定参考物。**标定场地面需有高精度距离测量能力（需要测量地面标定板准确位置，需要标尺/水平仪/激光测距仪等设备）

![](https://tcl-eaglelab.feishu.cn/space/api/box/stream/download/asynccode/?code=YWU1NTU1MWNlOTdkNDAwMmU1NWVjMTZiNWQ5ZWUwMThfSk1oa2ZwRjdkTnhXMkFFZ1JROTloQ2Y1V1VTc2xRWW5fVG9rZW46WW5wUGJqRmVIb0RtRWV4MG8yQ2NtNmhxblJmXzE3ODE2NjA0NjI6MTc4MTY2NDA2Ml9WNA&add_watermark=true&scene_type=CCM)

2. **底部夹具和顶部夹具：**配合机械限位或者高精度的定位，可以固定机器人底盘位置并便于测量夹具到标定间其余位置的距离。例如图所示，固定底座夹具到墙角2m, 作为几何结构约束**标定3dtof或线激光雷达**。

![](https://tcl-eaglelab.feishu.cn/space/api/box/stream/download/asynccode/?code=YWEyZTMyODkyZGY5NjMxNGY1MjQ0NGU0MzMxNGNjYWVfaFRrUlR4Y1U1WnNzSlU0ZWdBQTNyYjZsV1ExNUxDamVfVG9rZW46S3FEaGJJR0Ztb1Y0M0N4MDlSeWMzUktsbmZiXzE3ODE2NjA0NjI6MTc4MTY2NDA2Ml9WNA&add_watermark=true&scene_type=CCM)

暂时无法在飞书文档外展示此内容

3. **光照环境**：恒定光源，标定站需遮光，内部使用无频闪的LED灯带，保证光照均匀，避免阴影和过曝，确保相机和TOF稳定工作。
4. **apriltag和支架：**apriltag通过支架水平安装在机器人顶部，保证机器人顶端存在固定位置的平面，可用于相机观测轨迹groundtruth。apriltag size > 10cm。AprilTag 可以解决一般棋盘格的旋转二义性，只要能识别出 ID，角点顺序就是物理固定的。
5. **真值相机：**把相机当做固定观测台，可以得到机器人在地面坐标系的位置。全局快门，30-60fsp, 2k-4k, 镜头畸变较小（大于2448*2048, 2/3“, MV-SC050-10UC).

---

# 备选

夹具固定机器人在标定间的位置，使3d tof以flat模式朝向墙角或标定标志物等已知几何形状的对象，距离50cm。此时已知机器人在标定间中的位姿，已知传感器坐标系下标志物点云。建立机器人坐标系下的点云模板，可通过点云配准计算3dtof的外参。

![](https://tcl-eaglelab.feishu.cn/space/api/box/stream/download/asynccode/?code=NzJmYjBmOTcxOWZiMjg4MTlkY2ZlZTAwMDJmM2U0N2FfTVpkQXRaU0xja1VNemNhVlRzT04yekpoaVJmUVNmdHhfVG9rZW46UDJ0R2I3VTJyb0ZxRWp4MTVDNmNPaUpSbkFCXzE3ODE2NjA0NjI6MTc4MTY2NDA2Ml9WNA&add_watermark=true&scene_type=CCM)
