"""整车（四装甲板）几何模型与装甲板坐标系约定。

坐标系
------
相机系 C : OpenCV 光学系, x 右 / y 下 / z 前, 单位 m。
车体系 V : x 前 / y 左 / z 上, 原点取四块装甲板中心所在平面的中心。
板 系 P : x 观看者右 / y 上 / z 外法线(指向观看者), 右手序, z=0 为板面。

四块板按 k=0..3 依次为 前(+x) / 左(+y) / 后(-x) / 右(-y)。
前/后板中心到车心的纵向距离为 length/2，左/右板为 width/2。
（车体取长方形而不用正方形：正方形会让"整体旋转 90° + 换板编号"成为模型的
 精确对称，偏航不可观；长方形则让四块板的预测位置互不相同，消除该退化。）
"""
import numpy as np

# 装甲板：两灯条外边缘间距、灯条长度（由实拍近正视帧实测比例 2.62:1）
ARMOR_W = 0.27
ARMOR_H = 0.103
# 车体：前后面板中心距 / 左右面板中心距（对角距 = sqrt((L/2)^2+(W/2)^2) ≈ 0.66 m）
VEHICLE_LENGTH = 1.10
VEHICLE_WIDTH = 0.75

PLATE_NAMES = ("front", "left", "back", "right")
PLATE_LABELS = ("前", "左", "后", "右")
PLATE_NORMALS = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


def plate_offsets(length=VEHICLE_LENGTH, width=VEHICLE_WIDTH):
    return np.array([
        [length * 0.5, 0.0, 0.0],
        [0.0, width * 0.5, 0.0],
        [-length * 0.5, 0.0, 0.0],
        [0.0, -width * 0.5, 0.0],
    ])


def plate_rt(k, length=VEHICLE_LENGTH, width=VEHICLE_WIDTH):
    """返回 (R_vp, p_v)：板 k 相对车体系的姿态与位置。"""
    n = PLATE_NORMALS[k]
    p_v = plate_offsets(length, width)[k]
    y_p = np.array([0.0, 0.0, 1.0])       # 板 y 轴 = 车上方向
    z_p = n                               # 板 z 轴 = 外法线
    x_p = np.cross(y_p, z_p)              # 右手序
    R_vp = np.column_stack([x_p, y_p, z_p])
    return R_vp, p_v


def plate_yaw_offset(k):
    """板法线在车体系中的偏航角（rad）：前/左/后/右 = 0/90/180/-90 度。"""
    n = PLATE_NORMALS[k]
    return float(np.arctan2(n[1], n[0]))


def armor_object_points(w=ARMOR_W, h=ARMOR_H):
    """装甲板物方角点，顺序：左上、左下、右下、右上。"""
    return np.array([
        [-w / 2.0, h / 2.0, 0.0],
        [-w / 2.0, -h / 2.0, 0.0],
        [w / 2.0, -h / 2.0, 0.0],
        [w / 2.0, h / 2.0, 0.0],
    ], dtype=np.float64)


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def plate_pose_in_world(k, p_vw, yaw, length=VEHICLE_LENGTH, width=VEHICLE_WIDTH):
    R_vp, p_v = plate_rt(k, length, width)
    R = rot_z(yaw)
    return R @ R_vp, np.asarray(p_vw) + R @ p_v


def vehicle_plate_corners(k, p_vw, yaw, length=VEHICLE_LENGTH, width=VEHICLE_WIDTH,
                          w=ARMOR_W, h=ARMOR_H):
    R_wp, p_wp = plate_pose_in_world(k, p_vw, yaw, length, width)
    pts = armor_object_points(w, h) @ R_wp.T + p_wp.reshape(1, 3)
    return pts


def chassis_box(length=VEHICLE_LENGTH, width=VEHICLE_WIDTH, half_h=0.11):
    """车体包围盒顶点与 12 条棱（车体系），用于可视化。"""
    a, b = length * 0.5, width * 0.5
    pts = np.array([[x, y, z] for x in (-a, a) for y in (-b, b) for z in (-half_h, half_h)])
    edges = [(i, j) for i in range(8) for j in range(i + 1, 8)
             if bin(i ^ j).count("1") == 1]
    return pts, edges
