#!/usr/bin/env python3
"""
计算扫描轨迹的 IK 解
基于 DLS (Damped Least Squares) 数值逆运动学
参考 README.md 第 3 节的算法
"""

import numpy as np
import yaml
from pathlib import Path

# CR12A DH 参数 (根据官方 URDF)
# 注意：这里使用标准 DH 参数，需要根据实际 URDF 调整
class CR12A_Kinematics:
    def __init__(self):
        # 关节限位 (弧度)
        self.joint_limits = np.array([
            [-np.pi, np.pi],      # joint1
            [-np.pi, np.pi],      # joint2
            [-2.79, 2.79],        # joint3
            [-np.pi, np.pi],      # joint4
            [-np.pi, np.pi],      # joint5
            [-np.pi, np.pi],      # joint6
        ])

        # DH 参数 [a, alpha, d, theta_offset]
        # 需要根据 urdf/cr12_macro.xacro 精确提取
        # 这里使用近似值，实际应从 URDF 解析
        self.dh_params = [
            [0,      np.pi/2,  0.1785, 0],      # joint1
            [0.425,  0,        0,      -np.pi/2], # joint2
            [0.39225, 0,       0,      0],       # joint3
            [0,      np.pi/2,  0.10915, 0],     # joint4
            [0,      -np.pi/2, 0.09465, 0],     # joint5
            [0,      0,        0.0823,  0],     # joint6
        ]

    def dh_transform(self, a, alpha, d, theta):
        """标准 DH 变换矩阵"""
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),                 np.cos(alpha),                d],
            [0,              0,                             0,                            1]
        ])

    def forward_kinematics(self, q):
        """
        正运动学：关节角 -> tool0 位姿
        q: 6x1 关节角数组 (弧度)
        返回: 4x4 齐次变换矩阵
        """
        T = np.eye(4)
        for i, (a, alpha, d, offset) in enumerate(self.dh_params):
            theta = q[i] + offset
            T = T @ self.dh_transform(a, alpha, d, theta)
        return T

    def jacobian(self, q, delta=1e-6):
        """
        数值雅可比矩阵 (6x6)
        前3行：位置对关节角的偏导
        后3行：姿态(欧拉角)对关节角的偏导
        """
        J = np.zeros((6, 6))
        T0 = self.forward_kinematics(q)
        p0 = T0[:3, 3]

        for i in range(6):
            q_plus = q.copy()
            q_plus[i] += delta
            T_plus = self.forward_kinematics(q_plus)
            p_plus = T_plus[:3, 3]

            # 位置雅可比
            J[:3, i] = (p_plus - p0) / delta

            # 姿态雅可比 (简化：只考虑 Z 轴方向)
            z0 = T0[:3, 2]
            z_plus = T_plus[:3, 2]
            J[3:, i] = (z_plus - z0) / delta

        return J

    def ik_dls(self, target_pos, target_z_axis, q_init, max_iter=100, tol=1e-4, lambda_=0.01):
        """
        DLS 数值逆运动学
        target_pos: 目标位置 (3,)
        target_z_axis: 目标 Z 轴方向 (3,) - 相机光轴
        q_init: 初始关节角 (6,)
        """
        q = q_init.copy()

        for iteration in range(max_iter):
            T = self.forward_kinematics(q)
            current_pos = T[:3, 3]
            current_z = T[:3, 2]

            # 误差向量 (6维：3位置 + 3姿态)
            pos_error = target_pos - current_pos
            z_error = target_z_axis - current_z
            error = np.concatenate([pos_error, z_error])

            # 检查收敛
            if np.linalg.norm(error) < tol:
                return q, True, iteration

            # DLS 更新
            J = self.jacobian(q)
            JT = J.T
            dq = JT @ np.linalg.inv(J @ JT + lambda_**2 * np.eye(6)) @ error

            q = q + dq

            # 关节限位约束
            q = np.clip(q, self.joint_limits[:, 0], self.joint_limits[:, 1])

        return q, False, max_iter


def load_camera_calibration():
    """从 calibration.yaml 读取相机在法兰坐标系下的姿态"""
    calib_file = Path(__file__).parent.parent / "config" / "calibration.yaml"
    with open(calib_file) as f:
        calib = yaml.safe_load(f)

    # 提取相机到法兰的变换
    # 根据 README: 相机光轴(+Z)在法兰系里指向 +X
    # 这意味着相机坐标系相对法兰旋转了 90 度
    # T_flange_to_camera 的旋转部分
    R_flange_to_camera = np.array([
        [0, 0, 1],   # 相机 X = 法兰 Z
        [-1, 0, 0],  # 相机 Y = 法兰 -X
        [0, -1, 0]   # 相机 Z = 法兰 -Y
    ])

    return R_flange_to_camera


def compute_scan_trajectory():
    """计算完整扫描轨迹"""
    print("=== 开始计算扫描轨迹 IK 解 ===\n")

    # 加载航点
    waypoints_data = np.load('/tmp/scan_waypoints.npz')
    waypoints = waypoints_data['waypoints']
    print(f"✅ 加载了 {len(waypoints)} 个扫描航点\n")

    # 初始化运动学求解器
    robot = CR12A_Kinematics()
    R_flange_to_camera = load_camera_calibration()

    # 相机垂直向下的目标姿态 (世界坐标系)
    # 相机 Z 轴 = 世界 -Z 轴 (向下)
    camera_z_target = np.array([0, 0, -1])

    # 初始猜测: 零位
    q_current = np.zeros(6)

    trajectory = []
    trajectory.append(q_current.copy())  # P1: 零位起点

    for i, waypoint in enumerate(waypoints, 2):
        print(f"--- 计算 P{i} ---")
        target_pos = waypoint

        # 由于相机光轴在法兰系指向 +X，
        # 要让相机 Z 垂直向下，法兰 X 轴应该垂直向下
        # 即法兰姿态: X_flange = -Z_world
        flange_x_target = np.array([0, 0, -1])

        # 调用 IK 求解
        q_solution, success, iters = robot.ik_dls(
            target_pos=target_pos,
            target_z_axis=flange_x_target,
            q_init=q_current,
            max_iter=200,
            tol=1e-4
        )

        if success:
            # 验证解
            T = robot.forward_kinematics(q_solution)
            pos = T[:3, 3]
            error = np.linalg.norm(pos - target_pos)
            print(f"  ✅ 收敛于 {iters} 次迭代，位置误差: {error*1000:.2f} mm")
            print(f"  关节角 (度): {np.degrees(q_solution)}")

            trajectory.append(q_solution)
            q_current = q_solution  # 热启动下一个航点
        else:
            print(f"  ❌ IK 求解失败！")
            return None

    trajectory.append(np.zeros(6))  # P10: 零位终点

    return np.array(trajectory)


def save_trajectory_yaml(trajectory, output_file):
    """保存轨迹到 YAML 格式"""
    points = []

    # 时间安排：P1=0s, P2-P9每步1s, P10=13s
    times = [0, 3, 4, 5, 6, 7, 8, 9, 10, 13]

    for i, (q, t) in enumerate(zip(trajectory, times)):
        point = {
            'positions': q.tolist(),
            'velocities': [0.0] * 6,
            'accelerations': [0.0] * 6,
            'effort': [0.0] * 6,
            'time_from_start': {
                'sec': t,
                'nanosec': 0
            }
        }
        points.append(point)

    traj_data = {
        'header': {'stamp': {'sec': 0, 'nanosec': 0}, 'frame_id': ''},
        'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
        'points': points
    }

    with open(output_file, 'w') as f:
        yaml.dump(traj_data, f, default_flow_style=False, sort_keys=False)

    print(f"\n✅ 轨迹已保存到: {output_file}")


if __name__ == '__main__':
    trajectory = compute_scan_trajectory()

    if trajectory is not None:
        output_file = Path(__file__).parent.parent / "config" / "scan_traj_new.yaml"
        save_trajectory_yaml(trajectory, output_file)
        print("\n🎉 扫描轨迹计算完成！")
    else:
        print("\n❌ 扫描轨迹计算失败")
        exit(1)
