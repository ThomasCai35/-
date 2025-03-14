import numpy as np
import plotly.graph_objects as go

def dh_parameters():
    # 这里以PUMA560机器人为例设置DH参数
    # 格式：[theta, d, a, alpha]
    dh = np.array([
        [0, 0.660, 0, np.pi / 2],
        [-np.pi / 2, 0, -0.425, 0],
        [0, 0, -0.392, 0],
        [0, 0.1123, 0, np.pi / 2],
        [np.pi / 2, 0, 0, -np.pi / 2],
        [0, 0.093, 0, 0]
    ])
    return dh

def homogeneous_transform(theta, d, a, alpha):
    T = np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])
    return T

def forward_kinematics(dh, thetas):
    T = np.eye(4)
    for i in range(len(dh)):
        T_i = homogeneous_transform(thetas[i], dh[i][1], dh[i][2], dh[i][3])
        T = T @ T_i
    return T

def inverse_kinematics(dh, target_T):
    solutions = []
    # 这里是简化的逆解推导思路，实际完整推导较为复杂
    # 例如根据PUMA机器人的几何结构和运动学关系，通过三角函数运算求解关节角度
    # 计算theta1
    x = target_T[0, 3]
    y = target_T[1, 3]
    theta1 = np.arctan2(y, x)
    # 后续theta2 - theta6的计算类似，根据机器人具体结构和目标位姿逐步推导
    # 这里省略部分复杂计算过程，最终将所有可能的解存入solutions
    # 示例：添加一组可能的解
    solution = [theta1, 0, 0, 0, 0, 0]
    solutions.append(solution)
    return solutions

def cubic_interpolation(start, end, t, total_time):
    t = t / total_time
    q = start + (end - start) * (3 * t ** 2 - 2 * t ** 3)
    q_dot = (end - start) * (6 * t - 6 * t ** 2) / total_time
    q_ddot = (end - start) * (6 - 12 * t) / total_time ** 2
    return q, q_dot, q_ddot

def quintic_interpolation(start, end, t, total_time):
    t = t / total_time
    q = start + (end - start) * (10 * t ** 3 - 15 * t ** 4 + 6 * t ** 5)
    q_dot = (end - start) * (30 * t ** 2 - 60 * t ** 3 + 30 * t ** 4) / total_time
    q_ddot = (end - start) * (60 * t - 180 * t ** 2 + 120 * t ** 3) / total_time ** 2
    return q, q_dot, q_ddot

def trajectory_planning(start_angles, end_angles, total_time, num_points=100):
    time_points = np.linspace(0, total_time, num_points)
    cubic_q = []
    cubic_q_dot = []
    cubic_q_ddot = []
    quintic_q = []
    quintic_q_dot = []
    quintic_q_ddot = []
    dh = dh_parameters()

    for t in time_points:
        cubic_results = [cubic_interpolation(start, end, t, total_time) for start, end in zip(start_angles, end_angles)]
        cubic_q.append([result[0] for result in cubic_results])
        cubic_q_dot.append([result[1] for result in cubic_results])
        cubic_q_ddot.append([result[2] for result in cubic_results])

        quintic_results = [quintic_interpolation(start, end, t, total_time) for start, end in zip(start_angles, end_angles)]
        quintic_q.append([result[0] for result in quintic_results])
        quintic_q_dot.append([result[1] for result in quintic_results])
        quintic_q_ddot.append([result[2] for result in quintic_results])

        # 检查三次插值规划点是否超出工作空间
        cubic_thetas = np.array(cubic_q[-1])
        cubic_T = forward_kinematics(dh, cubic_thetas)
        if not check_workspace(cubic_T):
            print(f"三次插值在时间{t}s时的规划点超出工作空间")

        # 检查五次插值规划点是否超出工作空间
        quintic_thetas = np.array(quintic_q[-1])
        quintic_T = forward_kinematics(dh, quintic_thetas)
        if not check_workspace(quintic_T):
            print(f"五次插值在时间{t}s时的规划点超出工作空间")

    return time_points, cubic_q, cubic_q_dot, cubic_q_ddot, quintic_q, quintic_q_dot, quintic_q_ddot

def check_workspace(T):
    # 以简单的笛卡尔坐标范围判断为例，实际工作空间计算更复杂
    x = T[0, 3]
    y = T[1, 3]
    z = T[2, 3]

    # 假设工作空间边界
    x_min, x_max = -1, 1
    y_min, y_max = -1, 1
    z_min, z_max = 0, 2

    if x < x_min or x > x_max or y < y_min or y > y_max or z < z_min or z > z_max:
        return False
    return True

def plot_curves(time_points, q, q_dot, q_ddot, title):
    num_joints = len(q[0])
    fig = go.Figure()

    for i in range(num_joints):
        fig.add_trace(go.Scatter(x=time_points, y=[q_j[i] for q_j in q], mode='lines', name=f'Joint {i + 1} Position'))
        fig.add_trace(go.Scatter(x=time_points, y=[q_dot_j[i] for q_dot_j in q_dot], mode='lines', name=f'Joint {i + 1} Velocity'))
        fig.add_trace(go.Scatter(x=time_points, y=[q_ddot_j[i] for q_ddot_j in q_ddot], mode='lines', name=f'Joint {i + 1} Acceleration'))

    fig.update_layout(title=title, xaxis_title='Time (s)', yaxis_title='Angle (rad) or Angular Velocity/Acceleration')
    fig.write_html(f'{title.replace(" ", "_")}.html')

def main():
    dh = dh_parameters()
    start_angles = np.array([0, 0, 0, 0, 0, 0])
    end_angles = np.array([np.pi / 2, 0, 0, 0, 0, 0])
    total_time = 5

    time_points, cubic_q, cubic_q_dot, cubic_q_ddot, quintic_q, quintic_q_dot, quintic_q_ddot = \
        trajectory_planning(start_angles, end_angles, total_time)

    plot_curves(time_points, cubic_q, cubic_q_dot, cubic_q_ddot, 'Cubic Interpolation')
    plot_curves(time_points, quintic_q, quintic_q_dot, quintic_q_ddot, 'Quintic Interpolation')

    target_T = forward_kinematics(dh, end_angles)
    print("目标位姿齐次变换矩阵:")
    print(target_T)
    solutions = inverse_kinematics(dh, target_T)
    print("逆解结果:")
    for i, solution in enumerate(solutions):
        print(f"解 {i + 1}: {solution}")

if __name__ == "__main__":
    main()