#!/usr/bin/env python3
"""
シンプルな3Dアニメーション
発散前の短時間の振り子運動を可視化
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from fixed_robot_3d import FixedRobot3D
import pinocchio as pin

def create_simple_animation():
    """短時間の振り子運動アニメーション"""
    print("=== シンプル3Dアニメーション ===")
    
    # ロボット初期化
    robot = FixedRobot3D()
    
    # 初期状態（少し不安定な姿勢）
    initial_state = [
        0.0, 0.0, 0.0,    # x, y, yaw
        0.3, -0.8,        # 左脚
        0.5, -1.0         # 右脚
    ]
    
    # 短時間シミュレーション（発散前）
    results = robot.simulate(initial_state, T_sim=0.5, dt=0.01)
    
    # アニメーション作成
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 軸設定
    ax.set_xlim([-0.5, 0.5])
    ax.set_ylim([-0.5, 0.5])
    ax.set_zlim([0, 0.5])
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    
    # 地面
    x_ground = np.linspace(-0.5, 0.5, 5)
    y_ground = np.linspace(-0.5, 0.5, 5)
    X, Y = np.meshgrid(x_ground, y_ground)
    Z = np.zeros_like(X)
    ax.plot_surface(X, Y, Z, alpha=0.3, color='gray')
    
    # ロボット構造の線
    lines = {
        'base': ax.plot([], [], [], 'k-', linewidth=4)[0],
        'leg_L': ax.plot([], [], [], 'b-', linewidth=3)[0],
        'leg_R': ax.plot([], [], [], 'r-', linewidth=3)[0],
        'wheel_L': ax.plot([], [], [], 'bo', markersize=10)[0],
        'wheel_R': ax.plot([], [], [], 'ro', markersize=10)[0]
    }
    
    def update_frame(frame):
        if frame >= len(results['config']):
            return list(lines.values())
        
        q = results['config'][frame]
        
        # 順運動学
        pin.forwardKinematics(robot.model, robot.data, q)
        
        # 各関節位置を取得
        positions = []
        for i in range(robot.model.njoints):
            pos = robot.data.oMi[i].translation
            positions.append(pos)
        
        # ベース（root_joint）
        base_pos = positions[1]
        lines['base'].set_data_3d(
            [base_pos[0]-0.1, base_pos[0]+0.1],
            [base_pos[1], base_pos[1]],
            [base_pos[2], base_pos[2]]
        )
        
        # 左脚（インデックス確認が必要）
        # upper_link_L=2, lower_link_L=3, wheel_L=4
        leg_L_x = [base_pos[0], positions[2][0], positions[3][0], positions[4][0]]
        leg_L_y = [base_pos[1], positions[2][1], positions[3][1], positions[4][1]]
        leg_L_z = [base_pos[2], positions[2][2], positions[3][2], positions[4][2]]
        lines['leg_L'].set_data_3d(leg_L_x, leg_L_y, leg_L_z)
        
        # 右脚
        # upper_link_R=5, lower_link_R=6, wheel_R=7
        leg_R_x = [base_pos[0], positions[5][0], positions[6][0], positions[7][0]]
        leg_R_y = [base_pos[1], positions[5][1], positions[6][1], positions[7][1]]
        leg_R_z = [base_pos[2], positions[5][2], positions[6][2], positions[7][2]]
        lines['leg_R'].set_data_3d(leg_R_x, leg_R_y, leg_R_z)
        
        # ホイール
        lines['wheel_L'].set_data_3d([positions[4][0]], [positions[4][1]], [positions[4][2]])
        lines['wheel_R'].set_data_3d([positions[7][0]], [positions[7][1]], [positions[7][2]])
        
        # タイトル更新
        t = results['time'][frame]
        ax.set_title(f'Robot Pendulum Motion - t={t:.2f}s')
        
        return list(lines.values())
    
    # アニメーション
    print("🎬 アニメーション作成中...")
    anim = animation.FuncAnimation(
        fig, update_frame, 
        frames=len(results['config']),
        interval=50, blit=False, repeat=True
    )
    
    # 保存
    try:
        anim.save('robot_pendulum_3d.gif', writer='pillow', fps=20)
        print("✅ 保存完了: robot_pendulum_3d.gif")
    except:
        print("⚠️ GIF保存失敗（表示のみ）")
    
    plt.show()
    
    return anim

def analyze_motion():
    """運動の分析"""
    print("\n=== 運動分析 ===")
    
    robot = FixedRobot3D()
    
    # テスト条件
    initial_state = [0.0, 0.0, 0.0, 0.3, -0.8, 0.5, -1.0]
    results = robot.simulate(initial_state, T_sim=0.3, dt=0.01)
    
    # 関節角度の時間変化をプロット
    plt.figure(figsize=(10, 6))
    
    time = results['time']
    joints = results['state'][:, 3:]
    
    plt.subplot(2, 1, 1)
    plt.plot(time, joints[:, 0], 'b-', label='Left Upper')
    plt.plot(time, joints[:, 1], 'b--', label='Left Lower')
    plt.plot(time, joints[:, 2], 'r-', label='Right Upper')
    plt.plot(time, joints[:, 3], 'r--', label='Right Lower')
    plt.xlabel('Time [s]')
    plt.ylabel('Joint Angle [rad]')
    plt.title('Joint Angles vs Time')
    plt.legend()
    plt.grid(True)
    
    # 関節速度
    velocities = results['velocity'][:, 3:]
    
    plt.subplot(2, 1, 2)
    plt.plot(time, velocities[:, 0], 'b-', label='Left Upper')
    plt.plot(time, velocities[:, 1], 'b--', label='Left Lower')
    plt.plot(time, velocities[:, 2], 'r-', label='Right Upper')
    plt.plot(time, velocities[:, 3], 'r--', label='Right Lower')
    plt.xlabel('Time [s]')
    plt.ylabel('Joint Velocity [rad/s]')
    plt.title('Joint Velocities vs Time')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig('robot_motion_analysis.png')
    print("✅ 保存完了: robot_motion_analysis.png")
    plt.show()

if __name__ == "__main__":
    # 運動分析
    analyze_motion()
    
    # アニメーション作成
    create_simple_animation()