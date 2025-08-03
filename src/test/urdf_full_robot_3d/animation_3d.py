#!/usr/bin/env python3
"""
3Dロボットアニメーション専用モジュール
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import pinocchio as pin

def create_3d_animation(robot, results, save_gif=True, show_plot=True):
    """
    3Dアニメーションを作成
    
    Args:
        robot: FixedRobot3D インスタンス
        results: simulate() の結果辞書
        save_gif: GIFファイルとして保存するか
        show_plot: アニメーションを表示するか
    
    Returns:
        animation オブジェクト
    """
    print("🎬 3Dアニメーション作成中...")
    
    # アニメーション設定
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    
    # 軸設定
    ax.set_xlim([-0.6, 0.6])
    ax.set_ylim([-0.6, 0.6])
    ax.set_zlim([0, 0.5])
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title('6DOF Robot Dynamics Animation')
    
    # 地面
    x_ground = np.linspace(-0.6, 0.6, 10)
    y_ground = np.linspace(-0.6, 0.6, 10)
    X, Y = np.meshgrid(x_ground, y_ground)
    Z = np.zeros_like(X)
    ax.plot_surface(X, Y, Z, alpha=0.2, color='gray')
    
    # ロボット構造の線とマーカー
    lines = {
        'base': ax.plot([], [], [], 'k-', linewidth=5, label='Base')[0],
        'leg_L': ax.plot([], [], [], 'b-', linewidth=3, label='Left Leg')[0],
        'leg_R': ax.plot([], [], [], 'r-', linewidth=3, label='Right Leg')[0],
        'wheel_L': ax.plot([], [], [], 'bo', markersize=12, label='Left Wheel')[0],
        'wheel_R': ax.plot([], [], [], 'ro', markersize=12, label='Right Wheel')[0],
        'trajectory': ax.plot([], [], [], 'g--', alpha=0.5, label='Base Trajectory')[0]
    }
    
    # 軌跡記録用
    trajectory_x, trajectory_y, trajectory_z = [], [], []
    
    def update_frame(frame):
        if frame >= len(results['config']) or frame >= len(results['time']) or frame >= len(results['state']):
            return list(lines.values())
        
        q = results['config'][frame]
        t = results['time'][frame]
        state = results['state'][frame]
        
        # 順運動学
        pin.forwardKinematics(robot.model, robot.data, q)
        
        # 各関節位置を取得
        positions = []
        for i in range(robot.model.njoints):
            pos = robot.data.oMi[i].translation
            positions.append(pos)
        
        # ベース位置（root_joint = index 1）
        base_pos = positions[1]
        
        # 軌跡更新
        trajectory_x.append(base_pos[0])
        trajectory_y.append(base_pos[1])
        trajectory_z.append(base_pos[2])
        
        # ベース描画（幅を持たせて表示）
        base_width = 0.15
        base_left = base_pos + np.array([0, base_width/2, 0])
        base_right = base_pos + np.array([0, -base_width/2, 0])
        lines['base'].set_data_3d(
            [base_left[0], base_right[0]],
            [base_left[1], base_right[1]], 
            [base_left[2], base_right[2]]
        )
        
        # 左脚の描画
        # base -> upper_L -> lower_L -> wheel_L
        leg_L_x = [base_pos[0], positions[2][0], positions[3][0], positions[4][0]]
        leg_L_y = [base_pos[1], positions[2][1], positions[3][1], positions[4][1]]
        leg_L_z = [base_pos[2], positions[2][2], positions[3][2], positions[4][2]]
        lines['leg_L'].set_data_3d(leg_L_x, leg_L_y, leg_L_z)
        
        # 右脚の描画
        # base -> upper_R -> lower_R -> wheel_R
        leg_R_x = [base_pos[0], positions[5][0], positions[6][0], positions[7][0]]
        leg_R_y = [base_pos[1], positions[5][1], positions[6][1], positions[7][1]]
        leg_R_z = [base_pos[2], positions[5][2], positions[6][2], positions[7][2]]
        lines['leg_R'].set_data_3d(leg_R_x, leg_R_y, leg_R_z)
        
        # ホイール描画
        lines['wheel_L'].set_data_3d([positions[4][0]], [positions[4][1]], [positions[4][2]])
        lines['wheel_R'].set_data_3d([positions[7][0]], [positions[7][1]], [positions[7][2]])
        
        # 軌跡描画
        if len(trajectory_x) > 1:
            lines['trajectory'].set_data_3d(trajectory_x, trajectory_y, trajectory_z)
        
        # タイトル更新（状態情報表示：9自由度対応）
        ax.set_title(f'9DOF Robot: t={t:.2f}s | '
                    f'Pitch={state[2]:.2f} | '
                    f'Knee=[{state[3]:.2f}, {state[4]:.2f}] | '
                    f'Hip=[{state[5]:.2f}, {state[6]:.2f}] | '
                    f'Wheels=[{state[7]:.2f}, {state[8]:.2f}]')
        
        return list(lines.values())
    
    # アニメーション作成（最小配列長に合わせる）
    max_frames = min(len(results['config']), len(results['time']), len(results['state']))
    anim = animation.FuncAnimation(
        fig, update_frame, 
        frames=max_frames,
        interval=50, blit=False, repeat=True
    )
    
    # 凡例追加
    ax.legend(loc='upper right')
    
    # 保存
    if save_gif:
        try:
            anim.save('robot_pendulum_3d.gif', writer='pillow', fps=20)
            print("✅ アニメーション保存完了: robot_pendulum_3d.gif")
        except Exception as e:
            print(f"⚠️ GIF保存失敗: {e}")
    
    # 表示
    if show_plot:
        plt.show()
    
    return anim

def plot_motion_analysis(results, save_plot=True, show_plot=True):
    """
    運動分析プロット作成
    
    Args:
        results: simulate() の結果辞書
        save_plot: プロットを保存するか
        show_plot: プロットを表示するか
    """
    print("📊 運動分析プロット作成中...（簡素化版）")
    
    time = results['time']
    
    # 配列長の調整
    min_length = min(len(time), len(results['state']), len(results['config']))
    time = time[:min_length]
    state_data = results['state'][:min_length]
    
    # フローティングベース6成分を取得（q[0:6] = [x, y, z, roll, pitch, yaw]）
    base_positions = np.zeros((min_length, 6))
    for i in range(min_length):
        q = results['config'][i]
        base_positions[i, :] = q[0:6]  # [x, y, z, roll, pitch, yaw]
    
    # 関節角度（9自由度）
    pitch_data = state_data[:, 2]       # pitch（独立変数）
    knee_joints = state_data[:, 3:5]    # [phi_L_lower, phi_R_lower]
    hip_joints = state_data[:, 5:7]     # [phi_L_upper, phi_R_upper]
    wheels = state_data[:, 7:9]         # [wheel_L, wheel_R]
    
    # 2x3プロット（Base Pitchを分離）
    fig, axes = plt.subplots(2, 3, figsize=(20, 10))
    
    # フローティングベース位置（X, Y, Z）
    axes[0, 0].plot(time, base_positions[:, 0], 'b-', linewidth=2, label='X Position')
    axes[0, 0].plot(time, base_positions[:, 1], 'r-', linewidth=2, label='Y Position')
    axes[0, 0].plot(time, base_positions[:, 2], 'g-', linewidth=2, label='Z Position')
    axes[0, 0].set_xlabel('Time [s]')
    axes[0, 0].set_ylabel('Position [m]')
    axes[0, 0].set_title('Floating Base Position vs Time')
    axes[0, 0].set_xlim(0, 1.0)  # 時間軸を1秒でカット
    # 縦軸を1秒地点の値に合わせて調整
    t_1s_idx = min(len(time)-1, int(1.0/((time[1]-time[0]) if len(time)>1 else 0.001)))
    if t_1s_idx > 0:
        y_max = max(abs(base_positions[t_1s_idx, 0]), abs(base_positions[t_1s_idx, 1]), abs(base_positions[t_1s_idx, 2]))
        axes[0, 0].set_ylim(-y_max*1.1, y_max*1.1)
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # フローティングベース姿勢（Roll, Yaw のみ）
    axes[0, 1].plot(time, base_positions[:, 3], 'c-', linewidth=2, label='Roll')
    axes[0, 1].plot(time, base_positions[:, 5], 'y-', linewidth=2, label='Yaw')
    axes[0, 1].set_xlabel('Time [s]')
    axes[0, 1].set_ylabel('Orientation [rad]')
    axes[0, 1].set_title('Floating Base Roll & Yaw vs Time')
    axes[0, 1].set_xlim(0, 1.0)  # 時間軸を1秒でカット
    # 縦軸を1秒地点の値に合わせて調整
    if t_1s_idx > 0:
        y_max = max(abs(base_positions[t_1s_idx, 3]), abs(base_positions[t_1s_idx, 5]))
        axes[0, 1].set_ylim(-y_max*1.1, y_max*1.1)
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # Base Pitch（独立変数）- 独立したグラフ
    axes[0, 2].plot(time, base_positions[:, 4], 'm-', linewidth=2, label='Pitch (Independent)')
    axes[0, 2].set_xlabel('Time [s]')
    axes[0, 2].set_ylabel('Pitch [rad]')
    axes[0, 2].set_title('Base Pitch vs Time (Independent Variable)')
    axes[0, 2].set_xlim(0, 1.0)  # 時間軸を1秒でカット
    # 縦軸を1秒地点の値に合わせて調整
    if t_1s_idx > 0:
        y_max = abs(base_positions[t_1s_idx, 4])
        axes[0, 2].set_ylim(-y_max*1.1, y_max*1.1)
    axes[0, 2].legend()
    axes[0, 2].grid(True, alpha=0.3)
    
    # 脚関節角度（膝+腰）
    axes[1, 0].plot(time, knee_joints[:, 0], 'b-', linewidth=2, label='Left Knee')
    axes[1, 0].plot(time, knee_joints[:, 1], 'r-', linewidth=2, label='Right Knee')
    axes[1, 0].plot(time, hip_joints[:, 0], 'g--', linewidth=2, label='Left Hip')
    axes[1, 0].plot(time, hip_joints[:, 1], 'm--', linewidth=2, label='Right Hip')
    axes[1, 0].set_xlabel('Time [s]')
    axes[1, 0].set_ylabel('Joint Angle [rad]')
    axes[1, 0].set_title('Leg Joint Angles vs Time')
    axes[1, 0].set_xlim(0, 1.0)  # 時間軸を1秒でカット
    # 縦軸を1秒地点の値に合わせて調整
    if t_1s_idx > 0:
        y_max = max(abs(knee_joints[t_1s_idx, 0]), abs(knee_joints[t_1s_idx, 1]), 
                   abs(hip_joints[t_1s_idx, 0]), abs(hip_joints[t_1s_idx, 1]))
        axes[1, 0].set_ylim(-y_max*1.1, y_max*1.1)
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # ホイール角度
    axes[1, 1].plot(time, wheels[:, 0], 'b-', linewidth=2, label='Left Wheel')
    axes[1, 1].plot(time, wheels[:, 1], 'r-', linewidth=2, label='Right Wheel')
    axes[1, 1].set_xlabel('Time [s]')
    axes[1, 1].set_ylabel('Wheel Angle [rad]')
    axes[1, 1].set_title('Wheel Angles vs Time')
    axes[1, 1].set_xlim(0, 1.0)  # 時間軸を1秒でカット
    # 縦軸を1秒地点の値に合わせて調整
    if t_1s_idx > 0:
        y_max = max(abs(wheels[t_1s_idx, 0]), abs(wheels[t_1s_idx, 1]))
        axes[1, 1].set_ylim(-y_max*1.1, y_max*1.1)
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    
    # 独立変数Pitchの状態データ（状態変数から）
    axes[1, 2].plot(time, pitch_data, 'm-', linewidth=2, label='Pitch (State Variable)')
    axes[1, 2].set_xlabel('Time [s]')
    axes[1, 2].set_ylabel('Pitch [rad]')
    axes[1, 2].set_title('State Variable: Pitch vs Time')
    axes[1, 2].set_xlim(0, 1.0)  # 時間軸を1秒でカット
    # 縦軸を1秒地点の値に合わせて調整
    if t_1s_idx > 0:
        y_max = abs(pitch_data[t_1s_idx])
        axes[1, 2].set_ylim(-y_max*1.1, y_max*1.1)
    axes[1, 2].legend()
    axes[1, 2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # 保存
    if save_plot:
        plt.savefig('robot_motion_analysis.png', dpi=150, bbox_inches='tight')
        print("✅ 運動分析保存完了: robot_motion_analysis.png（簡素化版）")
    
    # 表示
    if show_plot:
        plt.show()

def plot_trajectory_2d(results, save_plot=True, show_plot=True):
    """
    2D軌跡プロット作成
    
    Args:
        results: simulate() の結果辞書
        save_plot: プロットを保存するか
        show_plot: プロットを表示するか
    """
    print("🗺️ 2D軌跡プロット作成中...")
    
    # ベース位置の軌跡を計算（完全構成から）
    base_x, base_y = [], []
    
    for q in results['config']:
        # ベース位置はq[0], q[1]
        base_x.append(q[0])
        base_y.append(q[1])
    
    base_x = np.array(base_x)
    base_y = np.array(base_y)
    time = results['time']
    
    # 配列長の調整（シミュレーション途中終了対応）
    min_length = min(len(time), len(base_x))
    time = time[:min_length]
    base_x = base_x[:min_length]
    base_y = base_y[:min_length]
    
    # プロット作成
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    # XY軌跡プロット
    ax1.plot(base_x, base_y, 'g-', linewidth=2, label='Base Trajectory')
    ax1.plot(base_x[0], base_y[0], 'go', markersize=10, label='Start')
    ax1.plot(base_x[-1], base_y[-1], 'rs', markersize=10, label='End')
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_title('Robot Base Trajectory (XY View)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')
    
    # 時間変化プロット
    ax2.plot(time, base_x, 'b-', linewidth=2, label='X Position')
    ax2.plot(time, base_y, 'r-', linewidth=2, label='Y Position')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Position [m]')
    ax2.set_title('Base Position vs Time')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # 保存
    if save_plot:
        plt.savefig('robot_trajectory_2d.png', dpi=150, bbox_inches='tight')
        print("✅ 軌跡プロット保存完了: robot_trajectory_2d.png")
    
    # 表示
    if show_plot:
        plt.show()

def create_comprehensive_analysis(robot, results, save_all=True, show_all=True):
    """
    包括的な分析とアニメーションを作成
    
    Args:
        robot: FixedRobot3D インスタンス
        results: simulate() の結果辞書
        save_all: すべてのファイルを保存するか
        show_all: すべてのプロットを表示するか
    
    Returns:
        animation オブジェクト
    """
    print("\n🎯 包括的分析開始...")
    
    # 各種分析とアニメーション作成
    anim = create_3d_animation(robot, results, save_gif=save_all, show_plot=show_all)
    plot_motion_analysis(results, save_plot=save_all, show_plot=show_all)
    plot_trajectory_2d(results, save_plot=save_all, show_plot=show_all)
    
    print("✅ 包括的分析完了！")
    return anim