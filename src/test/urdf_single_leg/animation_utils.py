#!/usr/bin/env python3
"""
アニメーションユーティリティ
ロボットシミュレーションの2Dアニメーション表示機能
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def create_noslip_robot_animation(t_array, joint_positions_history, com_history, 
                                  theta_wheel_history, phi1_init, phi2_init, wheel_radius):
    """ノンスリップロボットのアニメーション（タイヤ回転表示付き）"""
    print("🎬 ノンスリップアニメーション作成中...")
    
    joint_positions_array = np.array(joint_positions_history)
    com_array = np.array(com_history)
    n_frames = len(joint_positions_array)
    
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # プロット範囲設定
    all_x = joint_positions_array[:, :, 0].flatten()
    all_z = joint_positions_array[:, :, 2].flatten()
    
    x_min, x_max = np.min(all_x) - 0.1, np.max(all_x) + 0.1
    z_min, z_max = np.min(all_z) - 0.1, np.max(all_z) + 0.1
    
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(z_min, z_max)
    ax.set_xlabel('X Position [m]', fontsize=12)
    ax.set_ylabel('Z Position [m]', fontsize=12)
    ax.set_title(f'No-Slip Robot Animation (φ1={phi1_init:.1f}, φ2={phi2_init:.1f})', fontsize=14)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    # 地面ライン
    ax.axhline(y=0, color='brown', linewidth=2, alpha=0.7, label='Ground')
    
    # ロボット描画要素
    robot_lines, = ax.plot([], [], 'b-', linewidth=3, marker='o', markersize=6, label='Robot')
    wheel_circle = plt.Circle((0, 0), wheel_radius, fill=False, color='red', linewidth=2)
    ax.add_patch(wheel_circle)
    
    # タイヤ回転表示用のスポーク
    spoke_line, = ax.plot([], [], 'r-', linewidth=2, alpha=0.8)
    
    # 質量中心
    com_point, = ax.plot([], [], 'go', markersize=10, label='Center of Mass', zorder=5)
    com_trajectory_x, com_trajectory_z = [], []
    com_trajectory_line, = ax.plot([], [], 'g--', alpha=0.5, linewidth=1, label='CoM Trajectory')
    
    # 軌跡
    trajectory_x, trajectory_z = [], []
    trajectory_line, = ax.plot([], [], 'r--', alpha=0.5, linewidth=1, label='Base Trajectory')
    
    # 時間・角度表示
    time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, fontsize=12,
                       verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    ax.legend()
    
    def animate(frame):
        if frame >= n_frames:
            frame = n_frames - 1
            
        # 現在フレームの関節位置
        positions = joint_positions_array[frame]
        
        # ロボットの線分を描画
        x_coords = positions[:, 0]
        z_coords = positions[:, 2]
        robot_lines.set_data(x_coords, z_coords)
        
        # ホイール円の位置更新
        wheel_pos = positions[-1]
        wheel_circle.center = (wheel_pos[0], wheel_pos[2])
        
        # タイヤ回転表示（スポーク）
        theta_wheel = theta_wheel_history[frame]
        spoke_start_x = wheel_pos[0]
        spoke_start_z = wheel_pos[2]
        spoke_end_x = spoke_start_x + wheel_radius * np.cos(theta_wheel + np.pi/2)
        spoke_end_z = spoke_start_z + wheel_radius * np.sin(theta_wheel + np.pi/2)
        spoke_line.set_data([spoke_start_x, spoke_end_x], [spoke_start_z, spoke_end_z])
        
        # 質量中心の更新
        com_pos = com_array[frame]
        com_point.set_data([com_pos[0]], [com_pos[2]])
        
        # 軌跡更新
        com_trajectory_x.append(com_pos[0])
        com_trajectory_z.append(com_pos[2])
        com_trajectory_line.set_data(com_trajectory_x, com_trajectory_z)
        
        trajectory_x.append(positions[0, 0])
        trajectory_z.append(positions[0, 2])
        trajectory_line.set_data(trajectory_x, trajectory_z)
        
        # 情報表示
        current_time = t_array[frame] if frame < len(t_array) else t_array[-1]
        time_text.set_text(f'Time: {current_time:.2f}s\nWheel θ: {theta_wheel:.2f}rad')
        
        return robot_lines, wheel_circle, spoke_line, com_point, com_trajectory_line, trajectory_line, time_text
    
    # アニメーション作成
    anim = animation.FuncAnimation(fig, animate, frames=n_frames, 
                                 interval=50, blit=True, repeat=True)
    
    # 保存
    filename = f'robot_noslip_phi1_{phi1_init:.1f}_phi2_{phi2_init:.1f}.gif'
    print(f"💾 アニメーション保存中: {filename}")
    anim.save(filename, writer='pillow', fps=20)
    
    plt.show()
    print(f"✅ ノンスリップアニメーション完成: {filename}")

def create_robot_animation(t_array, joint_positions_history, com_history, phi1_init, phi2_init, wheel_radius):
    """ロボットの倒れる様子をx-z平面でアニメーション描画（質量中心付き）"""
    print("🎬 アニメーション作成中...")
    
    # データを numpy 配列に変換
    joint_positions_array = np.array(joint_positions_history)
    com_array = np.array(com_history)
    n_frames = len(joint_positions_array)
    n_joints = joint_positions_array.shape[1]
    
    # フィギュアとアクシスの設定
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # プロット範囲を設定（全フレームの最大最小を考慮）
    all_x = joint_positions_array[:, :, 0].flatten()
    all_z = joint_positions_array[:, :, 2].flatten()
    
    x_min, x_max = np.min(all_x) - 0.1, np.max(all_x) + 0.1
    z_min, z_max = np.min(all_z) - 0.1, np.max(all_z) + 0.1
    
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(z_min, z_max)
    ax.set_xlabel('X Position [m]', fontsize=12)
    ax.set_ylabel('Z Position [m]', fontsize=12)
    ax.set_title(f'Robot Animation (φ1={phi1_init:.1f}, φ2={phi2_init:.1f})', fontsize=14)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    # 地面ライン（Z=0）
    ax.axhline(y=0, color='brown', linewidth=2, alpha=0.7, label='Ground')
    
    # ロボットの線分とポイント
    robot_lines, = ax.plot([], [], 'b-', linewidth=3, marker='o', markersize=6, label='Robot')
    wheel_circle = plt.Circle((0, 0), wheel_radius, fill=False, color='red', linewidth=2)
    ax.add_patch(wheel_circle)
    
    # 質量中心
    com_point, = ax.plot([], [], 'go', markersize=10, label='Center of Mass', zorder=5)
    com_trajectory_x, com_trajectory_z = [], []
    com_trajectory_line, = ax.plot([], [], 'g--', alpha=0.5, linewidth=1, label='CoM Trajectory')
    
    # 軌跡
    trajectory_x, trajectory_z = [], []
    trajectory_line, = ax.plot([], [], 'r--', alpha=0.5, linewidth=1, label='Base Trajectory')
    
    # 時間表示
    time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, fontsize=12,
                       verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    ax.legend()
    
    def animate(frame):
        if frame >= n_frames:
            frame = n_frames - 1
            
        # 現在フレームの関節位置
        positions = joint_positions_array[frame]
        
        # ロボットの線分を描画（関節を線で繋ぐ）
        x_coords = positions[:, 0]
        z_coords = positions[:, 2]
        
        robot_lines.set_data(x_coords, z_coords)
        
        # ホイール円の位置更新（最後の関節=ホイール）
        wheel_pos = positions[-1]
        wheel_circle.center = (wheel_pos[0], wheel_pos[2])
        
        # 質量中心の更新
        com_pos = com_array[frame]
        com_point.set_data([com_pos[0]], [com_pos[2]])
        
        # 質量中心の軌跡
        com_trajectory_x.append(com_pos[0])
        com_trajectory_z.append(com_pos[2])
        com_trajectory_line.set_data(com_trajectory_x, com_trajectory_z)
        
        # ベースの軌跡（最初の関節=ベース）
        trajectory_x.append(positions[0, 0])
        trajectory_z.append(positions[0, 2])
        trajectory_line.set_data(trajectory_x, trajectory_z)
        
        # 時間表示
        current_time = t_array[frame] if frame < len(t_array) else t_array[-1]
        time_text.set_text(f'Time: {current_time:.2f}s')
        
        return robot_lines, wheel_circle, com_point, com_trajectory_line, trajectory_line, time_text
    
    # アニメーション作成
    anim = animation.FuncAnimation(fig, animate, frames=n_frames, 
                                 interval=50, blit=True, repeat=True)
    
    # 保存
    filename = f'robot_animation_phi1_{phi1_init:.1f}_phi2_{phi2_init:.1f}.gif'
    print(f"💾 アニメーション保存中: {filename}")
    anim.save(filename, writer='pillow', fps=20)
    
    plt.show()
    print(f"✅ アニメーション完成: {filename}")