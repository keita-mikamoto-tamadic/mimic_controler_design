#!/usr/bin/env python3
"""
シンプルな拘束付きシミュレーション
simple_testの手法をURDF single_legモデル（6自由度→5自由度）に適用
フローティングベース（3自由度）＋3関節 - 1拘束（接地）= 5自由度
"""

import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os

WHEEL_RADIUS = (77.95 / 2) / 1000  # [m]

def load_model():
    """モデル読み込み"""
    base_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(base_dir, "../../../urdf/mimic_v1_single_leg.urdf")
    
    model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
    data = model.createData()
    model.gravity.linear = np.array([0, 0, +9.81])  # 上向き重力でテスト
    
    return model, data

def compute_base_height(phi1, phi2, model, data):
    """
    simple_testアプローチ: 接地拘束を満たすベース高度を計算
    """
    # 仮構成
    q = pin.neutral(model)
    q[7] = np.cos(phi1)   # RUBY関節
    q[8] = np.sin(phi1)
    q[9] = np.cos(phi2)
    q[10] = np.sin(phi2)
    q[11] = 1.0  # wheel
    q[12] = 0.0
    
    # 順運動学
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    
    # ホイール位置
    wheel_pos = data.oMi[model.njoints-1].translation
    
    # 接地拘束: wheel_bottom = 0
    # wheel_center_z - WHEEL_RADIUS + base_height = 0
    # base_height = WHEEL_RADIUS - wheel_center_z
    base_height = WHEEL_RADIUS - wheel_pos[2]
    
    return base_height

def compute_com(model, data, q):
    """全体の質量中心を計算"""
    pin.centerOfMass(model, data, q, False)  # False = velocityを計算しない
    return data.com[0].copy()  # 全体のCoM位置をコピーして返す

def compute_3dof_dynamics(x_base, phi1, phi2, dx_base, dphi1, dphi2, model, data):
    """5自由度動力学: [x_base, z_base, pitch_base, phi1, phi2]から拘束で縮約"""
    # ベース高度を拘束条件から計算
    q_temp = pin.neutral(model)
    q_temp[0] = x_base  # X position（変動する）
    q_temp[7] = np.cos(phi1)
    q_temp[8] = np.sin(phi1)
    q_temp[9] = np.cos(phi2)
    q_temp[10] = np.sin(phi2)
    q_temp[11] = 1.0
    q_temp[12] = 0.0
    
    # 順運動学でホイール位置計算
    pin.forwardKinematics(model, data, q_temp)
    pin.updateFramePlacements(model, data)
    wheel_pos = data.oMi[model.njoints-1].translation
    
    # 接地拘束を満たすZ位置
    base_height = WHEEL_RADIUS - wheel_pos[2]
    
    # 完全構成
    q = pin.neutral(model)
    q[0] = x_base         # X（変動）
    q[2] = base_height    # Z（拘束により決定）
    q[7] = np.cos(phi1)
    q[8] = np.sin(phi1)
    q[9] = np.cos(phi2)
    q[10] = np.sin(phi2)
    q[11] = 1.0
    q[12] = 0.0
    
    # ベースZ速度を計算（simple_testのアプローチ）
    # 接地点速度=0の拘束から導出
    pin.computeJointJacobians(model, data, q)
    J_wheel = pin.getFrameJacobian(model, data, model.njoints-1, pin.ReferenceFrame.WORLD)
    
    # 速度ベクトル構築
    dq = np.zeros(model.nv)
    dq[0] = dx_base    # dx_base
    dq[6] = dphi1      # dphi1
    dq[7] = dphi2      # dphi2
    
    # 接地点Z速度=0の拘束からdz_baseを計算
    J_contact_z = J_wheel[2, :]  # Z方向ヤコビアン
    if abs(J_contact_z[2]) > 1e-6:  # dz成分が0でない場合
        other_contribution = (J_contact_z[0] * dx_base + 
                            J_contact_z[6] * dphi1 + 
                            J_contact_z[7] * dphi2)
        dq[2] = -other_contribution / J_contact_z[2]  # dz_base
    
    # 動力学計算
    pin.crba(model, data, q)
    pin.computeGeneralizedGravity(model, data, q)
    pin.computeCoriolisMatrix(model, data, q, dq)
    
    M = data.M
    g = data.g
    C = data.C @ dq
    
    # 3×3縮約: [x_base, phi1, phi2]
    indices = [0, 6, 7]  # 独立変数のインデックス
    M_red = M[np.ix_(indices, indices)]
    g_red = g[indices]
    C_red = C[indices]
    
    return M_red, g_red, C_red, q

def get_joint_positions(q, model, data):
    """各関節の位置を取得してロボットの形状データを返す"""
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    
    positions = []
    # ベース位置（フローティングベース）
    base_pos = data.oMi[1].translation  # joint index 1 = universe -> base
    positions.append(base_pos)
    
    # 各関節位置を取得
    for i in range(2, model.njoints):  # joint 2以降（base以降）
        joint_pos = data.oMi[i].translation
        positions.append(joint_pos)
    
    return np.array(positions)

def create_robot_animation(t_array, joint_positions_history, com_history, phi1_init, phi2_init):
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
    wheel_circle = plt.Circle((0, 0), WHEEL_RADIUS, fill=False, color='red', linewidth=2)
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

def simulate_simple(phi1_init, phi2_init, T_sim=3.0, dt=0.02):
    """5自由度シミュレーション（縮約後）: 実効的には[x_base, phi1, phi2]で制御"""
    model, data = load_model()
    
    # 独立変数（実効的に制御する3変数）
    x_base = 0.0  # 初期X位置
    phi1, phi2 = phi1_init, phi2_init
    dx_base = 0.0  # 初期X速度
    dphi1, dphi2 = 0.0, 0.0
    
    t_array = np.arange(0, T_sim, dt)
    N = len(t_array)
    
    state_history = np.zeros((N, 3))  # [x_base, phi1, phi2]
    base_heights = np.zeros(N)
    x_positions = np.zeros(N)
    joint_positions_history = []  # 各関節位置の時系列データ
    com_history = []  # 質量中心の時系列データ
    
    print(f"5自由度拘束シミュレーション（simple_testアプローチ）")
    print(f"初期: x={x_base:.3f}, φ1={phi1_init:.3f}, φ2={phi2_init:.3f}")
    
    for i, t in enumerate(t_array):
        state_history[i] = [x_base, phi1, phi2]
        x_positions[i] = x_base
        
        # 動力学計算（拘束による縮約後）
        M_red, g_red, C_red, q = compute_3dof_dynamics(x_base, phi1, phi2, dx_base, dphi1, dphi2, model, data)
        base_heights[i] = q[2]
        
        # 各関節位置を記録
        joint_positions = get_joint_positions(q, model, data)
        joint_positions_history.append(joint_positions)
        
        # 質量中心を計算・記録
        com = compute_com(model, data, q)
        com_history.append(com)
        
        if i % 20 == 0:
            print(f"t={t:.2f}s: x={x_base:.3f}, φ1={phi1:.3f}, φ2={phi2:.3f}, h={base_heights[i]:.3f}m")
        
        # 運動方程式（3×3）
        tau = np.zeros(3)  # 無制御
        
        try:
            dd_state = np.linalg.solve(M_red, tau - g_red - C_red)
        except:
            dd_state = np.zeros(3)
        
        # 積分
        dx_base += dd_state[0] * dt
        dphi1 += dd_state[1] * dt
        dphi2 += dd_state[2] * dt
        
        x_base += dx_base * dt
        phi1 += dphi1 * dt
        phi2 += dphi2 * dt
        
        # 発散チェック
        if not (np.isfinite(x_base) and np.isfinite(phi1) and np.isfinite(phi2)):
            print(f"発散: t={t:.3f}")
            break
    
    print(f"完了: 最終位置 x={x_base:.3f}m, 高度={base_heights[-1]:.3f}m")
    
    # アニメーション描画
    create_robot_animation(t_array[:i+1], joint_positions_history, com_history, phi1_init, phi2_init)
    
    # プロット（3つのグラフ）
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 5))
    
    # 関節角度
    ax1.plot(t_array[:i+1], state_history[:i+1, 1] * 180/np.pi, 'r-', label='φ1')
    ax1.plot(t_array[:i+1], state_history[:i+1, 2] * 180/np.pi, 'b-', label='φ2')
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Angle [deg]')
    ax1.set_title('Joint Angles')
    ax1.legend()
    ax1.grid(True)
    
    # ベース高度
    ax2.plot(t_array[:i+1], base_heights[:i+1], 'g-', linewidth=2)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Base Height [m]')
    ax2.set_title('Base Height (Should Fall!)')
    ax2.grid(True)
    
    # ベースX位置
    ax3.plot(t_array[:i+1], x_positions[:i+1], 'm-', linewidth=2)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Base X Position [m]')
    ax3.set_title('Base Horizontal Motion')
    ax3.grid(True)
    
    plt.tight_layout()
    plt.savefig('3dof_constrained_result.png', dpi=150, bbox_inches='tight')
    plt.show()
    
    return t_array[:i+1], state_history[:i+1], base_heights[:i+1], x_positions[:i+1]

if __name__ == "__main__":
    print("🧪 異なる初期条件でテスト:")
    print()
    
    # テスト1: 現在の条件
    print("1. 小さな傾斜（現状）:")
    simulate_simple(0.3, -0.6)
    
    # テスト2: より大きな傾斜
    print("\n2. 大きな傾斜:")
    simulate_simple(0.8, -1.2)
    
    # テスト3: 非常に大きな傾斜
    print("\n3. 非常に大きな傾斜:")
    simulate_simple(1.2, -1.5)