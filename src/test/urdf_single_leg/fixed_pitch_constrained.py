#!/usr/bin/env python3
"""
base_pitchを考慮した拘束付きシミュレーション
4自由度システム: [x_base, pitch_base, phi1, phi2]
フローティングベースのpitch回転を独立変数として扱い、
任意の初期姿勢から動作可能なシステムを実現
"""

import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os
from scipy.spatial.transform import Rotation

WHEEL_RADIUS = (77.95 / 2) / 1000  # [m]

def correct_gravity_to_world(q, g_original, model, data):
    # ワールド座標系での重力加速度
    g_world = np.array([0, 0, -9.81])
    
    # フローティングベースの姿勢を取得
    base_rotation = data.oMi[1].rotation  # ベースの回転行列
    
    # ワールド座標系の重力をベース座標系に変換
    g_base = base_rotation.T @ g_world
    
    # 補正された重力項を計算
    g_corrected = g_original.copy()
    
    # フローティングベースの並進部分（0-2）の重力項を補正
    # 重力は質量×加速度なので、総質量を考慮
    total_mass = sum(model.inertias[i].mass for i in range(1, len(model.inertias)))
    
    g_corrected[0] = total_mass * g_base[0]  # X方向
    g_corrected[1] = total_mass * g_base[1]  # Y方向  
    g_corrected[2] = total_mass * g_base[2]  # Z方向
    
    # 回転部分（3-5）は関節の重力トルクを補正
    # これは複雑なので、簡単な近似として：
    # pitch軸（4番目）の重力トルクを補正
    
    return g_corrected

def load_model():
    """モデル読み込み"""
    base_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(base_dir, "../../../urdf/mimic_v1_single_leg.urdf")
    
    model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
    data = model.createData()
    model.gravity.linear = np.array([0, 0, -9.81])  # 下向き重力（正しい物理）
    
    return model, data

def compute_base_height_with_pitch(x_base, pitch_base, phi1, phi2, model, data):
    """
    base_pitchを考慮した接地拘束を満たすベース高度を計算
    pitch≠0でも正しく動作するように修正
    """
    # まず適当なz_baseで構成を作成
    q = pin.neutral(model)
    q[0] = x_base
    q[2] = 0.5  # 初期推定値（地面から十分離れた位置）
    
    # base_pitchをクォータニオンで設定
    if pitch_base != 0:
        r = Rotation.from_euler('y', pitch_base, degrees=False)  # ラジアン入力に修正
        quat_xyzw = r.as_quat()
        q[3] = quat_xyzw[0]  # x
        q[4] = quat_xyzw[1]  # y
        q[5] = quat_xyzw[2]  # z
        q[6] = quat_xyzw[3]  # w
    else:
        q[3] = 0.0  # x
        q[4] = 0.0  # y
        q[5] = 0.0  # z
        q[6] = 1.0  # w
    
    # 関節角度設定
    q[7] = np.cos(phi1)
    q[8] = np.sin(phi1)
    q[9] = np.cos(phi2)
    q[10] = np.sin(phi2)
    q[11] = 1.0  # wheel angle (temporary)
    q[12] = 0.0
    
    # 順運動学
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    
    # ホイール位置取得
    wheel_pos = data.oMi[model.njoints-1].translation
    
    # 接地拘束: ホイール底面がz=0になるようベース高度を調整
    # ワールド座標でのオフセット計算
    wheel_bottom_z = wheel_pos[2] - WHEEL_RADIUS
    base_height = q[2] - wheel_bottom_z  # 現在のベース高度から必要な補正量を計算
    
    return base_height

def compute_wheel_angle_from_noslip_with_pitch(x_base, pitch_base, phi1, phi2, model, data):
    """
    base_pitchを考慮したノンスリップ拘束からタイヤ角度を計算
    """
    # まずz_baseを計算（x_baseも考慮）
    z_base = compute_base_height_with_pitch(x_base, pitch_base, phi1, phi2, model, data)
    
    # 完全な構成を構築
    q = pin.neutral(model)
    q[0] = x_base
    q[2] = z_base
    
    # base_pitchを設定
    if pitch_base != 0:
        r = Rotation.from_euler('y', pitch_base, degrees=False)  # ラジアン入力に修正
        quat_xyzw = r.as_quat()
        q[3] = quat_xyzw[0]  # x
        q[4] = quat_xyzw[1]  # y
        q[5] = quat_xyzw[2]  # z
        q[6] = quat_xyzw[3]  # w
    else:
        q[3] = 0.0  # x
        q[4] = 0.0  # y
        q[5] = 0.0  # z
        q[6] = 1.0  # w
    
    # 関節角度
    q[7] = np.cos(phi1)
    q[8] = np.sin(phi1)
    q[9] = np.cos(phi2)
    q[10] = np.sin(phi2)
    q[11] = 1.0
    q[12] = 0.0
    
    # 順運動学でホイール中心位置を計算
    pin.forwardKinematics(model, data, q)
    wheel_center = data.oMi[model.njoints-1].translation
    
    # ノンスリップ条件: θ_wheel = -x_wheel_center / r
    theta_wheel = -wheel_center[0] / WHEEL_RADIUS
    
    return theta_wheel

def compute_constrained_velocities(x_base, pitch_base, phi1, phi2,
                                  dx_base, dpitch_base, dphi1, dphi2,
                                  model, data):
    """
    拘束条件から従属速度（dz_base, dtheta_wheel）を計算
    """
    # 現在の構成を構築（x_baseも考慮）
    z_base = compute_base_height_with_pitch(x_base, pitch_base, phi1, phi2, model, data)
    theta_wheel = compute_wheel_angle_from_noslip_with_pitch(x_base, pitch_base, phi1, phi2, model, data)
    
    q = pin.neutral(model)
    q[0] = x_base
    q[2] = z_base
    if pitch_base != 0:
        r = Rotation.from_euler('y', pitch_base, degrees=False)  # ラジアン入力に修正
        quat_xyzw = r.as_quat()
        q[3] = quat_xyzw[0]  # x
        q[4] = quat_xyzw[1]  # y
        q[5] = quat_xyzw[2]  # z
        q[6] = quat_xyzw[3]  # w
    else:
        q[3] = 0.0  # x
        q[4] = 0.0  # y
        q[5] = 0.0  # z
        q[6] = 1.0  # w
    q[7] = np.cos(phi1)
    q[8] = np.sin(phi1)
    q[9] = np.cos(phi2)
    q[10] = np.sin(phi2)
    q[11] = np.cos(theta_wheel)
    q[12] = np.sin(theta_wheel)
    
    # ヤコビアン計算
    pin.computeJointJacobians(model, data, q)
    J_wheel = pin.getFrameJacobian(model, data, model.njoints-1, pin.ReferenceFrame.WORLD)
    
    # 1. 接地点Z速度 = 0 から dz_base を計算
    J_contact_z = J_wheel[2, :]
    if abs(J_contact_z[2]) > 1e-6:
        # 独立変数の寄与を計算
        other_contribution = (J_contact_z[0] * dx_base + 
                            J_contact_z[4] * dpitch_base +
                            J_contact_z[6] * dphi1 + 
                            J_contact_z[7] * dphi2)
        dz_base = -other_contribution / J_contact_z[2]
    else:
        dz_base = 0.0
    
    # 2. ノンスリップ条件から dtheta_wheel を計算
    # ホイール中心のx方向速度
    wheel_center_velocity_x = (J_wheel[0, 0] * dx_base + 
                              J_wheel[0, 2] * dz_base +
                              J_wheel[0, 4] * dpitch_base +
                              J_wheel[0, 6] * dphi1 + 
                              J_wheel[0, 7] * dphi2)
    
    # ノンスリップ条件: v_wheel_center = -r * ω_wheel
    dtheta_wheel = -wheel_center_velocity_x / WHEEL_RADIUS
    
    return dz_base, dtheta_wheel

def compute_4dof_pitch_dynamics(x_base, pitch_base, phi1, phi2, 
                                dx_base, dpitch_base, dphi1, dphi2, 
                                model, data):
    """
    base_pitchを含む4自由度動力学
    """
    # 1. 従属変数を計算（x_baseも考慮）
    z_base = compute_base_height_with_pitch(x_base, pitch_base, phi1, phi2, model, data)
    theta_wheel = compute_wheel_angle_from_noslip_with_pitch(x_base, pitch_base, phi1, phi2, model, data)
    
    # 2. 従属速度を計算
    dz_base, dtheta_wheel = compute_constrained_velocities(
        x_base, pitch_base, phi1, phi2,
        dx_base, dpitch_base, dphi1, dphi2,
        model, data
    )
    
    # 3. 完全な構成ベクトル構築
    q = pin.neutral(model)
    q[0] = x_base
    q[2] = z_base
    if pitch_base != 0:
        r = Rotation.from_euler('y', pitch_base, degrees=False)  # ラジアン入力に修正
        quat_xyzw = r.as_quat()
        q[3] = quat_xyzw[0]  # x
        q[4] = quat_xyzw[1]  # y
        q[5] = quat_xyzw[2]  # z
        q[6] = quat_xyzw[3]  # w
    else:
        q[3] = 0.0  # x
        q[4] = 0.0  # y
        q[5] = 0.0  # z
        q[6] = 1.0  # w
    q[7] = np.cos(phi1)
    q[8] = np.sin(phi1)
    q[9] = np.cos(phi2)
    q[10] = np.sin(phi2)
    q[11] = np.cos(theta_wheel)
    q[12] = np.sin(theta_wheel)
    
    # 4. 完全な速度ベクトル
    dq = np.zeros(model.nv)
    dq[0] = dx_base
    dq[2] = dz_base
    dq[4] = dpitch_base  # y軸周り角速度
    dq[6] = dphi1
    dq[7] = dphi2
    dq[8] = dtheta_wheel
    
    # 5. 動力学計算
    pin.crba(model, data, q)
    pin.computeGeneralizedGravity(model, data, q)
    pin.computeCoriolisMatrix(model, data, q, dq)
    
    M = data.M
    g = data.g
    C = data.C @ dq
    
    # 6. 重力項の補正（シンプルな方法）
    # ベースの並進部分(0-2)の重力を直接補正
    # 物理的には重力はワールド座標で [0, 0, +9.81] であるべき
    total_mass = sum(model.inertias[i].mass for i in range(1, len(model.inertias)))
    g_corrected = g.copy()
    g_corrected[0] = 0.0  # X方向の重力成分を0に
    g_corrected[1] = 0.0  # Y方向の重力成分を0に
    g_corrected[2] = total_mass * 9.81  # Z方向は総重量×重力加速度
    
    # 7. 4×4縮約（独立変数のみ）
    indices = [0, 4, 6, 7]  # x_base, pitch_base, phi1, phi2
    M_red = M[np.ix_(indices, indices)]
    g_red = g_corrected[indices]  # 補正された重力項を使用
    C_red = C[indices]
    
    return M_red, g_red, C_red, q, dq, theta_wheel

def get_joint_positions(q, model, data):
    """各関節の位置を取得してロボットの形状データを返す"""
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    
    positions = []
    # ベース位置
    base_pos = data.oMi[1].translation
    positions.append(base_pos)
    
    # 各関節位置を取得
    for i in range(2, model.njoints):
        joint_pos = data.oMi[i].translation
        positions.append(joint_pos)
    
    return np.array(positions)

def compute_com(model, data, q):
    """全体の質量中心を計算"""
    pin.centerOfMass(model, data, q, False)
    return data.com[0].copy()

def simulate_pitch_dynamics(x_base_init, pitch_base_init, phi1_init, phi2_init, T_sim=3.0, dt=0.02):
    """4自由度pitch考慮動力学シミュレーション"""
    model, data = load_model()
    
    # 独立変数（4自由度）
    x_base = x_base_init
    pitch_base = pitch_base_init
    phi1, phi2 = phi1_init, phi2_init
    
    dx_base = 0.0
    dpitch_base = 0.0
    dphi1, dphi2 = 0.0, 0.0
    
    t_array = np.arange(0, T_sim, dt)
    N = len(t_array)
    
    # 履歴保存用
    state_history = np.zeros((N, 4))  # [x_base, pitch_base, phi1, phi2]
    velocity_history = np.zeros((N, 4))
    theta_wheel_history = np.zeros(N)
    joint_positions_history = []
    com_history = []
    q_history = []
    
    print(f"4自由度pitch考慮シミュレーション")
    print(f"初期: x={x_base_init:.3f}, pitch={np.rad2deg(pitch_base_init):.1f}°, φ1={phi1_init:.3f}, φ2={phi2_init:.3f}")
    
    for i, t in enumerate(t_array):
        # 状態を記録
        state_history[i] = [x_base, pitch_base, phi1, phi2]
        velocity_history[i] = [dx_base, dpitch_base, dphi1, dphi2]
        
        # 動力学計算
        M_red, g_red, C_red, q, dq, theta_wheel = compute_4dof_pitch_dynamics(
            x_base, pitch_base, phi1, phi2, 
            dx_base, dpitch_base, dphi1, dphi2, 
            model, data
        )
        
        theta_wheel_history[i] = theta_wheel
        
        # 各関節位置を記録
        joint_positions = get_joint_positions(q, model, data)
        joint_positions_history.append(joint_positions)
        
        # 質量中心を計算・記録
        com = compute_com(model, data, q)
        com_history.append(com)
        
        # 完全な構成を記録（検証用）
        q_history.append(q.copy())
        
        if i % 20 == 0:
            print(f"t={t:.2f}s: x={x_base:.3f}, pitch={np.rad2deg(pitch_base):.1f}°, "
                  f"φ1={phi1:.3f}, φ2={phi2:.3f}, θ_w={theta_wheel:.3f}")
        
        # 運動方程式（4×4）
        tau = np.zeros(4)  # 無制御
        
        try:
            dd_state = np.linalg.solve(M_red, tau + g_red - C_red)
        except:
            print(f"行列が特異: t={t:.3f}")
            dd_state = np.zeros(4)
        
        # 積分
        dx_base += dd_state[0] * dt
        dpitch_base += dd_state[1] * dt
        dphi1 += dd_state[2] * dt
        dphi2 += dd_state[3] * dt
        
        x_base += dx_base * dt
        pitch_base += dpitch_base * dt
        phi1 += dphi1 * dt
        phi2 += dphi2 * dt
        
        # 発散チェック
        if not (np.isfinite(x_base) and np.isfinite(pitch_base) and 
                np.isfinite(phi1) and np.isfinite(phi2)):
            print(f"発散: t={t:.3f}")
            break
    
    print(f"完了: 最終 x={x_base:.3f}m, pitch={np.rad2deg(pitch_base):.1f}°, "
          f"φ1={phi1:.3f}, φ2={phi2:.3f}")
    
    results = {
        't_array': t_array[:i+1],
        'state_history': state_history[:i+1],
        'velocity_history': velocity_history[:i+1],
        'theta_wheel_history': theta_wheel_history[:i+1],
        'joint_positions_history': joint_positions_history,
        'com_history': com_history,
        'q_history': q_history,
        'x_base_init': x_base_init,
        'pitch_base_init': pitch_base_init,
        'phi1_init': phi1_init,
        'phi2_init': phi2_init
    }
    
    # 検証用データを保存
    import pickle
    filename = f'pitch_results_x{x_base_init:.1f}_p{np.rad2deg(pitch_base_init):.0f}_phi1_{phi1_init:.1f}_phi2_{phi2_init:.1f}.pkl'
    with open(filename, 'wb') as f:
        pickle.dump(results, f)
    print(f"💾 検証用データ保存: {filename}")
    
    return results

def create_pitch_animation(results):
    """pitch考慮のアニメーション"""
    print("🎬 アニメーション作成中...")
    
    t_array = results['t_array']
    joint_positions_history = results['joint_positions_history']
    com_history = results['com_history']
    theta_wheel_history = results['theta_wheel_history']
    state_history = results['state_history']
    
    joint_positions_array = np.array(joint_positions_history)
    com_array = np.array(com_history)
    n_frames = len(joint_positions_array)
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    # 上部: ロボットアニメーション
    all_x = joint_positions_array[:, :, 0].flatten()
    all_z = joint_positions_array[:, :, 2].flatten()
    
    x_min, x_max = np.min(all_x) - 0.2, np.max(all_x) + 0.2
    z_min, z_max = -0.4, 0.4
    
    ax1.set_xlim(x_min, x_max)
    ax1.set_ylim(z_min, z_max)
    ax1.set_xlabel('X Position [m]', fontsize=12)
    ax1.set_ylabel('Z Position [m]', fontsize=12)
    ax1.set_title(f'Robot with Pitch (Initial: pitch={np.rad2deg(results["pitch_base_init"]):.1f}°)', fontsize=14)
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')
    
    # 地面ライン
    ax1.axhline(y=0, color='brown', linewidth=2, alpha=0.7, label='Ground')
    
    # ロボット描画要素
    robot_lines, = ax1.plot([], [], 'b-', linewidth=3, marker='o', markersize=6, label='Robot')
    wheel_circle = plt.Circle((0, 0), WHEEL_RADIUS, fill=False, color='red', linewidth=2)
    ax1.add_patch(wheel_circle)
    
    # タイヤ回転表示用のスポーク
    spoke_line, = ax1.plot([], [], 'r-', linewidth=2, alpha=0.8)
    
    # 質量中心
    com_point, = ax1.plot([], [], 'go', markersize=10, label='Center of Mass', zorder=5)
    com_trajectory_x, com_trajectory_z = [], []
    com_trajectory_line, = ax1.plot([], [], 'g--', alpha=0.5, linewidth=1, label='CoM Trajectory')
    
    # ベース姿勢表示（pitch角度を視覚化）
    base_orientation_line, = ax1.plot([], [], 'k-', linewidth=2, alpha=0.7, label='Base Orientation')
    
    ax1.legend(loc='upper right')
    
    # 下部: 状態変数グラフ
    ax2.set_xlim(0, t_array[-1])
    ax2.set_xlabel('Time [s]', fontsize=12)
    ax2.set_ylabel('States', fontsize=12)
    ax2.grid(True, alpha=0.3)
    
    # 各状態変数の線
    x_line, = ax2.plot([], [], 'b-', label='x_base [m]')
    pitch_line, = ax2.plot([], [], 'r-', label='pitch_base [deg]')
    phi1_line, = ax2.plot([], [], 'g-', label='φ1 [rad]')
    phi2_line, = ax2.plot([], [], 'm-', label='φ2 [rad]')
    
    ax2.legend(loc='upper right')
    
    # 時間表示
    time_text = ax1.text(0.02, 0.98, '', transform=ax1.transAxes, fontsize=12,
                        verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
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
        spoke_end_x = spoke_start_x + WHEEL_RADIUS * np.cos(theta_wheel + np.pi/2)
        spoke_end_z = spoke_start_z + WHEEL_RADIUS * np.sin(theta_wheel + np.pi/2)
        spoke_line.set_data([spoke_start_x, spoke_end_x], [spoke_start_z, spoke_end_z])
        
        # ベース姿勢表示
        base_pos = positions[0]
        pitch_angle = state_history[frame, 1]
        orientation_length = 0.1
        orientation_end_x = base_pos[0] + orientation_length * np.cos(pitch_angle)
        orientation_end_z = base_pos[2] + orientation_length * np.sin(pitch_angle)
        base_orientation_line.set_data([base_pos[0], orientation_end_x], 
                                      [base_pos[2], orientation_end_z])
        
        # 質量中心の更新
        com_pos = com_array[frame]
        com_point.set_data([com_pos[0]], [com_pos[2]])
        
        # 軌跡更新
        com_trajectory_x.append(com_pos[0])
        com_trajectory_z.append(com_pos[2])
        com_trajectory_line.set_data(com_trajectory_x, com_trajectory_z)
        
        # 状態変数グラフ更新
        current_t = t_array[:frame+1]
        x_line.set_data(current_t, state_history[:frame+1, 0])
        pitch_line.set_data(current_t, np.rad2deg(state_history[:frame+1, 1]))
        phi1_line.set_data(current_t, state_history[:frame+1, 2])
        phi2_line.set_data(current_t, state_history[:frame+1, 3])
        
        # 情報表示
        current_time = t_array[frame] if frame < len(t_array) else t_array[-1]
        time_text.set_text(f'Time: {current_time:.2f}s\n'
                          f'Pitch: {np.rad2deg(pitch_angle):.1f}°\n'
                          f'Wheel θ: {theta_wheel:.2f}rad')
        
        return (robot_lines, wheel_circle, spoke_line, com_point, com_trajectory_line, 
                base_orientation_line, x_line, pitch_line, phi1_line, phi2_line, time_text)
    
    # アニメーション作成
    anim = animation.FuncAnimation(fig, animate, frames=n_frames, 
                                 interval=50, blit=True, repeat=True)
    
    # 保存
    filename = f'robot_pitch_x{results["x_base_init"]:.1f}_p{np.rad2deg(results["pitch_base_init"]):.0f}_phi1_{results["phi1_init"]:.1f}_phi2_{results["phi2_init"]:.1f}.gif'
    print(f"💾 アニメーション保存中: {filename}")
    anim.save(filename, writer='pillow', fps=20)
    
    plt.tight_layout()
    plt.show()
    print(f"✅ アニメーション完成: {filename}")

def verify_constraints(results):
    """拘束満足度を検証"""
    print("\n🔍 拘束検証:")
    
    q_history = results['q_history']
    model, data = load_model()
    
    ground_errors = []
    noslip_errors = []
    
    for i, q in enumerate(q_history):
        # 順運動学
        pin.forwardKinematics(model, data, q)
        
        # 接地拘束誤差
        wheel_pos = data.oMi[model.njoints-1].translation
        wheel_bottom_z = wheel_pos[2] - WHEEL_RADIUS
        ground_errors.append(abs(wheel_bottom_z))
        
        # ノンスリップ拘束誤差
        wheel_center_x = wheel_pos[0]
        theta_wheel = np.arctan2(q[12], q[11])
        expected_theta = -wheel_center_x / WHEEL_RADIUS
        noslip_error = abs(theta_wheel - expected_theta)
        noslip_errors.append(noslip_error)
    
    print(f"接地拘束誤差: 最大={max(ground_errors):.2e}, 平均={np.mean(ground_errors):.2e}")
    print(f"ノンスリップ拘束誤差: 最大={max(noslip_errors):.2e}, 平均={np.mean(noslip_errors):.2e}")
    
    # グラフ表示
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    ax1.semilogy(results['t_array'], ground_errors, 'b-')
    ax1.set_ylabel('Ground Constraint Error [m]')
    ax1.set_title('Constraint Satisfaction')
    ax1.grid(True)
    
    ax2.semilogy(results['t_array'], noslip_errors, 'r-')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('No-slip Constraint Error [rad]')
    ax2.grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    print("🚀 base_pitch考慮システムテスト:")
    print()
    
    # テスト2: pitch=10度のみ実行
    print("初期pitch=10度:")
    results = simulate_pitch_dynamics(0.0, np.deg2rad(0.0), 0.3, -0.6, T_sim=1.0)  # 1秒のみ
    create_pitch_animation(results)
    verify_constraints(results)