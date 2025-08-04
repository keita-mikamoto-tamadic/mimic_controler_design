#!/usr/bin/env python3
"""
ノンスリップ拘束付きシミュレーション
タイヤ中心速度からホイール回転角速度を計算するノンスリップ制約実装
simple_testの手法をURDF single_legモデル（6自由度→4自由度）に適用
フローティングベース（3自由度）＋3関節 - 2拘束（接地＋ノンスリップ）= 4自由度

ノンスリップ拘束はタイヤ角度を従属変数として扱い、ベースの自由度は保持する
"""

import pinocchio as pin
import numpy as np
from animation_utils import create_noslip_robot_animation
from plotting_utils import plot_noslip_results
import os

WHEEL_RADIUS = (77.95 / 2) / 1000  # [m]

def load_model():
    """モデル読み込み"""
    base_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(base_dir, "../../../urdf/mimic_v1_single_leg.urdf")
    
    model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
    data = model.createData()
    model.gravity.linear = np.array([0, 0, -9.81])  # 下向き重力（正しい物理）
    
    return model, data

def compute_wheel_angle_from_position(x_base, phi1, phi2, model, data):
    """
    ホイール中心のx位置からタイヤ回転角度を直接計算
    ノンスリップ拘束: タイヤ回転角 = ホイール中心x位置 / 半径
    """
    # 現在の構成でホイール中心位置を計算
    q_temp = pin.neutral(model)
    q_temp[0] = x_base
    q_temp[7] = np.cos(phi1)
    q_temp[8] = np.sin(phi1)
    q_temp[9] = np.cos(phi2)
    q_temp[10] = np.sin(phi2)
    q_temp[11] = 1.0
    q_temp[12] = 0.0
    
    pin.forwardKinematics(model, data, q_temp)
    wheel_center = data.oMi[model.njoints-1].translation
    
    # ノンスリップ条件: ホイール中心x位置がタイヤの転がり距離
    # タイヤ回転角 = -ホイール中心x位置 / 半径 （前進時は負の回転）
    theta_wheel = -wheel_center[0] / WHEEL_RADIUS
    
    return theta_wheel

def compute_wheel_velocity_from_base_motion(dx_base, dphi1, dphi2, phi1, phi2, model, data, wheel_radius):
    """
    ベース運動からタイヤ角速度を計算
    ホイール中心速度からno-slip条件で角速度を導出
    """
    # 仮構成でホイール中心速度を計算
    q_temp = pin.neutral(model)
    q_temp[7] = np.cos(phi1)
    q_temp[8] = np.sin(phi1)
    q_temp[9] = np.cos(phi2)
    q_temp[10] = np.sin(phi2)
    q_temp[11] = 1.0
    q_temp[12] = 0.0
    
    # ヤコビアン計算
    pin.computeJointJacobians(model, data, q_temp)
    J_wheel = pin.getFrameJacobian(model, data, model.njoints-1, pin.ReferenceFrame.WORLD)
    
    # 速度ベクトル構築
    dq_temp = np.zeros(model.nv)
    dq_temp[0] = dx_base    # dx_base
    dq_temp[6] = dphi1      # dphi1
    dq_temp[7] = dphi2      # dphi2
    
    # ホイール中心のx方向速度
    wheel_center_velocity_x = J_wheel[0, :] @ dq_temp
    
    # no-slip条件: v_wheel_center = r * ω_wheel
    dtheta_wheel = wheel_center_velocity_x / wheel_radius
    
    return dtheta_wheel

def compute_base_position_from_constraint(phi1, phi2, model, data):
    """
    接地拘束を満たすベース位置（x=0地点での）を計算
    """
    # 仮構成
    q = pin.neutral(model)
    q[0] = 0.0  # x_base = 0
    q[7] = np.cos(phi1)
    q[8] = np.sin(phi1)
    q[9] = np.cos(phi2)
    q[10] = np.sin(phi2)
    q[11] = 1.0
    q[12] = 0.0
    
    # 順運動学
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    
    # ホイール位置
    wheel_pos = data.oMi[model.njoints-1].translation
    
    # 接地拘束: wheel_bottom = 0
    base_height = WHEEL_RADIUS - wheel_pos[2]
    
    return base_height

def compute_wheel_angular_velocity_from_center_velocity(x_base, dx_base, phi1, phi2, dphi1, dphi2, model, data):
    """
    ホイール中心速度からタイヤ角速度を計算（ノンスリップ条件）
    ホイール中心速度 = ベース速度 + 関節運動による寄与
    タイヤ角速度 = ホイール中心速度 / 半径
    """
    # 現在の構成でヤコビアン計算
    q_temp = pin.neutral(model)
    q_temp[0] = x_base
    q_temp[7] = np.cos(phi1)
    q_temp[8] = np.sin(phi1)
    q_temp[9] = np.cos(phi2)
    q_temp[10] = np.sin(phi2)
    q_temp[11] = 1.0
    q_temp[12] = 0.0
    
    pin.computeJointJacobians(model, data, q_temp)
    J_wheel = pin.getFrameJacobian(model, data, model.njoints-1, pin.ReferenceFrame.WORLD)
    
    # 速度ベクトル構築（ベース速度を含む）
    dq_temp = np.zeros(model.nv)
    dq_temp[0] = dx_base    # ベースx速度
    dq_temp[6] = dphi1      # 股関節角速度
    dq_temp[7] = dphi2      # 膝関節角速度
    
    # ホイール中心のx方向速度（ベース速度と関節速度の両方を考慮）
    wheel_center_velocity_x = J_wheel[0, :] @ dq_temp
    
    # ノンスリップ条件: v_wheel_center = -r * ω_wheel （前進時は負の回転）
    dtheta_wheel = -wheel_center_velocity_x / WHEEL_RADIUS
    
    return dtheta_wheel

def compute_4dof_noslip_dynamics(x_base, phi1, phi2, dx_base, dphi1, dphi2, model, data):
    """
    4自由度ノンスリップ拘束動力学: [x_base, phi1, phi2]システム
    タイヤ角度は従属変数として扱う（位置から直接計算）
    """
    # 1. 接地拘束からベース高度を計算
    base_height = compute_base_position_from_constraint(phi1, phi2, model, data)
    
    # 2. タイヤ角度を位置から直接計算（ノンスリップ拘束）
    theta_wheel = compute_wheel_angle_from_position(x_base, phi1, phi2, model, data)
    
    # 3. ベース垂直速度を計算（接地点速度=0から）
    q_temp = pin.neutral(model)
    q_temp[0] = x_base
    q_temp[2] = base_height
    q_temp[7] = np.cos(phi1)
    q_temp[8] = np.sin(phi1)
    q_temp[9] = np.cos(phi2)
    q_temp[10] = np.sin(phi2)
    q_temp[11] = np.cos(theta_wheel)
    q_temp[12] = np.sin(theta_wheel)
    
    pin.computeJointJacobians(model, data, q_temp)
    J_wheel = pin.getFrameJacobian(model, data, model.njoints-1, pin.ReferenceFrame.WORLD)
    
    # 接地点Z速度=0の拘束からdz_baseを計算
    J_contact_z = J_wheel[2, :]
    if abs(J_contact_z[2]) > 1e-6:
        other_contribution = (J_contact_z[0] * dx_base + 
                            J_contact_z[6] * dphi1 + 
                            J_contact_z[7] * dphi2)
        dz_base = -other_contribution / J_contact_z[2]
    else:
        dz_base = 0.0
    
    # 4. タイヤ角速度を計算（ノンスリップ条件から）
    dtheta_wheel = compute_wheel_angular_velocity_from_center_velocity(
        x_base, dx_base, phi1, phi2, dphi1, dphi2, model, data
    )
    
    # 5. 完全構成を構築
    q = pin.neutral(model)
    q[0] = x_base
    q[2] = base_height
    q[7] = np.cos(phi1)
    q[8] = np.sin(phi1)
    q[9] = np.cos(phi2)
    q[10] = np.sin(phi2)
    # タイヤ角度をquaternionで代入（位置から計算された値）
    q[11] = np.cos(theta_wheel)
    q[12] = np.sin(theta_wheel)
    
    dq = np.zeros(model.nv)
    dq[0] = dx_base
    dq[2] = dz_base
    dq[6] = dphi1
    dq[7] = dphi2
    dq[8] = dtheta_wheel  # ホイール角速度
    
    # 6. 動力学計算
    pin.crba(model, data, q)
    pin.computeGeneralizedGravity(model, data, q)
    pin.computeCoriolisMatrix(model, data, q, dq)
    
    M = data.M
    g = data.g
    C = data.C @ dq
    
    # 7. 3×3縮約: [x_base, phi1, phi2]（独立変数）
    indices = [0, 6, 7]  # x_base, phi1, phi2
    M_red = M[np.ix_(indices, indices)]
    g_red = g[indices]
    C_red = C[indices]
    
    return M_red, g_red, C_red, q, theta_wheel, dtheta_wheel

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


def simulate_noslip_dynamics(phi1_init, phi2_init, T_sim=3.0, dt=0.02):
    """4自由度ノンスリップ動力学シミュレーション（計算のみ）"""
    model, data = load_model()
    
    # 独立変数（3自由度）
    x_base = 0.0  # 初期X位置
    phi1, phi2 = phi1_init, phi2_init
    dx_base = 0.0  # 初期X速度
    dphi1, dphi2 = 0.0, 0.0
    
    t_array = np.arange(0, T_sim, dt)
    N = len(t_array)
    
    state_history = np.zeros((N, 3))  # [x_base, phi1, phi2]
    base_positions = np.zeros(N)
    theta_wheel_history = np.zeros(N)
    joint_positions_history = []
    com_history = []
    q_history = []  # 検証用
    
    print(f"4自由度ノンスリップシミュレーション")
    print(f"初期: x={x_base:.3f}, φ1={phi1_init:.3f}, φ2={phi2_init:.3f}")
    
    for i, t in enumerate(t_array):
        state_history[i] = [x_base, phi1, phi2]
        base_positions[i] = x_base
        
        # 動力学計算
        M_red, g_red, C_red, q, theta_wheel, dtheta_wheel = compute_4dof_noslip_dynamics(
            x_base, phi1, phi2, dx_base, dphi1, dphi2, model, data
        )
        
        theta_wheel_history[i] = theta_wheel
        
        # 各関節位置を記録
        joint_positions = get_joint_positions(q, model, data)
        joint_positions_history.append(joint_positions)
        
        # 質量中心を計算・記録
        com = compute_com(model, data, q)
        com_history.append(com)
        
        # 検証用データ記録
        q_history.append(q.copy())
        
        if i % 20 == 0:
            print(f"t={t:.2f}s: x={x_base:.3f}, φ1={phi1:.3f}, φ2={phi2:.3f}, θ_w={theta_wheel:.3f}")
        
        # 運動方程式（3×3）
        tau = np.zeros(3)  # 無制御
        
        try:
            dd_state = np.linalg.solve(M_red, tau + g_red - C_red)
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
    
    print(f"完了: 最終 x={x_base:.3f}m, φ1={phi1:.3f}, φ2={phi2:.3f}")
    
    results = {
        't_array': t_array[:i+1],
        'state_history': state_history[:i+1],
        'base_positions': base_positions[:i+1],
        'theta_wheel_history': theta_wheel_history[:i+1],
        'joint_positions_history': joint_positions_history,
        'com_history': com_history,
        'q_history': q_history,
        'phi1_init': phi1_init,
        'phi2_init': phi2_init
    }
    
    # 検証用データを保存（オプション）
    import pickle
    filename = f'noslip_results_phi1_{phi1_init:.1f}_phi2_{phi2_init:.1f}.pkl'
    with open(filename, 'wb') as f:
        pickle.dump(results, f)
    print(f"💾 検証用データ保存: {filename}")
    
    return results

def simulate_noslip(phi1_init, phi2_init, T_sim=3.0, dt=0.02):
    """ノンスリップシミュレーションのメイン関数（計算と表示を統合）"""
    # 動力学計算を実行
    results = simulate_noslip_dynamics(phi1_init, phi2_init, T_sim, dt)
    
    # アニメーション描画
    create_noslip_robot_animation(
        results['t_array'], 
        results['joint_positions_history'], 
        results['com_history'],
        results['theta_wheel_history'], 
        results['phi1_init'], 
        results['phi2_init'], 
        WHEEL_RADIUS
    )
    
    # 静止画グラフ表示
    plot_noslip_results(
        results['t_array'],
        results['state_history'],
        results['base_positions'],
        results['theta_wheel_history']
    )
    
    return results

if __name__ == "__main__":
    print("🧪 ノンスリップ拘束システムテスト（4自由度）:")
    print()
    
    # テスト1: 小さな傾斜
    print("1. 小さな傾斜（ノンスリップ）:")
    simulate_noslip(0.3, -0.6)
    
    # テスト2: より大きな傾斜
    print("\n2. 大きな傾斜（ノンスリップ）:")
    simulate_noslip(0.8, -1.2)