#!/usr/bin/env python3
"""
シンプルな拘束付きシミュレーション
simple_testの手法をURDF single_legモデル（6自由度→5自由度）に適用
フローティングベース（3自由度）＋3関節 - 1拘束（接地）= 5自由度
"""

import pinocchio as pin
import numpy as np
from animation_utils import create_robot_animation
from plotting_utils import plot_simple_results
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


def simulate_simple_dynamics(phi1_init, phi2_init, T_sim=3.0, dt=0.02):
    """5自由度動力学シミュレーション（計算のみ）"""
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
    
    print(f"完了: 最終位置 x={x_base:.3f}m, 高度={base_heights[-1]:.3f}m")
    
    results = {
        't_array': t_array[:i+1],
        'state_history': state_history[:i+1],
        'base_heights': base_heights[:i+1],
        'x_positions': x_positions[:i+1],
        'joint_positions_history': joint_positions_history,
        'com_history': com_history,
        'phi1_init': phi1_init,
        'phi2_init': phi2_init
    }
    
    return results

def simulate_simple(phi1_init, phi2_init, T_sim=3.0, dt=0.02):
    """シンプル拘束シミュレーションのメイン関数（計算と表示を統合）"""
    # 動力学計算を実行
    results = simulate_simple_dynamics(phi1_init, phi2_init, T_sim, dt)
    
    # アニメーション描画
    create_robot_animation(
        results['t_array'], 
        results['joint_positions_history'], 
        results['com_history'], 
        results['phi1_init'], 
        results['phi2_init'], 
        WHEEL_RADIUS
    )
    
    # 静止画グラフ表示
    plot_simple_results(
        results['t_array'],
        results['state_history'],
        results['base_heights'],
        results['x_positions']
    )
    
    return results

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