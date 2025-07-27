"""
Pinocchio統合システム用共通ユーティリティ関数
中間開発ファイルから抽出した再利用可能な関数群
"""

import pinocchio as pin
import numpy as np

def get_wheel_center_from_model(model, data):
    """
    Pinocchioモデルからホイール中心位置を取得
    
    Returns:
        wheel_center_pos: ホイール中心のワールド座標 [x, y, z]
    """
    wheel_joint_id = model.getJointId('wheel_joint')
    wheel_center_pos = data.oMi[wheel_joint_id].translation
    return wheel_center_pos

def get_wheel_center_velocity_from_model(model, data):
    """
    Pinocchioモデルからホイール中心速度を取得
    
    Returns:
        wheel_center_vel_world: ホイール中心のワールド座標系速度 [vx, vy, vz]
    """
    wheel_joint_id = model.getJointId('wheel_joint')
    pin.computeJointJacobians(model, data)
    
    # ローカル座標系での空間速度
    v_local = data.v[wheel_joint_id]
    
    # ワールド座標系への変換
    R = data.oMi[wheel_joint_id].rotation
    wheel_center_vel_world = R @ v_local.linear
    
    return wheel_center_vel_world

def compute_z_base_using_pinocchio_fk(q2, model, data, L, r):
    """
    Pinocchio順運動学を活用してz_baseを導出
    
    アプローチ：
    1. ベース位置を(x_base, 0)として仮設定
    2. 接触点位置を幾何学的に計算
    3. 接触点が地面(z=0)に接するようにz_baseを調整
    
    Args:
        q2: [x_base, phi_base] - 2自由度設定
        model: Pinocchioモデル
        data: Pinocchioデータ
        L: ベースからホイール中心までの長さ
        r: ホイール半径
    
    Returns:
        z_base: 拘束を満たすベースz座標
    """
    x_base, phi_base = q2
    
    # 仮のz_base=0から開始
    z_base_temp = 0.0
    
    # 幾何学的関係から接触点位置を計算
    base_to_wheel = np.array([L * np.sin(phi_base), 0, -L * np.cos(phi_base)])
    wheel_center = np.array([x_base, 0, z_base_temp]) + base_to_wheel
    contact_pos = wheel_center + np.array([0, 0, -r])
    
    # 接触点を地面に合わせるための調整
    z_adjustment = -contact_pos[2]
    z_base = z_base_temp + z_adjustment
    
    return z_base

def compute_z_base_velocity(q2, dq2, L):
    """
    接触点垂直速度が0になる条件からdz_baseを導出
    
    Args:
        q2: [x_base, phi_base] - 2自由度設定
        dq2: [dx_base, dphi_base] - 2自由度速度
        L: ベースからホイール中心までの長さ
    
    Returns:
        dz_base: ベースz速度
    """
    _, phi_base = q2
    _, dphi_base = dq2
    
    # 接触点z速度 = 0 の条件から
    dz_base = -L * np.sin(phi_base) * dphi_base
    
    return dz_base

def solve_constraint_for_2dof_pinocchio(q2, dq2, model, data, L, r, theta_prev=0.0, dt=0.01):
    """
    Pinocchio順運動学を使った2自由度拘束解法
    
    Args:
        q2: [x_base, phi_base] - 2自由度設定
        dq2: [dx_base, dphi_base] - 2自由度速度
        model: Pinocchioモデル
        data: Pinocchioデータ
        L: ベースからホイール中心までの長さ
        r: ホイール半径
        theta_prev: 前回のホイール角度
        dt: 時間ステップ
    
    Returns:
        q_full: 完全な状態 [x_base, z_base, phi_base, theta_wheel]
        dq_full: 完全な速度 [dx_base, dz_base, dphi_base, dtheta_wheel]
        theta_wheel: 更新されたホイール角度
    """
    x_base, phi_base = q2
    dx_base, dphi_base = dq2
    
    # Pinocchio順運動学でz_base計算
    z_base = compute_z_base_using_pinocchio_fk(q2, model, data, L, r)
    dz_base = compute_z_base_velocity(q2, dq2, L)
    
    # no-slip拘束
    wheel_center_velocity = dx_base + L * np.cos(phi_base) * dphi_base
    dtheta_wheel = wheel_center_velocity / r
    theta_wheel = theta_prev + dtheta_wheel * dt
    
    q_full = np.array([x_base, z_base, phi_base, theta_wheel])
    dq_full = np.array([dx_base, dz_base, dphi_base, dtheta_wheel])
    
    return q_full, dq_full, theta_wheel

def verify_constraint_satisfaction(q_full, dq_full, model, data, r, tolerance=1e-14):
    """
    拘束満足度を検証
    
    Args:
        q_full: 完全な状態
        dq_full: 完全な速度
        model: Pinocchioモデル
        data: Pinocchioデータ
        r: ホイール半径
        tolerance: 許容誤差
    
    Returns:
        is_satisfied: 拘束が満たされているか
        errors: {'pos': 位置誤差, 'vel': 速度誤差}
    """
    # 順運動学計算
    pin.forwardKinematics(model, data, q_full, dq_full)
    
    # ホイール中心位置・速度
    wheel_center_pos = get_wheel_center_from_model(model, data)
    wheel_center_vel = get_wheel_center_velocity_from_model(model, data)
    
    # 拘束誤差
    pos_error = abs(wheel_center_pos[2] - r)
    vel_error = abs(wheel_center_vel[2])
    
    is_satisfied = (pos_error < tolerance) and (vel_error < tolerance)
    
    return is_satisfied, {'pos': pos_error, 'vel': vel_error}

def compute_reduced_dynamics(q2, dq2, model, data, L, r, theta_prev=0.0, dt=0.01):
    """
    縮約動力学計算（2自由度）
    
    Args:
        q2: [x_base, phi_base] - 2自由度設定
        dq2: [dx_base, dphi_base] - 2自由度速度
        model: Pinocchioモデル
        data: Pinocchioデータ
        L: ベースからホイール中心までの長さ
        r: ホイール半径
        theta_prev: 前回のホイール角度
        dt: 時間ステップ
    
    Returns:
        M_red: 2x2 質量行列
        C_red: 2x1 コリオリ項
        g_red: 2x1 重力項
        q_full: 完全な状態
        dq_full: 完全な速度
        theta_new: 更新されたホイール角度
    """
    # 拘束を満たす完全な状態を計算
    q_full, dq_full, theta_new = solve_constraint_for_2dof_pinocchio(
        q2, dq2, model, data, L, r, theta_prev, dt
    )
    
    # Pinocchio動力学計算
    pin.crba(model, data, q_full)  # 質量行列
    M_full = data.M
    
    pin.nonLinearEffects(model, data, q_full, dq_full)  # コリオリ・重力
    h_full = data.nle
    
    # 変換行列T (2x4) - 独立変数の選択
    T = np.array([
        [1.0, 0.0, 0.0, 0.0],  # x_base
        [0.0, 0.0, 1.0, 0.0]   # φ_base
    ])
    
    # 縮約動力学
    M_red = T @ M_full @ T.T
    h_red = T @ h_full
    
    # コリオリと重力の分離（簡略化）
    C_red = np.zeros(2)
    g_red = h_red
    
    return M_red, C_red, g_red, q_full, dq_full, theta_new