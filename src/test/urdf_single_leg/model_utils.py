#!/usr/bin/env python3
"""
モデル関連のユーティリティ関数
"""

import pinocchio as pin
import numpy as np
import os
from common_constants import URDF_RELATIVE_PATH, GRAVITY, WHEEL_RADIUS


def load_single_leg_model():
    """
    統一されたモデル読み込み関数
    
    Returns:
        model: Pinocchioモデル
        data: モデルデータ
    """
    base_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(base_dir, URDF_RELATIVE_PATH)
    
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDFファイルが見つかりません: {urdf_path}")
    
    model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
    data = model.createData()
    model.gravity.linear = np.array([0, 0, GRAVITY])
    
    return model, data


def get_wheel_frame_id(model):
    """
    ホイールフレームのIDを取得
    
    Args:
        model: Pinocchioモデル
        
    Returns:
        int: ホイールフレームID（通常は最後のジョイント）
    """
    return model.njoints - 1


def compute_wheel_center(model, data, q):
    """
    ホイール中心位置を取得
    
    Args:
        model: Pinocchioモデル
        data: モデルデータ
        q: 構成ベクトル
        
    Returns:
        np.array: ホイール中心の3D位置
    """
    pin.forwardKinematics(model, data, q)
    wheel_id = get_wheel_frame_id(model)
    return data.oMi[wheel_id].translation.copy()


def compute_wheel_bottom_z(model, data, q):
    """
    ホイール底面のZ座標を計算
    
    Args:
        model: Pinocchioモデル
        data: モデルデータ
        q: 構成ベクトル
        
    Returns:
        float: ホイール底面のZ座標
    """
    wheel_center = compute_wheel_center(model, data, q)
    return wheel_center[2] - WHEEL_RADIUS


def compute_base_position(model, data, q):
    """
    ベース位置を取得
    
    Args:
        model: Pinocchioモデル
        data: モデルデータ
        q: 構成ベクトル
        
    Returns:
        np.array: ベースの3D位置
    """
    pin.forwardKinematics(model, data, q)
    return data.oMi[1].translation.copy()


def build_configuration(x_base, z_base, pitch_base, phi1, phi2, theta_wheel):
    """
    構成ベクトルを構築
    
    Args:
        x_base: ベースX位置
        z_base: ベースZ位置
        pitch_base: ベースピッチ角
        phi1: 股関節角度
        phi2: 膝関節角度
        theta_wheel: ホイール回転角度
        
    Returns:
        np.array: 構成ベクトル
    """
    q = np.zeros(13)  # フリーフライヤー(7) + 3関節(2*3) = 13
    
    # ベース位置
    q[0] = x_base
    q[2] = z_base
    
    # ベース姿勢（クォータニオン）
    if pitch_base != 0:
        from scipy.spatial.transform import Rotation
        r = Rotation.from_euler('y', np.rad2deg(pitch_base), degrees=True)
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
    
    # 関節角度（cos-sin表現）
    q[7] = np.cos(phi1)
    q[8] = np.sin(phi1)
    q[9] = np.cos(phi2)
    q[10] = np.sin(phi2)
    q[11] = np.cos(theta_wheel)
    q[12] = np.sin(theta_wheel)
    
    return q


def get_joint_positions(model, data, q):
    """
    各関節の位置を取得してロボットの形状データを返す
    
    Args:
        model: Pinocchioモデル
        data: モデルデータ
        q: 構成ベクトル
        
    Returns:
        np.array: 各関節の3D位置の配列
    """
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    
    positions = []
    # ベース位置
    base_pos = data.oMi[1].translation
    positions.append(base_pos.copy())
    
    # 各関節位置を取得
    for i in range(2, model.njoints):
        joint_pos = data.oMi[i].translation
        positions.append(joint_pos.copy())
    
    return np.array(positions)


def compute_com(model, data, q):
    """
    全体の質量中心を計算
    
    Args:
        model: Pinocchioモデル
        data: モデルデータ
        q: 構成ベクトル
        
    Returns:
        np.array: 質量中心の3D位置
    """
    pin.centerOfMass(model, data, q, False)
    return data.com[0].copy()