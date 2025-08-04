#!/usr/bin/env python3
"""
動力学計算のユーティリティ関数
"""

import pinocchio as pin
import numpy as np
from common_constants import EPSILON, ZERO_THRESHOLD


class SimulationError(Exception):
    """シミュレーション関連のエラー"""
    pass


class ConstraintError(SimulationError):
    """拘束条件関連のエラー"""
    pass


class DivergenceError(SimulationError):
    """数値発散エラー"""
    pass


def compute_dynamics_matrices(model, data, q, dq):
    """
    動力学行列を計算
    
    Args:
        model: Pinocchioモデル
        data: モデルデータ
        q: 構成ベクトル
        dq: 速度ベクトル
        
    Returns:
        M: 質量行列
        g: 重力項
        C: コリオリ・遠心力項
    """
    # 質量行列
    pin.crba(model, data, q)
    M = data.M.copy()
    
    # 重力項
    pin.computeGeneralizedGravity(model, data, q)
    g = data.g.copy()
    
    # コリオリ・遠心力項
    pin.computeCoriolisMatrix(model, data, q, dq)
    C = (data.C @ dq).copy()
    
    return M, g, C


def solve_dynamics(M, tau, g, C):
    """
    運動方程式を解く共通関数
    M * ddq = tau + g - C
    
    Args:
        M: 質量行列
        tau: 制御入力
        g: 重力項
        C: コリオリ・遠心力項
        
    Returns:
        ddq: 加速度ベクトル
        
    Raises:
        DivergenceError: 行列が特異な場合
    """
    try:
        # 運動方程式: M * ddq = tau + g - C
        ddq = np.linalg.solve(M, tau + g - C)
        
        # 発散チェック
        if not np.all(np.isfinite(ddq)):
            raise DivergenceError("加速度が発散しました")
            
        return ddq
        
    except np.linalg.LinAlgError:
        raise DivergenceError("質量行列が特異です")


def extract_reduced_dynamics(M, g, C, indices):
    """
    指定されたインデックスの自由度のみを抽出
    
    Args:
        M: 完全な質量行列
        g: 完全な重力項
        C: 完全なコリオリ項
        indices: 抽出する自由度のインデックスリスト
        
    Returns:
        M_red: 縮約質量行列
        g_red: 縮約重力項
        C_red: 縮約コリオリ項
    """
    M_red = M[np.ix_(indices, indices)]
    g_red = g[indices]
    C_red = C[indices]
    
    return M_red, g_red, C_red


def compute_jacobian(model, data, q, frame_id, reference_frame=pin.ReferenceFrame.WORLD):
    """
    指定フレームのヤコビアンを計算
    
    Args:
        model: Pinocchioモデル
        data: モデルデータ
        q: 構成ベクトル
        frame_id: フレームID
        reference_frame: 参照フレーム
        
    Returns:
        J: 6×n ヤコビアン行列
    """
    pin.computeJointJacobians(model, data, q)
    J = pin.getFrameJacobian(model, data, frame_id, reference_frame)
    return J.copy()


def check_constraint_satisfaction(constraint_value, tolerance=EPSILON):
    """
    拘束条件の満足度をチェック
    
    Args:
        constraint_value: 拘束条件の値（0であるべき）
        tolerance: 許容誤差
        
    Returns:
        bool: 拘束が満たされているか
        
    Raises:
        ConstraintError: 拘束誤差が大きすぎる場合
    """
    error = np.abs(constraint_value)
    
    if error > tolerance:
        raise ConstraintError(f"拘束誤差が許容値を超えています: {error:.6e} > {tolerance:.6e}")
    
    return True


def integrate_state(state, dstate, dt):
    """
    状態を時間積分
    
    Args:
        state: 現在の状態
        dstate: 状態の時間微分
        dt: タイムステップ
        
    Returns:
        new_state: 更新された状態
    """
    new_state = state + dstate * dt
    
    # 数値チェック
    if not np.all(np.isfinite(new_state)):
        raise DivergenceError("状態が発散しました")
    
    return new_state