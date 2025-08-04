#!/usr/bin/env python3
"""
URDFシングルレッグシミュレーション共通定数
"""

# 物理定数
WHEEL_RADIUS = (77.95 / 2) / 1000  # [m] ホイール半径
GRAVITY = -9.81  # [m/s^2] 重力加速度（下向き）

# 数値計算パラメータ
EPSILON = 1e-6  # 数値誤差許容値
ZERO_THRESHOLD = 1e-12  # ゼロ判定閾値
DEFAULT_DT = 0.02  # デフォルトタイムステップ [s]
DEFAULT_T_SIM = 3.0  # デフォルトシミュレーション時間 [s]

# 収束判定
MAX_ITERATIONS = 50  # 反復計算の最大回数
CONVERGENCE_TOL = 1e-10  # 収束判定許容誤差

# デフォルト初期値
DEFAULT_PHI1 = 0.3  # 股関節初期角度 [rad]
DEFAULT_PHI2 = -0.6  # 膝関節初期角度 [rad]
DEFAULT_PITCH = 0.0  # ベースピッチ初期角度 [rad]

# アニメーション設定
ANIMATION_FPS = 20  # アニメーションフレームレート
ANIMATION_DPI = 100  # アニメーション解像度

# ファイルパス
URDF_RELATIVE_PATH = "../../../urdf/mimic_v1_single_leg.urdf"