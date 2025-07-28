#!/usr/bin/env python3
"""
Pinocchioフローティングベース実装例

このスクリプトは、Pinocchioでフローティングベースロボットを扱う
完全な実装例を提供する。
"""

import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

def load_floating_base_model(urdf_path, use_floating_joint=True):
    """
    フローティングベースモデルの読み込み
    
    Args:
        urdf_path: URDFファイルのパス
        use_floating_joint: True=JointModelFreeFlyer使用, False=URDF内定義使用
    """
    if use_floating_joint:
        # 方法1: JointModelFreeFlyer()を明示的に指定
        model = pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
        print("✓ JointModelFreeFlyer()を使用してフローティングベースモデルを構築")
    else:
        # 方法2: URDF内のfloating joint定義を使用
        model = pin.buildModelFromUrdf(str(urdf_path))
        print("✓ URDF内のfloating joint定義を使用")
    
    data = model.createData()
    
    # 重力設定
    model.gravity.linear = np.array([0, 0, -9.81])
    
    print(f"モデル情報:")
    print(f"  構成空間次元 (nq): {model.nq}")
    print(f"  速度空間次元 (nv): {model.nv}")
    print(f"  次元差 (nq-nv): {model.nq - model.nv}")
    print(f"  関節数: {model.njoints}")
    print(f"  フレーム数: {model.nframes}")
    
    # 関節名表示
    print("関節一覧:")
    for i, name in enumerate(model.names):
        if i > 0:  # universe関節をスキップ
            joint = model.joints[i]
            print(f"  [{i}] {name}: {joint}")
    
    return model, data

def create_initial_configuration(model, base_position=None, base_orientation=None, joint_angles=None):
    """
    初期構成ベクトルの作成
    
    Args:
        model: Pinocchioモデル
        base_position: ベース位置 [x, y, z] (m)
        base_orientation: ベース姿勢クォータニオン [qx, qy, qz, qw]
        joint_angles: 関節角度リスト (rad)
    """
    # 中立構成から開始
    q = pin.neutral(model)
    
    # デフォルト値設定
    if base_position is None:
        base_position = [0.0, 0.0, 1.0]  # 1m高さ
    if base_orientation is None:
        base_orientation = [0.0, 0.0, 0.0, 1.0]  # 単位クォータニオン
    
    # フローティングベースの次元確認
    if model.nq >= 7:  # フローティングベースが存在
        # ベース位置設定
        q[0:3] = base_position
        
        # ベース姿勢（クォータニオン）設定・正規化
        quat = np.array(base_orientation)
        q[3:7] = quat / np.linalg.norm(quat)
        
        # 関節角度設定
        if joint_angles is not None:
            joint_start_idx = 7
            joint_end_idx = min(joint_start_idx + len(joint_angles), model.nq)
            q[joint_start_idx:joint_end_idx] = joint_angles[:joint_end_idx-joint_start_idx]
    
    print(f"初期構成ベクトル作成:")
    print(f"  ベース位置: {q[0:3]}")
    print(f"  ベース姿勢: {q[3:7]} (クォータニオン)")
    if model.nq > 7:
        print(f"  関節角度: {q[7:]}")
    
    return q

def compute_floating_base_dynamics(q, dq, tau, model, data):
    """
    フローティングベース動力学計算
    
    Args:
        q: 構成ベクトル
        dq: 速度ベクトル
        tau: 制御入力（トルク）
        model: Pinocchioモデル
        data: Pinocchioデータ
        
    Returns:
        ddq: 加速度ベクトル
        M: 慣性行列
        g: 重力項
        C: コリオリ・遠心力項
    """
    # 順運動学
    pin.forwardKinematics(model, data, q, dq)
    pin.updateFramePlacements(model, data)
    
    # 慣性行列（CRBA: Composite Rigid Body Algorithm）
    pin.crba(model, data, q)
    M = data.M.copy()
    
    # 重力項
    pin.computeGeneralizedGravity(model, data, q)
    g = data.g.copy()
    
    # コリオリ・遠心力項
    pin.computeCoriolisMatrix(model, data, q, dq)
    C = data.C @ dq
    
    # 運動方程式: M * ddq = tau - g - C
    try:
        ddq = np.linalg.solve(M, tau - g - C)
    except np.linalg.LinAlgError:
        print("警告: 慣性行列が特異 - 零加速度を使用")
        ddq = np.zeros(model.nv)
    
    return ddq, M, g, C

def integrate_configuration(q, dq, dt, model):
    """
    構成ベクトルの数値積分（Pinocchioの標準積分関数を使用）
    
    Args:
        q: 現在の構成ベクトル
        dq: 速度ベクトル
        dt: 時間刻み
        model: Pinocchioモデル
        
    Returns:
        q_new: 更新された構成ベクトル
    """
    # Pinocchioの標準積分関数を使用
    # これはフローティングベース、クォータニオン、特殊関節すべてを適切に処理
    q_new = pin.integrate(model, q, dq * dt)
    
    return q_new

def simulate_floating_base_robot(model, data, q0, dq0, T_sim=5.0, dt=0.01, control_func=None):
    """
    フローティングベースロボットシミュレーション
    
    Args:
        model: Pinocchioモデル
        data: Pinocchioデータ
        q0: 初期構成ベクトル
        dq0: 初期速度ベクトル
        T_sim: シミュレーション時間 [s]
        dt: 時間刻み [s]
        control_func: 制御関数 func(t, q, dq, model, data) -> tau
        
    Returns:
        t_array: 時間配列
        q_history: 構成履歴
        dq_history: 速度履歴
        energy_history: エネルギー履歴
    """
    t_array = np.arange(0, T_sim, dt)
    N = len(t_array)
    
    # 履歴配列
    q_history = np.zeros((N, model.nq))
    dq_history = np.zeros((N, model.nv))
    energy_history = np.zeros(N)
    
    q, dq = q0.copy(), dq0.copy()
    
    print(f"\\nシミュレーション開始:")
    print(f"  時間: {T_sim}s, ステップ数: {N}, dt: {dt}s")
    print(f"  初期ベース位置: {q[0:3]}")
    
    for i, t in enumerate(t_array):
        # 状態記録
        q_history[i] = q
        dq_history[i] = dq
        
        # 制御入力
        if control_func is not None:
            tau = control_func(t, q, dq, model, data)
        else:
            tau = np.zeros(model.nv)  # 無制御
        
        # 動力学計算
        ddq, M, g, C = compute_floating_base_dynamics(q, dq, tau, model, data)
        
        # エネルギー計算
        kinetic = 0.5 * dq.T @ M @ dq
        # 重力ポテンシャル（簡易）
        if model.nq >= 3:
            potential = q[2] * 9.81 * 10.0  # 概算質量10kg
        else:
            potential = 0.0
        energy_history[i] = kinetic + potential
        
        # 進捗表示
        if i % (N // 10) == 0:
            print(f"  t={t:.2f}s: 高度={q[2]:.3f}m, エネルギー={energy_history[i]:.1f}J")
        
        # 数値積分
        dq = dq + ddq * dt
        q = integrate_configuration(q, dq, dt, model)
        
        # 安全チェック
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(dq)):
            print(f"警告: t={t:.3f}で数値発散")
            break
    
    print(f"シミュレーション完了!")
    if model.nq >= 3:
        print(f"  初期高度: {q_history[0, 2]:.3f}m")
        print(f"  最終高度: {q_history[-1, 2]:.3f}m")
        print(f"  落下距離: {q_history[0, 2] - q_history[-1, 2]:+.3f}m")
    
    return t_array, q_history, dq_history, energy_history

def plot_simulation_results(t_array, q_history, dq_history, energy_history, model):
    """シミュレーション結果のプロット"""
    if model.nq < 3:
        print("フローティングベースでないため、プロットをスキップ")
        return
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('Floating Base Robot Simulation Results', fontsize=16)
    
    # ベース位置
    ax = axes[0, 0]
    ax.plot(t_array, q_history[:, 0], label='X', linewidth=2)
    ax.plot(t_array, q_history[:, 1], label='Y', linewidth=2)
    ax.plot(t_array, q_history[:, 2], label='Z', linewidth=2)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Base Position [m]')
    ax.set_title('Base Position')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # ベース速度
    ax = axes[0, 1]
    ax.plot(t_array, dq_history[:, 0], label='Vx', linewidth=2)
    ax.plot(t_array, dq_history[:, 1], label='Vy', linewidth=2)
    ax.plot(t_array, dq_history[:, 2], label='Vz', linewidth=2)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Base Velocity [m/s]')
    ax.set_title('Base Velocity')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # ベース姿勢（クォータニオン）
    ax = axes[1, 0]
    ax.plot(t_array, q_history[:, 3], label='qx', linewidth=2)
    ax.plot(t_array, q_history[:, 4], label='qy', linewidth=2)
    ax.plot(t_array, q_history[:, 5], label='qz', linewidth=2)
    ax.plot(t_array, q_history[:, 6], label='qw', linewidth=2)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Base Orientation (quaternion)')
    ax.set_title('Base Orientation')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # エネルギー
    ax = axes[1, 1]
    ax.plot(t_array, energy_history, 'r-', linewidth=2)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Total Energy [J]')
    ax.set_title('Energy')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

def simple_pd_control(t, q, dq, model, data, target_q=None, kp=100.0, kd=10.0):
    """
    簡単なPD制御例
    
    Args:
        t: 現在時刻
        q, dq: 現在の状態
        model, data: Pinocchioモデル・データ
        target_q: 目標構成（関節角度のみ）
        kp, kd: PD制御ゲイン
    """
    tau = np.zeros(model.nv)
    
    if target_q is not None and model.nv > 6:
        # 関節のみPD制御（フローティングベースは制御しない）
        joint_start = 6  # 速度空間でのインデックス
        
        # 目標角度との誤差
        q_error = target_q - q[7:]  # 構成空間での関節角度誤差
        dq_joints = dq[joint_start:]  # 関節角速度
        
        # PD制御トルク
        tau_joints = kp * q_error - kd * dq_joints
        tau[joint_start:] = tau_joints
    
    return tau

def main():
    """メイン実行関数"""
    print("=" * 60)
    print("🚀 Pinocchio フローティングベース実装例")
    print("=" * 60)
    
    # URDFパス（既存のフローティングベースURDFを使用）
    base_dir = Path(__file__).parent.parent.parent.parent
    urdf_path = base_dir / "urdf" / "mimic_v1_single_leg_floating.urdf"
    
    if not urdf_path.exists():
        print(f"❌ URDFファイルが見つかりません: {urdf_path}")
        print("代替として、プロジェクト内の別のURDFを使用するか、")
        print("mimic_v1_single_leg_floating.urdfを作成してください。")
        return
    
    try:
        # 1. モデル読み込み
        print("\\n1. モデル読み込み")
        model, data = load_floating_base_model(urdf_path, use_floating_joint=False)
        
        # 2. 初期構成設定
        print("\\n2. 初期構成設定")
        initial_joint_angles = [0.4, -0.7, 0.0] if model.nq > 10 else None
        q0 = create_initial_configuration(
            model, 
            base_position=[0.0, 0.0, 0.8],
            base_orientation=[0.0, 0.0, 0.0, 1.0],
            joint_angles=initial_joint_angles
        )
        dq0 = np.zeros(model.nv)
        
        # 3. 制御関数定義（オプション）
        def control_func(t, q, dq, model, data):
            target_angles = [0.2, -0.5, 0.0] if model.nq > 10 else None
            return simple_pd_control(t, q, dq, model, data, target_angles)
        
        # 4. シミュレーション実行
        print("\\n3. シミュレーション実行")
        results = simulate_floating_base_robot(
            model, data, q0, dq0, 
            T_sim=5.0, dt=0.01,
            control_func=None  # 無制御 or control_func
        )
        
        # 5. 結果表示
        print("\\n4. 結果プロット")
        plot_simulation_results(*results, model)
        
        print("\\n" + "=" * 60)
        print("✅ フローティングベース実装例完了!")
        print("   重要なポイント:")
        print("   - 構成空間nq vs 速度空間nv の次元差")
        print("   - クォータニオンの適切な積分")
        print("   - フローティングベース制約の扱い")
        print("=" * 60)
        
    except Exception as e:
        print(f"❌ エラーが発生しました: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()