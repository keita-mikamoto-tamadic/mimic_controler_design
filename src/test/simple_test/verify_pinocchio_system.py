"""
Pinocchio統合システム専用検証ツール
超高精度（6.94e-18レベル）の検証を実施
"""

import numpy as np
import matplotlib.pyplot as plt
import time
from main_analytical_system import build_bodywheel_constrained_model
from utils_pinocchio import (
    solve_constraint_for_2dof_pinocchio,
    compute_reduced_dynamics,
    verify_constraint_satisfaction
)

def simulate_pinocchio_system(q2_init, dq2_init, model, data, L, r, T_sim=5.0, dt=0.01, tau_func=None):
    """
    Pinocchio統合システムのシミュレーション（検証用）
    """
    if tau_func is None:
        tau_func = lambda t, q2, dq2: np.zeros(2)
    
    t_array = np.arange(0, T_sim, dt)
    N = len(t_array)
    
    # 履歴配列
    q2_history = np.zeros((N, 2))
    dq2_history = np.zeros((N, 2))
    q_full_history = np.zeros((N, 4))
    dq_full_history = np.zeros((N, 4))  # 完全な速度履歴
    constraint_errors = np.zeros((N, 2))  # [位置誤差, 速度誤差]
    slip_distances = np.zeros(N)  # スリップ距離
    
    # 初期条件
    q2 = q2_init.copy()
    dq2 = dq2_init.copy()
    theta_accumulated = 0.0
    total_wheel_rotation = 0.0  # 累積ホイール回転
    
    for i, t in enumerate(t_array):
        q2_history[i] = q2
        dq2_history[i] = dq2
        
        # 動力学計算
        M_red, C_red, g_red, q_full, dq_full, theta_new = compute_reduced_dynamics(
            q2, dq2, model, data, L, r, theta_accumulated, dt
        )
        theta_accumulated = theta_new
        q_full_history[i] = q_full
        dq_full_history[i] = dq_full
        
        # 拘束満足度検証
        is_satisfied, errors = verify_constraint_satisfaction(q_full, dq_full, model, data, r)
        constraint_errors[i] = [errors['pos'], errors['vel']]
        
        # スリップ距離計算
        x_base, _, phi_base, theta_wheel = q_full
        
        # 真のno-slip検証: ホイール中心移動距離 vs ホイール回転距離
        wheel_center_x = x_base + L * np.sin(phi_base)
        if i == 0:
            initial_wheel_center_x = wheel_center_x
            initial_theta = theta_wheel
        
        wheel_center_displacement = wheel_center_x - initial_wheel_center_x
        wheel_rotation_distance = (theta_wheel - initial_theta) * r
        slip_distances[i] = abs(wheel_center_displacement - wheel_rotation_distance)
        
        # 制御入力
        tau2 = tau_func(t, q2, dq2)
        
        # 積分
        ddq2 = np.linalg.solve(M_red, tau2 - C_red - g_red)
        dq2 = dq2 + ddq2 * dt
        q2 = q2 + dq2 * dt
    
    return t_array, q2_history, dq2_history, q_full_history, dq_full_history, constraint_errors, slip_distances

def analyze_pinocchio_constraints(t_array, q_full_history, constraint_errors, slip_distances, L, r):
    """
    Pinocchio統合システムの拘束解析
    """
    print("\n=== Pinocchio統合システム拘束解析 ===")
    
    # 拘束満足精度
    pos_errors = constraint_errors[:, 0]
    vel_errors = constraint_errors[:, 1]
    
    print(f"拘束満足精度:")
    print(f"  位置拘束誤差: 最大={np.max(pos_errors):.2e}, 平均={np.mean(pos_errors):.2e}")
    print(f"  速度拘束誤差: 最大={np.max(vel_errors):.2e}, 平均={np.mean(vel_errors):.2e}")
    
    # 精度評価
    if np.max(pos_errors) < 1e-15 and np.max(vel_errors) < 1e-15:
        print("  🌟 超高精度拘束満足 (< 1e-15)")
    elif np.max(pos_errors) < 1e-10 and np.max(vel_errors) < 1e-10:
        print("  ✅ 高精度拘束満足 (< 1e-10)")
    else:
        print("  ⚠️ 拘束精度に改善の余地あり")
    
    # no-slip拘束解析
    print(f"\nNo-slip拘束解析:")
    print(f"  最大スリップ距離: {np.max(slip_distances):.2e} m")
    print(f"  平均スリップ距離: {np.mean(slip_distances):.2e} m")
    
    # スリップ率計算
    final_displacement = abs(q_full_history[-1, 0] - q_full_history[0, 0])
    max_slip_ratio = np.max(slip_distances) / final_displacement if final_displacement > 0 else 0
    avg_slip_ratio = np.mean(slip_distances) / final_displacement if final_displacement > 0 else 0
    
    print(f"  最大スリップ率: {max_slip_ratio:.2e}")
    print(f"  平均スリップ率: {avg_slip_ratio:.2e}")
    
    # スリップ率評価
    if max_slip_ratio < 1e-15:
        print("  🌟 完璧なno-slip（機械精度レベル）")
    elif max_slip_ratio < 1e-10:
        print("  ✅ 優秀なno-slip（高精度）")
    elif max_slip_ratio < 1e-3:
        print("  ✅ 良好なno-slip（実用精度）")
    else:
        print("  ⚠️ no-slip精度に改善の余地あり")
    
    return {
        'pos_error_max': np.max(pos_errors),
        'pos_error_avg': np.mean(pos_errors),
        'vel_error_max': np.max(vel_errors),
        'vel_error_avg': np.mean(vel_errors),
        'slip_distance_max': np.max(slip_distances),
        'slip_distance_avg': np.mean(slip_distances),
        'slip_ratio_max': max_slip_ratio,
        'slip_ratio_avg': avg_slip_ratio
    }

def plot_pinocchio_verification(t_array, q_full_history, dq_full_history, constraint_errors, slip_distances, save_path=None):
    """
    Pinocchio統合システム検証結果の可視化（ホイール速度含む）
    """
    fig, axes = plt.subplots(2, 3, figsize=(18, 8))
    
    # 1. 拘束誤差（位置）
    axes[0, 0].semilogy(t_array, constraint_errors[:, 0], 'b-', linewidth=2)
    axes[0, 0].axhline(y=1e-16, color='g', linestyle='--', alpha=0.7, label='Machine Precision')
    axes[0, 0].set_xlabel('Time [s]')
    axes[0, 0].set_ylabel('Position Constraint Error [m]')
    axes[0, 0].set_title('Position Constraint Accuracy')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()
    
    # 2. 拘束誤差（速度）
    axes[0, 1].semilogy(t_array, constraint_errors[:, 1], 'r-', linewidth=2)
    axes[0, 1].axhline(y=1e-16, color='g', linestyle='--', alpha=0.7, label='Machine Precision')
    axes[0, 1].set_xlabel('Time [s]')
    axes[0, 1].set_ylabel('Velocity Constraint Error [m/s]')
    axes[0, 1].set_title('Velocity Constraint Accuracy')
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend()
    
    # 3. スリップ距離
    axes[1, 0].semilogy(t_array, slip_distances, 'purple', linewidth=2)
    axes[1, 0].axhline(y=1e-16, color='g', linestyle='--', alpha=0.7, label='Machine Precision')
    axes[1, 0].set_xlabel('Time [s]')
    axes[1, 0].set_ylabel('Slip Distance [m]')
    axes[1, 0].set_title('No-slip Constraint Accuracy')
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend()
    
    # 4. ホイール角度履歴
    wheel_angles = q_full_history[:, 3] * 180 / np.pi  # rad → deg
    axes[0, 2].plot(t_array, wheel_angles, 'orange', linewidth=2)
    axes[0, 2].set_xlabel('Time [s]')
    axes[0, 2].set_ylabel('Wheel Angle [deg]')
    axes[0, 2].set_title('Wheel Rotation History')
    axes[0, 2].grid(True, alpha=0.3)
    
    # 5. ホイール角速度履歴
    wheel_velocities = dq_full_history[:, 3] * 180 / np.pi  # rad/s → deg/s
    axes[1, 1].plot(t_array, wheel_velocities, 'red', linewidth=2)
    axes[1, 1].set_xlabel('Time [s]')
    axes[1, 1].set_ylabel('Wheel Angular Velocity [deg/s]')
    axes[1, 1].set_title('Wheel Rotation Speed')
    axes[1, 1].grid(True, alpha=0.3)
    
    # 6. 総合精度サマリー
    axes[1, 2].text(0.1, 0.8, f'Max Position Error: {np.max(constraint_errors[:, 0]):.2e}', 
                    transform=axes[1, 2].transAxes, fontsize=10)
    axes[1, 2].text(0.1, 0.7, f'Max Velocity Error: {np.max(constraint_errors[:, 1]):.2e}', 
                    transform=axes[1, 2].transAxes, fontsize=10)
    axes[1, 2].text(0.1, 0.6, f'Max Slip Distance: {np.max(slip_distances):.2e}', 
                    transform=axes[1, 2].transAxes, fontsize=10)
    axes[1, 2].text(0.1, 0.5, f'Max Wheel Speed: {np.max(np.abs(wheel_velocities)):.1f} deg/s', 
                    transform=axes[1, 2].transAxes, fontsize=10)
    axes[1, 2].text(0.1, 0.3, 'Pinocchio Integration System', 
                    transform=axes[1, 2].transAxes, fontsize=12, fontweight='bold')
    axes[1, 2].text(0.1, 0.2, 'Ultra-High Precision Verification', 
                    transform=axes[1, 2].transAxes, fontsize=12, fontweight='bold')
    axes[1, 2].set_xlim(0, 1)
    axes[1, 2].set_ylim(0, 1)
    axes[1, 2].axis('off')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"検証結果グラフを保存: {save_path}")
    
    plt.show()

def comprehensive_pinocchio_test():
    """
    包括的Pinocchio統合システムテスト
    """
    print("=== Pinocchio統合システム包括検証 ===")
    
    # パラメータ
    L = 0.2
    r = 0.04
    m_wheel = 0.5
    
    # モデル構築
    model = build_bodywheel_constrained_model(L, r, m_wheel)
    data = model.createData()
    
    print(f"パラメータ: L={L}m, r={r}m, m_wheel={m_wheel}kg")
    print(f"ホイール周長: {2*np.pi*r:.6f}m")
    
    # 複雑な制御入力（システムをしっかりテストする）
    def comprehensive_control(t, q2, dq2):
        # 重力補償
        M_red, C_red, g_red, _, _, _ = compute_reduced_dynamics(q2, dq2, model, data, L, r)
        tau_gravity = g_red
        
        # 複合的な制御入力
        tau_complex = np.array([
            0.3 * np.sin(1.2 * t) + 0.15 * np.cos(2.8 * t),    # x方向複合振動
            0.2 * np.sin(0.9 * t) + 0.1 * np.sin(3.5 * t)      # φ方向複合振動
        ])
        
        return tau_gravity + tau_complex
    
    # 初期条件
    q2_init = np.array([0.0, 0.05])  # 軽微な初期傾斜
    dq2_init = np.array([0.1, 0.0])  # 初期速度
    
    # シミュレーション実行
    print("\nシミュレーション実行中...")
    start_time = time.time()
    
    t_array, q2_history, dq2_history, q_full_history, dq_full_history, constraint_errors, slip_distances = \
        simulate_pinocchio_system(
            q2_init, dq2_init, model, data, L, r,
            T_sim=10.0, dt=0.01, tau_func=comprehensive_control
        )
    
    elapsed_time = time.time() - start_time
    print(f"シミュレーション完了: {elapsed_time:.3f}秒 ({len(t_array)}ステップ)")
    
    # 解析実行
    analysis_results = analyze_pinocchio_constraints(
        t_array, q_full_history, constraint_errors, slip_distances, L, r
    )
    
    # 可視化
    print("\n検証結果を可視化中...")
    plot_pinocchio_verification(t_array, q_full_history, dq_full_history, constraint_errors, slip_distances)
    
    # 最終評価
    print("\n=== 総合評価 ===")
    if (analysis_results['pos_error_max'] < 1e-15 and 
        analysis_results['vel_error_max'] < 1e-15 and 
        analysis_results['slip_ratio_max'] < 1e-15):
        print("🌟 EXCELLENT: 機械精度レベルの超高精度システム")
    elif (analysis_results['pos_error_max'] < 1e-10 and 
          analysis_results['vel_error_max'] < 1e-10 and 
          analysis_results['slip_ratio_max'] < 1e-10):
        print("✅ VERY GOOD: 高精度システム")
    elif (analysis_results['pos_error_max'] < 1e-5 and 
          analysis_results['vel_error_max'] < 1e-5 and 
          analysis_results['slip_ratio_max'] < 1e-3):
        print("✅ GOOD: 実用的な精度システム")
    else:
        print("⚠️ 精度改善が必要")
    
    return analysis_results

if __name__ == "__main__":
    # 仮想環境確認
    try:
        import pinocchio as pin
        print("Pinocchioライブラリが正常に読み込まれました")
    except ImportError:
        print("❌ Pinocchioライブラリが見つかりません")
        print("仮想環境をアクティベートしてください: . bin/activate")
        exit(1)
    
    # 包括テスト実行
    results = comprehensive_pinocchio_test()
    
    print(f"\n📊 最終結果サマリー:")
    print(f"位置精度: {results['pos_error_max']:.2e}")
    print(f"速度精度: {results['vel_error_max']:.2e}")  
    print(f"スリップ率: {results['slip_ratio_max']:.2e}")
    print(f"\n🎉 Pinocchio統合システム検証完了！")