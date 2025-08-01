"""
真のPinocchio順運動学を使った完全統合アニメーション
z_base導出にPinocchioを真に活用したシステム
注: システムは3自由度（位置拘束1つのみ）
"""

import pinocchio as pin
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from main_analytical_system import build_bodywheel_constrained_model
from utils_pinocchio import (
    compute_z_base_using_pinocchio_fk,
    compute_z_base_velocity,
    solve_constraint_for_2dof_pinocchio,
    compute_reduced_dynamics,
    verify_constraint_satisfaction
)

# 関数の削除（pinocchio_utilsから使用するため）

def simulate_true_fk_system(q2_init, dq2_init, model, data, L, r, T_sim=5.0, dt=0.01, tau_func=None):
    """
    真のPinocchio順運動学シミュレーション
    """
    if tau_func is None:
        tau_func = lambda t, q2, dq2: np.zeros(2)
    
    t_array = np.arange(0, T_sim, dt)
    N = len(t_array)
    
    q2_history = np.zeros((N, 2))
    dq2_history = np.zeros((N, 2))
    q_full_history = np.zeros((N, 4))
    contact_errors = np.zeros(N)  # 接触点高さ誤差
    
    q2 = q2_init.copy()
    dq2 = dq2_init.copy()
    theta_accumulated = 0.0
    
    print("真のPinocchio順運動学シミュレーション開始...")
    start_time = time.time()
    
    for i, t in enumerate(t_array):
        q2_history[i] = q2
        dq2_history[i] = dq2
        
        # 動力学計算
        M_red, C_red, g_red, q_full, dq_full, theta_new = compute_reduced_dynamics(
            q2, dq2, model, data, L, r, theta_accumulated, dt
        )
        theta_accumulated = theta_new
        q_full_history[i] = q_full
        
        # 接触点高さ確認（デバッグ用）
        x_base, z_base, phi_base, theta_wheel = q_full
        base_to_wheel = np.array([L * np.sin(phi_base), 0, -L * np.cos(phi_base)])
        wheel_center = np.array([x_base, 0, z_base]) + base_to_wheel
        contact_pos = wheel_center + np.array([0, 0, -r])
        contact_errors[i] = abs(contact_pos[2])
        
        # 制御入力
        tau2 = tau_func(t, q2, dq2)
        
        # 積分
        ddq2 = np.linalg.solve(M_red, tau2 - C_red - g_red)
        dq2 = dq2 + ddq2 * dt
        q2 = q2 + dq2 * dt
    
    elapsed_time = time.time() - start_time
    print(f"シミュレーション完了: {elapsed_time:.3f}秒 ({N}ステップ)")
    print(f"最大接触点誤差: {np.max(contact_errors):.2e}")
    
    return t_array, q2_history, dq2_history, q_full_history, contact_errors

def animate_true_fk_system(t_array, q_full_history, contact_errors, L, r, save_path=None):
    """
    真のPinocchio順運動学システムのアニメーション
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    # 左側: メインアニメーション
    x_min = np.min(q_full_history[:, 0]) - 0.3
    x_max = np.max(q_full_history[:, 0]) + 0.3
    y_min = -r * 0.2
    y_max = L + r + 0.1
    
    ax1.set_xlim(x_min, x_max)
    ax1.set_ylim(y_min, y_max)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Z [m]')
    ax1.set_title('True Pinocchio FK System')
    
    # 地面
    ground_x = [x_min, x_max]
    ground_y = [0, 0]
    ax1.plot(ground_x, ground_y, 'k-', linewidth=3, label='Ground')
    
    # 描画要素
    body_line, = ax1.plot([], [], 'b-', linewidth=3, label='Body')
    wheel_circle = plt.Circle((0, 0), r, fill=False, color='red', linewidth=2)
    ax1.add_patch(wheel_circle)
    contact_point, = ax1.plot([], [], 'ro', markersize=5, label='Contact Point')
    
    # FK情報テキスト
    info_text = ax1.text(0.02, 0.95, '', transform=ax1.transAxes, fontsize=10,
                        bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.8))
    
    ax1.legend()
    
    # 右側: 接触点誤差グラフ
    ax2.set_xlim(0, t_array[-1])
    ax2.set_ylim(1e-20, 1e-10)
    ax2.set_yscale('log')
    ax2.grid(True, alpha=0.3)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Contact Point Error [m]')
    ax2.set_title('Contact Constraint Accuracy')
    
    error_line, = ax2.plot([], [], 'b-', linewidth=2)
    ax2.axhline(y=1e-16, color='g', linestyle='--', alpha=0.7, label='Machine Precision')
    ax2.legend()
    
    def animate(frame):
        # 現在の状態
        q_full = q_full_history[frame]
        x_base, z_base, phi_base, theta_wheel = q_full
        
        # ボディ座標計算
        body_x = [x_base, x_base + L * np.sin(phi_base)]
        body_z = [z_base, z_base - L * np.cos(phi_base)]
        
        # ホイール中心座標
        wheel_center_x = x_base + L * np.sin(phi_base)
        wheel_center_z = z_base - L * np.cos(phi_base)
        
        # 接触点座標
        contact_x = wheel_center_x
        contact_z = 0.0
        
        # アニメーション更新
        body_line.set_data(body_x, body_z)
        wheel_circle.center = (wheel_center_x, wheel_center_z)
        contact_point.set_data([contact_x], [contact_z])
        
        # 情報テキスト（Pinocchio FKを強調）
        info_text.set_text(
            f'Time: {t_array[frame]:.2f}s\n'
            f'x_base: {x_base:.3f}, φ: {phi_base:.3f}rad\n'
            f'z_base (Pinocchio FK): {z_base:.4f}m\n'
            f'Contact Error: {contact_errors[frame]:.2e}m'
        )
        
        # 誤差グラフ更新
        if frame > 0:
            t_current = t_array[:frame+1]
            errors_current = contact_errors[:frame+1]
            errors_current = np.maximum(errors_current, 1e-20)  # ログスケール用
            error_line.set_data(t_current, errors_current)
        
        return body_line, wheel_circle, contact_point, info_text, error_line
    
    # アニメーション作成
    anim = animation.FuncAnimation(fig, animate, frames=len(t_array),
                                 interval=50, blit=True, repeat=True)
    
    if save_path:
        anim.save(save_path, writer='pillow', fps=20)
        print(f"Animation saved to: {save_path}")
    
    plt.tight_layout()
    plt.show()
    return anim

def demo_true_fk_animation():
    """
    真のPinocchio順運動学アニメーションデモ
    """
    print("=== 真のPinocchio順運動学アニメーション デモ ===")
    
    # パラメータ
    L = 0.2
    r = 0.04
    m_wheel = 0.5
    
    # モデル構築
    model = build_bodywheel_constrained_model(L, r, m_wheel)
    data = model.createData()
    
    # 初期条件
    q2_init = np.array([0.0, 0.05])  # [x=0, φ=0.05rad]
    dq2_init = np.array([0.1, 0.0])  # [dx=0.1, dφ=0]
    
    # 制御入力関数（より興味深い動作を生成）
    def control_func(t, q2, dq2):
        # 重力補償
        M_red, C_red, g_red, _, _, _ = compute_reduced_dynamics(
            q2, dq2, model, data, L, r
        )
        tau_gravity = g_red
        
        # 周期的な外乱
        tau_disturbance = np.array([
            0.05 * np.sin(2 * np.pi * 0.5 * t),    # x方向の力
            0.2 * np.sin(2 * np.pi * 0.3 * t)      # 回転トルク
        ])
        
        return tau_gravity + tau_disturbance
    
    # シミュレーション実行
    t_array, q2_history, dq2_history, q_full_history, contact_errors = \
        simulate_true_fk_system(
            q2_init, dq2_init, model, data, L, r,
            T_sim=10.0, dt=0.01, tau_func=control_func
        )
    
    # 結果表示
    print(f"\n結果:")
    print(f"  平均接触点誤差: {np.mean(contact_errors):.2e}")
    print(f"  最大接触点誤差: {np.max(contact_errors):.2e}")
    print(f"  最終位置: x={q2_history[-1, 0]:.3f}, φ={q2_history[-1, 1]:.3f}")
    
    # アニメーション表示
    print("\nアニメーション表示中...")
    print("黄色のボックスにPinocchio FKによるz_base計算が表示されます！")
    animate_true_fk_system(t_array, q_full_history, contact_errors, L, r)

if __name__ == "__main__":
    # 仮想環境確認
    try:
        import pinocchio as pin
        print("Pinocchioライブラリが正常に読み込まれました")
    except ImportError:
        print("❌ Pinocchioライブラリが見つかりません")
        print("仮想環境をアクティベートしてください: . bin/activate")
        exit(1)
    
    # デモ実行
    demo_true_fk_animation()