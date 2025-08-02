#!/usr/bin/env python3
"""
ノンスリップ拘束精度検証システム
simple_testと同様の高精度検証機能を提供
"""

import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
from noslip_constrained import load_model, WHEEL_RADIUS

def verify_ground_contact_constraint(q, model, data, tolerance=1e-14):
    """
    接地拘束の満足度を検証
    ホイール底面が地面（z=0）に接触しているかを確認
    """
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    
    # ホイール中心位置
    wheel_center_pos = data.oMi[model.njoints-1].translation
    
    # 接触点位置（ホイール底面）
    contact_point_z = wheel_center_pos[2] - WHEEL_RADIUS
    
    # 拘束誤差
    contact_error = abs(contact_point_z - 0.0)
    
    is_satisfied = contact_error < tolerance
    
    return is_satisfied, contact_error

def verify_noslip_constraint(q_history, dt, tolerance=1e-12):
    """
    ノンスリップ拘束の満足度を検証
    ホイール中心の移動距離 vs ホイール回転距離の比較
    """
    if len(q_history) < 2:
        return True, 0.0, np.array([]), np.array([])
    
    q_array = np.array(q_history)
    N = len(q_array)
    
    # 初期状態
    x_base_init = q_array[0, 0]
    theta_wheel_init = np.arctan2(q_array[0, 12], q_array[0, 11])  # quaternionから角度を復元
    
    slip_errors = np.zeros(N)
    wheel_center_displacements = np.zeros(N)
    wheel_rotation_distances = np.zeros(N)
    
    model, data = load_model()
    
    # 角度の連続性を保つための累積角度
    theta_wheel_cumulative = 0.0
    theta_wheel_prev = theta_wheel_init
    
    for i in range(N):
        q = q_array[i]
        
        # 現在のベース位置
        x_base_current = q[0]
        
        # 現在のホイール角度（-π～π）
        theta_wheel_current = np.arctan2(q[12], q[11])
        
        # 角度差を計算（-π～πの範囲で正規化）
        if i > 0:
            angle_diff = theta_wheel_current - theta_wheel_prev
            # 2π跨ぎを修正
            if angle_diff > np.pi:
                angle_diff -= 2 * np.pi
            elif angle_diff < -np.pi:
                angle_diff += 2 * np.pi
            theta_wheel_cumulative += angle_diff
        
        theta_wheel_prev = theta_wheel_current
        
        # ホイール中心位置を正確に計算
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        wheel_center_pos = data.oMi[model.njoints-1].translation
        
        # 初期位置からの変位
        if i == 0:
            wheel_center_x_init = wheel_center_pos[0]
        
        # ホイール中心の移動距離
        wheel_center_displacement = wheel_center_pos[0] - wheel_center_x_init
        wheel_center_displacements[i] = wheel_center_displacement
        
        # ホイール回転による移動距離（累積角度を使用、符号修正）
        wheel_rotation_distance = -theta_wheel_cumulative * WHEEL_RADIUS
        wheel_rotation_distances[i] = wheel_rotation_distance
        
        # スリップ誤差
        slip_error = abs(wheel_center_displacement - wheel_rotation_distance)
        slip_errors[i] = slip_error
    
    max_slip_error = np.max(slip_errors)
    is_satisfied = max_slip_error < tolerance
    
    return is_satisfied, max_slip_error, wheel_center_displacements, wheel_rotation_distances

def verify_constraint_velocity(q, dq, model, data, tolerance=1e-14):
    """
    拘束の速度レベルでの満足度を検証
    接地点の垂直速度とno-slip条件の速度を確認
    """
    pin.forwardKinematics(model, data, q, dq)
    pin.computeJointJacobians(model, data)
    
    # ホイール中心速度
    J_wheel = pin.getFrameJacobian(model, data, model.njoints-1, pin.ReferenceFrame.WORLD)
    wheel_center_velocity = J_wheel @ dq
    
    # 接地点垂直速度（ゼロであるべき）
    contact_velocity_z = wheel_center_velocity[2]
    contact_velocity_error = abs(contact_velocity_z)
    
    # no-slip速度拘束（ホイール中心水平速度 = r * ω_wheel）
    wheel_center_velocity_x = wheel_center_velocity[0]
    wheel_angular_velocity = dq[8]  # ホイール関節の角速度
    expected_velocity_x = WHEEL_RADIUS * wheel_angular_velocity
    
    noslip_velocity_error = abs(wheel_center_velocity_x - expected_velocity_x)
    
    contact_satisfied = contact_velocity_error < tolerance
    noslip_satisfied = noslip_velocity_error < tolerance
    
    return (contact_satisfied, noslip_satisfied), (contact_velocity_error, noslip_velocity_error)

def comprehensive_constraint_verification(t_array, q_history, dq_history=None):
    """
    包括的な拘束検証システム
    """
    print("\n=== ノンスリップシステム拘束検証 ===")
    
    model, data = load_model()
    N = len(q_history)
    
    # 位置レベル検証
    contact_errors = np.zeros(N)
    contact_satisfied_count = 0
    
    for i, q in enumerate(q_history):
        is_satisfied, error = verify_ground_contact_constraint(q, model, data)
        contact_errors[i] = error
        if is_satisfied:
            contact_satisfied_count += 1
    
    # no-slip検証
    dt = t_array[1] - t_array[0] if len(t_array) > 1 else 0.01
    noslip_satisfied, max_slip_error, wheel_displacements, wheel_rotations = verify_noslip_constraint(q_history, dt)
    
    # 速度レベル検証（利用可能な場合）
    velocity_results = None
    if dq_history is not None:
        contact_vel_errors = np.zeros(N)
        noslip_vel_errors = np.zeros(N)
        
        for i, (q, dq) in enumerate(zip(q_history, dq_history)):
            (contact_vel_ok, noslip_vel_ok), (contact_vel_err, noslip_vel_err) = verify_constraint_velocity(q, dq, model, data)
            contact_vel_errors[i] = contact_vel_err
            noslip_vel_errors[i] = noslip_vel_err
        
        velocity_results = {
            'contact_velocity_errors': contact_vel_errors,
            'noslip_velocity_errors': noslip_vel_errors
        }
    
    # 結果表示
    print(f"接地拘束検証:")
    print(f"  満足率: {contact_satisfied_count}/{N} ({100*contact_satisfied_count/N:.1f}%)")
    print(f"  最大誤差: {np.max(contact_errors):.2e}")
    print(f"  平均誤差: {np.mean(contact_errors):.2e}")
    
    print(f"\nノンスリップ拘束検証:")
    print(f"  満足: {'✅' if noslip_satisfied else '❌'}")
    print(f"  最大スリップ誤差: {max_slip_error:.2e}")
    
    if velocity_results:
        print(f"\n速度拘束検証:")
        print(f"  接地点速度誤差: 最大={np.max(velocity_results['contact_velocity_errors']):.2e}")
        print(f"  no-slip速度誤差: 最大={np.max(velocity_results['noslip_velocity_errors']):.2e}")
    
    # 精度評価
    overall_precision = "超高精度" if (np.max(contact_errors) < 1e-15 and max_slip_error < 1e-12) else \
                       "高精度" if (np.max(contact_errors) < 1e-10 and max_slip_error < 1e-8) else \
                       "改善要"
    
    print(f"\n総合評価: {overall_precision}")
    
    return {
        'contact_errors': contact_errors,
        'max_slip_error': max_slip_error,
        'wheel_displacements': wheel_displacements,
        'wheel_rotations': wheel_rotations,
        'velocity_results': velocity_results,
        'overall_precision': overall_precision
    }

def plot_constraint_verification_results(t_array, verification_results):
    """
    拘束検証結果のプロット
    """
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    
    # 接地拘束誤差
    axes[0, 0].semilogy(t_array, verification_results['contact_errors'], 'b-', linewidth=2)
    axes[0, 0].axhline(y=1e-15, color='g', linestyle='--', alpha=0.7, label='Target (1e-15)')
    axes[0, 0].set_xlabel('Time [s]')
    axes[0, 0].set_ylabel('Contact Error [m]')
    axes[0, 0].set_title('Ground Contact Constraint Error')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()
    
    # ホイール移動距離 vs 回転距離
    axes[0, 1].plot(t_array, verification_results['wheel_displacements'], 'b-', linewidth=2, label='Center Displacement')
    axes[0, 1].plot(t_array, verification_results['wheel_rotations'], 'r--', linewidth=2, label='Rotation Distance')
    axes[0, 1].set_xlabel('Time [s]')
    axes[0, 1].set_ylabel('Distance [m]')
    axes[0, 1].set_title('No-Slip Verification: Movement vs Rotation')
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend()
    
    # スリップ誤差
    slip_errors = np.abs(verification_results['wheel_displacements'] - verification_results['wheel_rotations'])
    axes[1, 0].semilogy(t_array, slip_errors, 'r-', linewidth=2)
    axes[1, 0].axhline(y=1e-12, color='g', linestyle='--', alpha=0.7, label='Target (1e-12)')
    axes[1, 0].set_xlabel('Time [s]')
    axes[1, 0].set_ylabel('Slip Error [m]')
    axes[1, 0].set_title('No-Slip Constraint Error')
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend()
    
    # 速度拘束誤差（利用可能な場合）
    if verification_results['velocity_results']:
        vel_results = verification_results['velocity_results']
        axes[1, 1].semilogy(t_array, vel_results['contact_velocity_errors'], 'b-', linewidth=2, label='Contact Velocity')
        axes[1, 1].semilogy(t_array, vel_results['noslip_velocity_errors'], 'r-', linewidth=2, label='No-Slip Velocity')
        axes[1, 1].axhline(y=1e-14, color='g', linestyle='--', alpha=0.7, label='Target (1e-14)')
        axes[1, 1].set_xlabel('Time [s]')
        axes[1, 1].set_ylabel('Velocity Error [m/s]')
        axes[1, 1].set_title('Velocity Constraint Errors')
        axes[1, 1].grid(True, alpha=0.3)
        axes[1, 1].legend()
    else:
        axes[1, 1].text(0.5, 0.5, 'Velocity data\nnot available', 
                       transform=axes[1, 1].transAxes, ha='center', va='center', fontsize=14)
        axes[1, 1].set_title('Velocity Constraint Errors')
    
    plt.tight_layout()
    plt.savefig('noslip_constraint_verification.png', dpi=150, bbox_inches='tight')
    plt.show()

if __name__ == "__main__":
    print("ノンスリップ拘束検証システム テスト")
    
    # サンプルデータでテスト
    from noslip_constrained import simulate_noslip
    
    print("シミュレーション実行中...")
    t_array, state_history, base_positions, theta_wheel_history = simulate_noslip(0.3, -0.6, T_sim=2.0, dt=0.01)
    
    # 検証用にq_historyを再構成（簡略版）
    # 実际の实装では、noslip_constrained.pyから直接q_historyを取得する必要があります
    print("検証機能のインターフェーステストのみ実行...")
    
    model, data = load_model()
    q_sample = pin.neutral(model)
    
    # 接地拘束テスト
    is_satisfied, error = verify_ground_contact_constraint(q_sample, model, data)
    print(f"接地拘束テスト: 満足={is_satisfied}, 誤差={error:.2e}")