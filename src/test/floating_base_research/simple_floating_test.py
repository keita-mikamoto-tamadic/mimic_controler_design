#!/usr/bin/env python3
"""
Pinocchioフローティングベース簡単テスト

基本的なフローティングベース機能のテスト
"""

import pinocchio as pin
import numpy as np
from pathlib import Path

def test_floating_base_basics():
    """フローティングベース基本機能のテスト"""
    print("=" * 50)
    print("🧪 フローティングベース基本テスト")
    print("=" * 50)
    
    # URDFパス
    base_dir = Path(__file__).parent.parent.parent.parent
    urdf_path = base_dir / "urdf" / "mimic_v1_single_leg_floating.urdf"
    
    if not urdf_path.exists():
        print(f"❌ URDFファイルが見つかりません: {urdf_path}")
        return
    
    # 1. モデル読み込み
    print("\n1. モデル読み込み")
    model = pin.buildModelFromUrdf(str(urdf_path))
    data = model.createData()
    model.gravity.linear = np.array([0, 0, -9.81])
    
    print(f"✓ モデル読み込み成功")
    print(f"  構成空間次元 (nq): {model.nq}")
    print(f"  速度空間次元 (nv): {model.nv}")
    print(f"  次元差: {model.nq - model.nv}")
    
    # フローティングベースの確認
    floating_joint = None
    for i, joint in enumerate(model.joints):
        if i > 0:  # universe jointをスキップ
            joint_info = f"  関節[{i}]: {model.names[i]} - {joint}"
            print(joint_info)
            
            # フローティングベース関節の詳細確認
            if hasattr(joint, 'nq') and hasattr(joint, 'nv'):
                print(f"    nq={joint.nq}, nv={joint.nv}")
                if joint.nq == 7 and joint.nv == 6:
                    floating_joint = joint
                    print(f"    ✓ フローティングベース関節発見! (nq=7, nv=6)")
    
    if floating_joint is None:
        print("⚠️  フローティングベース関節が見つかりませんが、nq-nv=4なので存在する可能性があります")
    else:
        print(f"✓ フローティングベース関節確認: {floating_joint}")
    
    # 2. 構成ベクトルテスト
    print("\n2. 構成ベクトルテスト")
    
    # 中立構成
    q_neutral = pin.neutral(model)
    print(f"  中立構成: {q_neutral}")
    print(f"    ベース位置: {q_neutral[0:3]}")
    print(f"    ベース姿勢(quat): {q_neutral[3:7]}")
    print(f"    関節構成: {q_neutral[7:]}")
    
    # カスタム構成
    q_custom = q_neutral.copy()
    q_custom[0:3] = [0.1, 0.2, 0.5]  # ベース位置
    q_custom[3:7] = [0, 0, 0, 1]     # 単位クォータニオン
    print(f"  カスタム構成: {q_custom}")
    
    # 3. 積分テスト
    print("\n3. 積分テスト")
    dq_test = np.zeros(model.nv)
    dq_test[0:3] = [0.1, 0, 0]  # X方向に移動
    dq_test[5] = 0.1            # Z軸周りに回転
    
    dt = 0.01
    print(f"  初期構成: {q_custom[0:7]}")
    print(f"  速度: {dq_test[0:6]}")
    
    try:
        q_integrated = pin.integrate(model, q_custom, dq_test * dt)
        print(f"  積分後構成: {q_integrated[0:7]}")
        print("  ✓ 積分成功")
    except Exception as e:
        print(f"  ❌ 積分失敗: {e}")
        return
    
    # 4. 順運動学テスト
    print("\n4. 順運動学テスト")
    try:
        pin.forwardKinematics(model, data, q_custom)
        pin.updateFramePlacements(model, data)
        
        # ベースフレーム位置
        base_transform = data.oMi[1]  # floating_base joint
        print(f"  ベース変換行列:")
        print(f"    位置: {base_transform.translation}")
        print(f"    回転: {base_transform.rotation}")
        print("  ✓ 順運動学成功")
    except Exception as e:
        print(f"  ❌ 順運動学失敗: {e}")
        return
    
    # 5. 動力学テスト（軽量版）
    print("\n5. 動力学テスト")
    try:
        # 慣性行列
        pin.crba(model, data, q_custom)
        M = data.M
        print(f"  慣性行列サイズ: {M.shape}")
        print(f"  慣性行列条件数: {np.linalg.cond(M):.2e}")
        
        # 重力項
        pin.computeGeneralizedGravity(model, data, q_custom)
        g = data.g
        print(f"  重力項サイズ: {g.shape}")
        print(f"  重力項ノルム: {np.linalg.norm(g):.3f}")
        
        print("  ✓ 動力学計算成功")
    except Exception as e:
        print(f"  ❌ 動力学計算失敗: {e}")
        return
    
    # 6. 簡単な1ステップシミュレーション
    print("\n6. 1ステップシミュレーションテスト")
    try:
        q = q_custom.copy()
        dq = np.zeros(model.nv)
        tau = np.zeros(model.nv)  # 無制御
        dt = 0.001  # 小さなタイムステップ
        
        # 動力学計算
        pin.crba(model, data, q)
        M = data.M
        pin.computeGeneralizedGravity(model, data, q)
        g = data.g
        
        # 運動方程式: M * ddq = tau - g
        ddq = np.linalg.solve(M, tau - g)
        
        # 積分
        dq_new = dq + ddq * dt
        q_new = pin.integrate(model, q, dq_new * dt)
        
        print(f"  初期高度: {q[2]:.6f}m")
        print(f"  1ステップ後高度: {q_new[2]:.6f}m")
        print(f"  高度変化: {q_new[2] - q[2]:+.6f}m")
        print(f"  Z軸加速度: {ddq[2]:.3f}m/s²")
        print("  ✓ 1ステップシミュレーション成功")
        
        # 重力の確認
        expected_accel = -9.81  # 自由落下
        print(f"  期待加速度: {expected_accel:.3f}m/s²")
        print(f"  計算加速度: {ddq[2]:.3f}m/s²")
        
    except Exception as e:
        print(f"  ❌ シミュレーション失敗: {e}")
        return
    
    print("\n" + "=" * 50)
    print("✅ フローティングベース基本テスト完了!")
    print("   主要機能:")
    print("   - URDFからのフローティングベースモデル読み込み")
    print("   - 構成ベクトル・速度ベクトルの次元確認")
    print("   - pin.integrate()による適切な積分")
    print("   - 順運動学・動力学計算")
    print("   - 基本的な1ステップシミュレーション")
    print("=" * 50)

def test_jointmodelfreeflyer_method():
    """JointModelFreeFlyer()を使った方法のテスト"""
    print("\n" + "=" * 50)
    print("🧪 JointModelFreeFlyer()方法テスト")
    print("=" * 50)
    
    # 固定ベースURDFを使用
    base_dir = Path(__file__).parent.parent.parent.parent
    urdf_path = base_dir / "urdf" / "mimic_v1_single_leg.urdf"
    
    if not urdf_path.exists():
        print(f"❌ URDFファイルが見つかりません: {urdf_path}")
        return
    
    try:
        # JointModelFreeFlyer()を明示的に指定
        print("1. JointModelFreeFlyer()でモデル読み込み")
        model = pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
        data = model.createData()
        model.gravity.linear = np.array([0, 0, -9.81])
        
        print(f"✓ モデル読み込み成功")
        print(f"  構成空間次元 (nq): {model.nq}")
        print(f"  速度空間次元 (nv): {model.nv}")
        print(f"  次元差: {model.nq - model.nv}")
        
        # フローティングベースの確認
        print("2. 関節構成確認")
        for i, joint in enumerate(model.joints):
            if i > 0:
                print(f"  関節[{i}]: {model.names[i]} - {type(joint).__name__}")
        
        # 基本テスト
        print("3. 基本機能テスト")
        q = pin.neutral(model)
        q[2] = 0.5  # 高度設定
        
        pin.forwardKinematics(model, data, q)
        pin.crba(model, data, q)
        pin.computeGeneralizedGravity(model, data, q)
        
        print(f"  ベース高度: {q[2]:.3f}m")
        print(f"  重力項Z成分: {data.g[2]:.3f}")
        print("  ✓ JointModelFreeFlyer()方法成功")
        
    except Exception as e:
        print(f"❌ JointModelFreeFlyer()方法失敗: {e}")
        import traceback
        traceback.print_exc()

def main():
    """メイン実行"""
    test_floating_base_basics()
    test_jointmodelfreeflyer_method()

if __name__ == "__main__":
    main()