"""
関節インデックスのデバッグ用スクリプト
PinocchioのURDFモデルでの関節設定方法を詳細に調査
"""

import pinocchio as pin
import numpy as np
from load_urdf_model import load_mimic_robot_model

def debug_joint_indices():
    """
    関節インデックスとq配列の対応を詳細調査
    """
    print("=== Joint Index Debug ===")
    
    # モデル読み込み
    model, data, joint_names, link_names = load_mimic_robot_model()
    
    print(f"Model: {model.name}")
    print(f"nq (config dimension): {model.nq}")
    print(f"nv (velocity dimension): {model.nv}")
    print(f"njoints: {model.njoints}")
    print()
    
    # 各関節の詳細情報
    print("=== Joint Details ===")
    for i in range(model.njoints):
        joint_name = model.names[i]
        if i > 0:  # universe除外
            joint = model.joints[i]
            idx_q = joint.idx_q
            idx_v = joint.idx_v
            nq = joint.nq
            nv = joint.nv
            
            print(f"Joint {i}: {joint_name}")
            print(f"  Type: {type(joint.shortname())}")
            print(f"  idx_q: {idx_q}, nq: {nq}")
            print(f"  idx_v: {idx_v}, nv: {nv}")
            print(f"  Q range: [{idx_q}:{idx_q+nq}]")
            print()
    
    # 中立設定確認
    print("=== Neutral Configuration ===")
    q_neutral = pin.neutral(model)
    print(f"q_neutral shape: {q_neutral.shape}")
    print(f"q_neutral: {q_neutral}")
    print()
    
    # フローティングベース部分の確認
    print("=== Floating Base Part (first 7 elements) ===")
    print("Expected: [x, y, z, qx, qy, qz, qw]")
    print(f"Actual: {q_neutral[:7]}")
    print()
    
    # 関節角度部分の確認
    print("=== Joint Angles Part (after floating base) ===")
    joint_angles_part = q_neutral[7:]
    print(f"Joint angles: {joint_angles_part}")
    print(f"Length: {len(joint_angles_part)}")
    
    return model, data

def test_manual_joint_setting():
    """
    手動で関節角度を設定してテスト
    """
    print("\n=== Manual Joint Setting Test ===")
    
    model, data = debug_joint_indices()
    
    # 手動で関節角度を設定
    q = pin.neutral(model)
    print(f"Original q: {q}")
    
    # 右脚のupper_link_R_joint (hip_pitch) を52度に設定
    try:
        # 関節IDを取得
        upper_r_id = model.getJointId('upper_link_R_joint')
        print(f"upper_link_R_joint ID: {upper_r_id}")
        
        # その関節のq_idxを取得
        joint = model.joints[upper_r_id]
        q_idx = joint.idx_q
        print(f"upper_link_R_joint q_idx: {q_idx}")
        
        # 52度をラジアンで設定
        angle_rad = np.radians(52)
        q[q_idx] = angle_rad
        print(f"Set q[{q_idx}] = {angle_rad} ({np.degrees(angle_rad)}°)")
        
        # 右脚のlower_link_R_joint (knee) を-104度に設定
        lower_r_id = model.getJointId('lower_link_R_joint')
        joint_lower = model.joints[lower_r_id]
        q_idx_lower = joint_lower.idx_q
        angle_lower_rad = np.radians(-104)
        q[q_idx_lower] = angle_lower_rad
        print(f"Set q[{q_idx_lower}] = {angle_lower_rad} ({np.degrees(angle_lower_rad)}°)")
        
        print(f"Modified q: {q}")
        
        # 順運動学実行
        pin.forwardKinematics(model, data, q)
        
        # 結果確認
        print("\n=== Forward Kinematics Results ===")
        for i in range(1, model.njoints):
            joint_name = model.names[i]
            pos_3d = data.oMi[i].translation
            pos_2d = [pos_3d[0], pos_3d[2]]
            print(f"{joint_name}: [{pos_2d[0]:.4f}, {pos_2d[1]:.4f}]")
            
    except Exception as e:
        print(f"Error in manual setting: {e}")

def compare_neutral_vs_specified():
    """
    ニュートラル設定と指定設定を比較
    """
    print("\n=== Neutral vs Specified Comparison ===")
    
    model, data = debug_joint_indices()
    
    # ニュートラル設定
    q_neutral = pin.neutral(model)
    pin.forwardKinematics(model, data, q_neutral)
    
    print("Neutral positions:")
    neutral_positions = {}
    for i in range(1, model.njoints):
        joint_name = model.names[i]
        pos_3d = data.oMi[i].translation
        pos_2d = [pos_3d[0], pos_3d[2]]
        neutral_positions[joint_name] = pos_2d
        print(f"  {joint_name}: [{pos_2d[0]:.4f}, {pos_2d[1]:.4f}]")
    
    # 指定設定
    q_specified = q_neutral.copy()
    
    # 右脚関節角度設定
    upper_r_id = model.getJointId('upper_link_R_joint')
    lower_r_id = model.getJointId('lower_link_R_joint')
    
    q_specified[model.joints[upper_r_id].idx_q] = np.radians(52)
    q_specified[model.joints[lower_r_id].idx_q] = np.radians(-104)
    
    pin.forwardKinematics(model, data, q_specified)
    
    print("\nSpecified positions:")
    specified_positions = {}
    for i in range(1, model.njoints):
        joint_name = model.names[i]
        pos_3d = data.oMi[i].translation
        pos_2d = [pos_3d[0], pos_3d[2]]
        specified_positions[joint_name] = pos_2d
        print(f"  {joint_name}: [{pos_2d[0]:.4f}, {pos_2d[1]:.4f}]")
    
    # 差分確認
    print("\nDifferences:")
    for joint_name in neutral_positions:
        if joint_name in specified_positions:
            diff = np.array(specified_positions[joint_name]) - np.array(neutral_positions[joint_name])
            dist = np.linalg.norm(diff)
            print(f"  {joint_name}: diff=[{diff[0]:.4f}, {diff[1]:.4f}], dist={dist:.4f}")

if __name__ == "__main__":
    try:
        import pinocchio as pin
        print("Pinocchio library loaded successfully")
    except ImportError:
        print("❌ Pinocchio library not found")
        exit(1)
    
    debug_joint_indices()
    test_manual_joint_setting()
    compare_neutral_vs_specified()