#!/usr/bin/env python3
"""
フルロボット構造解析
mimic_v1.urdfの詳細構造を解析し、joint/linkの情報を出力
"""

import pinocchio as pin
import numpy as np
import os

WHEEL_RADIUS = (77.95 / 2) / 1000  # [m]

def load_full_robot_model():
    """フルロボットモデルの読み込み"""
    base_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(base_dir, "../../../urdf/mimic_v1.urdf")
    
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF not found: {urdf_path}")
    
    # フローティングベースでロード
    model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
    data = model.createData()
    
    # 重力設定（前回知見を適用）
    model.gravity.linear = np.array([0, 0, +9.81])
    
    return model, data

def analyze_robot_structure(model, data):
    """ロボット構造の詳細解析"""
    print("="*60)
    print("🤖 フルロボット構造解析")
    print("="*60)
    
    # 基本情報
    print(f"📊 基本情報:")
    print(f"   総joint数: {model.njoints}")
    print(f"   総frame数: {model.nframes}")
    print(f"   構成空間次元 (nq): {model.nq}")
    print(f"   速度空間次元 (nv): {model.nv}")
    print(f"   総質量: {sum([model.inertias[i].mass for i in range(1, model.njoints)]):.3f} kg")
    print()
    
    # Joint構造
    print(f"🔗 Joint構造:")
    for i in range(model.njoints):
        joint_name = model.names[i]
        if i > 0:  # universe以外
            parent_id = model.parents[i]
            parent_name = model.names[parent_id]
            joint_type = str(model.joints[i])
            mass = model.inertias[i].mass
            print(f"   {i:2d}: {joint_name:20s} (親: {parent_name:15s}) [{joint_type[:20]}] 質量: {mass:.3f}kg")
        else:
            print(f"   {i:2d}: {joint_name:20s} (root)")
    print()
    
    # Frame情報（end-effector等）
    print(f"📍 Frame情報:")
    for i in range(model.nframes):
        frame_name = model.frames[i].name
        parent_joint = model.frames[i].parent
        parent_name = model.names[parent_joint] if parent_joint < len(model.names) else "unknown"
        frame_type = str(model.frames[i].type)
        print(f"   {i:2d}: {frame_name:20s} (親Joint: {parent_name:15s}) [{frame_type}]")
    print()
    
    return model, data

def analyze_neutral_configuration(model, data):
    """中立構成での各関節位置解析"""
    print("🎯 中立構成解析:")
    
    # 中立構成
    q_neutral = pin.neutral(model)
    print(f"   中立構成 q_neutral: {q_neutral.shape} = {q_neutral}")
    print()
    
    # 順運動学計算
    pin.forwardKinematics(model, data, q_neutral)
    pin.updateFramePlacements(model, data)
    
    # 各Joint位置
    print("📍 各Joint位置 (中立構成):")
    for i in range(1, model.njoints):  # universe以外
        joint_name = model.names[i]
        pos = data.oMi[i].translation
        print(f"   {joint_name:20s}: ({pos[0]:8.4f}, {pos[1]:8.4f}, {pos[2]:8.4f})")
    
    print()
    
    # 重要なFrame位置
    print("🎯 重要Frame位置:")
    for i in range(model.nframes):
        frame_name = model.frames[i].name
        if 'wheel' in frame_name.lower():
            pos = data.oMf[i].translation
            print(f"   {frame_name:20s}: ({pos[0]:8.4f}, {pos[1]:8.4f}, {pos[2]:8.4f})")
    
    return q_neutral

def analyze_wheel_geometry(model, data, q_neutral):
    """ホイール幾何形状と接地点解析"""
    print("\n" + "="*60)
    print("🛞 ホイール・接地点解析")
    print("="*60)
    
    # ホイール中心位置（中立構成）
    pin.forwardKinematics(model, data, q_neutral)
    pin.updateFramePlacements(model, data)
    
    wheel_joints = []
    for i in range(model.njoints):
        if 'wheel' in model.names[i].lower():
            wheel_joints.append((i, model.names[i]))
    
    print(f"ホイール半径: {WHEEL_RADIUS:.6f} m")
    print(f"発見されたホイールJoint: {len(wheel_joints)}個")
    print()
    
    wheel_info = {}
    for joint_idx, joint_name in wheel_joints:
        wheel_center = data.oMi[joint_idx].translation
        wheel_bottom = wheel_center.copy()
        wheel_bottom[2] -= WHEEL_RADIUS  # Z方向にホイール半径分下げる
        
        print(f"📍 {joint_name}:")
        print(f"   中心位置: ({wheel_center[0]:8.4f}, {wheel_center[1]:8.4f}, {wheel_center[2]:8.4f})")
        print(f"   接地点: ({wheel_bottom[0]:8.4f}, {wheel_bottom[1]:8.4f}, {wheel_bottom[2]:8.4f})")
        print(f"   地面からの高さ: {wheel_bottom[2]:8.4f} m")
        print()
        
        wheel_info[joint_name] = {
            'joint_idx': joint_idx,
            'center': wheel_center,
            'bottom': wheel_bottom,
            'ground_height': wheel_bottom[2]
        }
    
    return wheel_info

def check_ruby_joints(model):
    """RUBY関節の確認"""
    print("\n" + "="*60)
    print("💎 RUBY関節変換確認")
    print("="*60)
    
    continuous_joints = []
    for i in range(model.njoints):
        joint_type_str = str(model.joints[i])
        if 'continuous' in joint_type_str.lower() or 'ruby' in joint_type_str.lower():
            continuous_joints.append((i, model.names[i], joint_type_str))
    
    print(f"RUBY関節数: {len(continuous_joints)}")
    for joint_idx, joint_name, joint_type in continuous_joints:
        print(f"   {joint_name:20s}: {joint_type}")
    
    print(f"\nnq (構成空間次元): {model.nq}")
    print(f"nv (速度空間次元): {model.nv}")
    print(f"期待値: nq=19 (floating_base:7 + RUBY_joints:6*2), nv=15 (floating_base:6 + joints:6)")
    
    return continuous_joints

def analyze_mass_distribution(model):
    """質量分布解析"""
    print("\n" + "="*60)
    print("⚖️  質量分布解析")
    print("="*60)
    
    total_mass = 0
    mass_breakdown = {}
    
    for i in range(1, model.njoints):  # universe以外
        link_name = model.names[i]
        mass = model.inertias[i].mass
        com = model.inertias[i].lever  # 重心位置
        
        total_mass += mass
        mass_breakdown[link_name] = mass
        
        print(f"{link_name:20s}: {mass:6.3f} kg, COM: ({com[0]:7.4f}, {com[1]:7.4f}, {com[2]:7.4f})")
    
    print(f"\n総質量: {total_mass:.3f} kg")
    
    # 左右対称性確認
    left_mass = sum([mass for name, mass in mass_breakdown.items() if '_L' in name])
    right_mass = sum([mass for name, mass in mass_breakdown.items() if '_R' in name])
    base_mass = mass_breakdown.get('base_link', 0)
    
    print(f"\n左脚質量: {left_mass:.3f} kg")
    print(f"右脚質量: {right_mass:.3f} kg")
    print(f"ベース質量: {base_mass:.3f} kg")
    print(f"左右対称性: {'✅' if abs(left_mass - right_mass) < 0.001 else '❌'}")
    
    return mass_breakdown

def main():
    """メイン解析実行"""
    try:
        # モデル読み込み
        model, data = load_full_robot_model()
        
        # 構造解析
        analyze_robot_structure(model, data)
        
        # 中立構成解析
        q_neutral = analyze_neutral_configuration(model, data)
        
        # ホイール解析
        wheel_info = analyze_wheel_geometry(model, data, q_neutral)
        
        # RUBY関節確認
        ruby_joints = check_ruby_joints(model)
        
        # 質量分布
        mass_breakdown = analyze_mass_distribution(model)
        
        print("\n" + "="*60)
        print("✅ フルロボット構造解析完了")
        print("="*60)
        
        return {
            'model': model,
            'data': data,
            'q_neutral': q_neutral,
            'wheel_info': wheel_info,
            'ruby_joints': ruby_joints,
            'mass_breakdown': mass_breakdown
        }
        
    except Exception as e:
        print(f"❌ エラー: {e}")
        raise

if __name__ == "__main__":
    main()