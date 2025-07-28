#!/usr/bin/env python3
"""
基本URDF読み込みと可視化機能
フルロボットの基本的な読み込み、初期化、簡単な可視化
"""

import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
import os

WHEEL_RADIUS = (77.95 / 2) / 1000  # [m]

class FullRobotLoader:
    """フルロボットの基本読み込みクラス"""
    
    def __init__(self):
        self.model = None
        self.data = None
        self.wheel_info = {}
        self.ground_offset = 0.0
        
    def load_model(self):
        """モデルの読み込みと初期化"""
        base_dir = os.path.dirname(os.path.abspath(__file__))
        urdf_path = os.path.join(base_dir, "../../../urdf/mimic_v1.urdf")
        
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF not found: {urdf_path}")
        
        # フローティングベースでロード
        self.model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
        self.data = self.model.createData()
        
        # 重力設定（前回の重要知見）
        self.model.gravity.linear = np.array([0, 0, +9.81])
        
        print(f"✅ モデル読み込み完了")
        print(f"   nq={self.model.nq}, nv={self.model.nv}")
        print(f"   総質量: {sum([self.model.inertias[i].mass for i in range(1, self.model.njoints)]):.3f} kg")
        
        return self.model, self.data
    
    def find_wheels(self):
        """ホイール関節の特定"""
        wheel_joints = {}
        for i in range(self.model.njoints):
            joint_name = self.model.names[i]
            if 'wheel' in joint_name.lower():
                wheel_joints[joint_name] = {
                    'joint_idx': i,
                    'joint_name': joint_name
                }
        
        print(f"🛞 発見されたホイール: {list(wheel_joints.keys())}")
        return wheel_joints
    
    def compute_ground_offset(self, q=None):
        """接地点が地面（Z=0）になるための高度オフセット計算"""
        if q is None:
            q = pin.neutral(self.model)
        
        # 順運動学
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        
        # ホイール中心位置取得
        wheel_centers = []
        wheel_joints = self.find_wheels()
        
        for wheel_name, wheel_data in wheel_joints.items():
            joint_idx = wheel_data['joint_idx']
            center_pos = self.data.oMi[joint_idx].translation
            wheel_centers.append(center_pos[2])  # Z座標のみ
        
        if wheel_centers:
            lowest_wheel_center = min(wheel_centers)
            self.ground_offset = -(lowest_wheel_center - WHEEL_RADIUS)
            print(f"📏 地面オフセット計算: {self.ground_offset:.4f} m")
            print(f"   最低ホイール中心: {lowest_wheel_center:.4f} m")
            print(f"   接地点高度: {lowest_wheel_center - WHEEL_RADIUS:.4f} m → 0.0000 m")
        
        return self.ground_offset
    
    def get_initial_configuration(self, base_height_offset=0.0):
        """初期構成の生成（地面接地を考慮）"""
        q_init = pin.neutral(self.model)
        
        # ベース高度調整
        q_init[2] = self.ground_offset + base_height_offset
        
        print(f"🎯 初期構成生成:")
        print(f"   ベース位置: ({q_init[0]:.3f}, {q_init[1]:.3f}, {q_init[2]:.3f})")
        print(f"   ベース姿勢: ({q_init[3]:.3f}, {q_init[4]:.3f}, {q_init[5]:.3f}, {q_init[6]:.3f})")
        
        return q_init
    
    def get_joint_positions(self, q):
        """各関節の3D位置を取得"""
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        
        positions = []
        joint_names = []
        
        for i in range(1, self.model.njoints):  # universe以外
            pos = self.data.oMi[i].translation
            positions.append(pos)
            joint_names.append(self.model.names[i])
        
        return np.array(positions), joint_names
    
    def visualize_robot_2d(self, q=None, title="Robot Configuration"):
        """ロボットの2D可視化（x-z平面）"""
        if q is None:
            q = self.get_initial_configuration()
        
        positions, joint_names = self.get_joint_positions(q)
        
        # プロット準備
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # 関節位置プロット
        x_coords = positions[:, 0]
        z_coords = positions[:, 2]
        
        # 関節をポイントで表示
        ax.scatter(x_coords, z_coords, c='red', s=80, alpha=0.8, zorder=5)
        
        # 関節名をラベル表示
        for i, (x, z, name) in enumerate(zip(x_coords, z_coords, joint_names)):
            ax.annotate(name.replace('_joint', ''), (x, z), 
                       xytext=(5, 5), textcoords='offset points', 
                       fontsize=8, alpha=0.7)
        
        # ロボット骨格の描画（簡易版）
        # ベース → 左右上部
        base_idx = joint_names.index('root_joint')
        base_pos = positions[base_idx]
        
        for side in ['L', 'R']:
            try:
                upper_idx = joint_names.index(f'upper_link_{side}_joint')
                lower_idx = joint_names.index(f'lower_link_{side}_joint')
                wheel_idx = joint_names.index(f'wheel_{side}_joint')
                
                # 骨格線描画
                leg_x = [base_pos[0], positions[upper_idx][0], 
                        positions[lower_idx][0], positions[wheel_idx][0]]
                leg_z = [base_pos[2], positions[upper_idx][2], 
                        positions[lower_idx][2], positions[wheel_idx][2]]
                
                color = 'blue' if side == 'L' else 'green'
                ax.plot(leg_x, leg_z, color=color, linewidth=3, alpha=0.7, 
                       label=f'{side} Leg')
                
                # ホイール円描画
                wheel_pos = positions[wheel_idx]
                circle = plt.Circle((wheel_pos[0], wheel_pos[2]), WHEEL_RADIUS, 
                                  fill=False, color=color, linewidth=2, alpha=0.8)
                ax.add_patch(circle)
                
            except ValueError:
                continue
        
        # 地面ライン
        ax.axhline(y=0, color='brown', linewidth=2, alpha=0.7, label='Ground')
        
        # 設定
        ax.set_xlabel('X Position [m]', fontsize=12)
        ax.set_ylabel('Z Position [m]', fontsize=12)
        ax.set_title(title, fontsize=14)
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_aspect('equal')
        
        # 表示範囲調整
        margin = 0.1
        ax.set_xlim(min(x_coords) - margin, max(x_coords) + margin)
        ax.set_ylim(min(z_coords) - margin, max(z_coords) + margin)
        
        plt.tight_layout()
        plt.savefig('full_robot_initial_config.png', dpi=150, bbox_inches='tight')
        plt.show()
        
        print(f"💾 2D可視化保存: full_robot_initial_config.png")
    
    def verify_ground_contact(self, q=None):
        """地面接触の検証"""
        if q is None:
            q = self.get_initial_configuration()
        
        positions, joint_names = self.get_joint_positions(q)
        
        print(f"\n🔍 地面接触検証:")
        
        wheel_contact_errors = []
        for i, (pos, name) in enumerate(zip(positions, joint_names)):
            if 'wheel' in name:
                contact_z = pos[2] - WHEEL_RADIUS
                wheel_contact_errors.append(abs(contact_z))
                status = "✅" if abs(contact_z) < 0.001 else "❌"
                print(f"   {name:20s}: 中心Z={pos[2]:7.4f}, 接地Z={contact_z:7.4f} {status}")
        
        max_error = max(wheel_contact_errors) if wheel_contact_errors else 0
        print(f"   最大接地誤差: {max_error:.6f} m")
        
        return max_error < 0.001

def main():
    """メイン実行"""
    print("="*60)
    print("🤖 フルロボット基本読み込みテスト")
    print("="*60)
    
    try:
        # ローダー初期化
        loader = FullRobotLoader()
        
        # モデル読み込み
        model, data = loader.load_model()
        
        # 地面オフセット計算
        ground_offset = loader.compute_ground_offset()
        
        # 初期構成生成
        q_init = loader.get_initial_configuration()
        
        # 地面接触検証
        contact_ok = loader.verify_ground_contact(q_init)
        
        # 2D可視化
        loader.visualize_robot_2d(q_init, "Full Robot - Initial Configuration")
        
        print(f"\n✅ 基本読み込みテスト完了")
        print(f"   地面接触: {'✅ OK' if contact_ok else '❌ NG'}")
        
        return loader
        
    except Exception as e:
        print(f"❌ エラー: {e}")
        raise

if __name__ == "__main__":
    main()