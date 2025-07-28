#!/usr/bin/env python3
"""
二足接地拘束システムの設計・実装
single_legで成功したsimple_testアプローチをフルロボットに拡張
"""

import pinocchio as pin
import numpy as np
from basic_loader import FullRobotLoader

WHEEL_RADIUS = (77.95 / 2) / 1000  # [m]

class BipdealConstraintSolver:
    """二足接地拘束ソルバー"""
    
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.wheel_indices = self._find_wheel_indices()
        
        # 独立変数のインデックス（縮約後）
        # nv=12から2拘束を引いて10自由度
        self.free_indices = self._define_free_variables()
        
        print(f"🔧 拘束ソルバー初期化完了")
        print(f"   ホイール数: {len(self.wheel_indices)}")
        print(f"   独立変数数: {len(self.free_indices)}")
    
    def _find_wheel_indices(self):
        """ホイール関節のインデックスを特定"""
        wheel_indices = {}
        for i in range(self.model.njoints):
            joint_name = self.model.names[i]
            if 'wheel' in joint_name.lower():
                wheel_indices[joint_name] = i
        return wheel_indices
    
    def _define_free_variables(self):
        """独立変数の定義
        
        フルロボット12自由度:
        [0-5]: フローティングベース (x, y, z, roll, pitch, yaw)
        [6]: upper_link_L 
        [7]: lower_link_L
        [8]: wheel_L
        [9]: upper_link_R
        [10]: lower_link_R
        [11]: wheel_R
        
        拘束後10自由度:
        wheel_L, wheel_R の2つを従属変数とし、残り10個を独立変数とする
        """
        all_indices = list(range(self.model.nv))
        
        # wheel関節を除外（従属変数）
        dependent_indices = [8, 11]  # wheel_L, wheel_R
        free_indices = [i for i in all_indices if i not in dependent_indices]
        
        print(f"   従属変数インデックス: {dependent_indices}")
        print(f"   独立変数インデックス: {free_indices}")
        
        return free_indices
    
    def compute_wheel_contact_positions(self, q):
        """各ホイールの接地点位置を計算"""
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        
        contact_positions = {}
        for wheel_name, joint_idx in self.wheel_indices.items():
            wheel_center = self.data.oMi[joint_idx].translation
            contact_pos = wheel_center.copy()
            contact_pos[2] -= WHEEL_RADIUS  # 接地点はホイール中心の下
            
            contact_positions[wheel_name] = {
                'center': wheel_center,
                'contact': contact_pos,
                'joint_idx': joint_idx
            }
        
        return contact_positions
    
    def solve_constraint_configuration(self, free_state, method='analytical'):
        """拘束条件を満たす完全構成を解く
        
        Args:
            free_state: 独立変数の値 (10要素)
            method: 'analytical' or 'numerical'
        
        Returns:
            q: 完全構成 (19要素)
        """
        if method == 'analytical':
            return self._solve_analytical(free_state)
        else:
            return self._solve_numerical(free_state)
    
    def _solve_analytical(self, free_state):
        """解析的拘束解法（simple_testアプローチ）"""
        # 初期構成
        q = pin.neutral(self.model)
        
        # 独立変数を設定
        for i, free_idx in enumerate(self.free_indices):
            if free_idx < 6:  # フローティングベース
                q[free_idx] = free_state[i]
            elif free_idx == 6:  # upper_link_L
                q[7] = np.cos(free_state[i])
                q[8] = np.sin(free_state[i])
            elif free_idx == 7:  # lower_link_L  
                q[9] = np.cos(free_state[i])
                q[10] = np.sin(free_state[i])
            elif free_idx == 9:  # upper_link_R
                q[13] = np.cos(free_state[i])
                q[14] = np.sin(free_state[i])
            elif free_idx == 10:  # lower_link_R
                q[15] = np.cos(free_state[i])
                q[16] = np.sin(free_state[i])
        
        # 拘束条件から従属変数（wheel角度）を計算
        # この段階では仮の値を設定（後で接地拘束から計算）
        q[11] = 1.0  # wheel_L cos
        q[12] = 0.0  # wheel_L sin
        q[17] = 1.0  # wheel_R cos  
        q[18] = 0.0  # wheel_R sin
        
        # 接地拘束を満たすようにベース高度を調整
        q = self._adjust_base_height_for_contact(q, free_state)
        
        return q
    
    def _adjust_base_height_for_contact(self, q, free_state):
        """接地拘束を満たすベース高度調整"""
        # 現在のホイール位置計算
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        
        # 両ホイールの接地点高度を取得
        contact_heights = []
        for wheel_name, joint_idx in self.wheel_indices.items():
            wheel_center_z = self.data.oMi[joint_idx].translation[2]
            contact_z = wheel_center_z - WHEEL_RADIUS
            contact_heights.append(contact_z)
        
        # 平均的な接地高度を0にするための調整
        avg_contact_height = np.mean(contact_heights)
        q[2] -= avg_contact_height  # ベースZ位置を調整
        
        return q
    
    def compute_constraint_jacobian(self, q):
        """拘束ヤコビアンの計算"""
        pin.forwardKinematics(self.model, self.data, q)
        pin.computeJointJacobians(self.model, self.data, q)
        
        # 各ホイール接地点のヤコビアン（Z成分のみ）
        constraint_jacobians = []
        
        for wheel_name, joint_idx in self.wheel_indices.items():
            # ホイール接地点のヤコビアン
            J_wheel = pin.getFrameJacobian(self.model, self.data, joint_idx, pin.ReferenceFrame.WORLD)
            J_contact_z = J_wheel[2, :]  # Z方向のみ
            constraint_jacobians.append(J_contact_z)
        
        # 拘束ヤコビアン行列 (2 x nv)
        C_jacobian = np.array(constraint_jacobians)
        
        return C_jacobian
    
    def compute_reduced_dynamics(self, free_state, free_velocity):
        """縮約動力学の計算"""
        # 完全構成を解く
        q = self.solve_constraint_configuration(free_state)
        
        # 完全速度を構築（拘束条件から）
        dq = self._compute_full_velocity(q, free_velocity)
        
        # 動力学行列計算
        pin.crba(self.model, self.data, q)
        pin.computeGeneralizedGravity(self.model, self.data, q)
        pin.computeCoriolisMatrix(self.model, self.data, q, dq)
        
        M = self.data.M
        g = self.data.g
        C = self.data.C @ dq
        
        # 拘束ヤコビアン
        C_jac = self.compute_constraint_jacobian(q)
        
        # 縮約（独立変数のみ）
        M_reduced = M[np.ix_(self.free_indices, self.free_indices)]
        g_reduced = g[self.free_indices]
        C_reduced = C[self.free_indices]
        
        return M_reduced, g_reduced, C_reduced, q, dq
    
    def _compute_full_velocity(self, q, free_velocity):
        """拘束条件から完全速度ベクトルを計算"""
        dq = np.zeros(self.model.nv)
        
        # 独立変数の速度を設定
        for i, free_idx in enumerate(self.free_indices):
            dq[free_idx] = free_velocity[i]
        
        # 拘束条件から従属変数の速度を計算
        # 接地点速度 = 0 の条件を使用
        C_jac = self.compute_constraint_jacobian(q)
        
        # 従属変数インデックス
        dependent_indices = [8, 11]  # wheel_L, wheel_R
        
        # C_jac @ dq = 0 から従属速度を解く
        if C_jac.shape[0] == len(dependent_indices):
            C_dep = C_jac[:, dependent_indices]
            C_free = C_jac[:, self.free_indices]
            
            # C_dep @ dq_dep + C_free @ dq_free = 0
            # dq_dep = -C_dep^(-1) @ C_free @ dq_free
            if np.linalg.det(C_dep) != 0:
                dq_dep = -np.linalg.solve(C_dep, C_free @ free_velocity)
                for i, dep_idx in enumerate(dependent_indices):
                    dq[dep_idx] = dq_dep[i]
        
        return dq

def test_constraint_solver():
    """拘束ソルバーのテスト"""
    print("="*60)
    print("🔧 二足接地拘束ソルバーテスト")
    print("="*60)
    
    # ロボット読み込み
    loader = FullRobotLoader()
    model, data = loader.load_model()
    loader.compute_ground_offset()
    
    # 拘束ソルバー初期化
    solver = BipdealConstraintSolver(model, data)
    
    # テスト用独立変数（10要素）
    free_state = np.array([
        0.0, 0.0, 0.339,  # ベース位置 (x, y, z)
        0.0, 0.0, 0.0,    # ベース姿勢 (roll, pitch, yaw)
        0.3, -0.6,        # 左脚関節角度 (upper, lower)  
        0.3, -0.6         # 右脚関節角度 (upper, lower)
    ])
    
    free_velocity = np.zeros(10)
    
    print(f"🎯 テスト用独立変数:")
    print(f"   free_state: {free_state}")
    
    try:
        # 拘束解法テスト
        q = solver.solve_constraint_configuration(free_state)
        print(f"✅ 拘束解法成功")
        print(f"   完全構成 q: shape={q.shape}")
        
        # 接地点検証
        contact_positions = solver.compute_wheel_contact_positions(q)
        print(f"📍 接地点検証:")
        
        max_error = 0
        for wheel_name, pos_data in contact_positions.items():
            contact_z = pos_data['contact'][2]
            error = abs(contact_z)
            max_error = max(max_error, error)
            status = "✅" if error < 0.01 else "❌"
            print(f"   {wheel_name}: 接地Z={contact_z:8.5f} m {status}")
        
        print(f"   最大接地誤差: {max_error:.6f} m")
        
        # 縮約動力学テスト
        M_red, g_red, C_red, q_full, dq_full = solver.compute_reduced_dynamics(free_state, free_velocity)
        print(f"✅ 縮約動力学計算成功")
        print(f"   M_reduced: {M_red.shape}")
        print(f"   g_reduced: {g_red.shape}")
        print(f"   C_reduced: {C_red.shape}")
        
        # 可視化
        loader.visualize_robot_2d(q, "Constrained Configuration Test")
        
        print(f"\n✅ 拘束ソルバーテスト完了")
        return solver
        
    except Exception as e:
        print(f"❌ エラー: {e}")
        raise

if __name__ == "__main__":
    test_constraint_solver()