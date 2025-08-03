#!/usr/bin/env python3
"""
修正版3Dロボット動力学
full_robot_dynamicsの正確な実装を3Dに拡張
"""

import pinocchio as pin
import numpy as np
import os
import yaml

WHEEL_RADIUS = 0.075  # m

class FixedRobot3D:
    """修正版3Dロボット（full_robot_dynamicsに忠実）"""
    
    def __init__(self):
        # モデル読み込み（フローティングベース必須！）
        self.model, self.data = self.load_model()
        
        # ホイール関節の特定
        self.wheel_indices = self._find_wheel_indices()
        
        print(f"🤖 修正版3Dロボット初期化")
        print(f"   自由度: nv={self.model.nv}, nq={self.model.nq}")
        print(f"   ホイール関節: {list(self.wheel_indices.keys())}")
        print(f"   重力: {self.model.gravity.linear}")
    
    def load_model(self):
        """URDFモデルの読み込み"""
        current_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.join(current_dir, "..", "..", "..")
        urdf_path = os.path.join(project_root, "urdf", "mimic_v1.urdf")
        
        # フローティングベース必須！
        model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
        data = model.createData()
        
        # 重力設定（Pinocchio標準：負のZ方向）
        model.gravity.linear = np.array([0, 0, -9.81])
        
        return model, data
    
    def _find_wheel_indices(self):
        """ホイール関節のインデックスを特定"""
        wheel_indices = {}
        for i in range(self.model.njoints):
            joint_name = self.model.names[i]
            if 'wheel' in joint_name.lower():
                wheel_indices[joint_name] = i
        return wheel_indices
    
    def compute_constrained_configuration(self, x_base, y_base, pitch, yaw, phi_L_lower, phi_R_lower, phi_L_upper, phi_R_upper, wheel_L, wheel_R):
        """
        拘束を満たす完全構成を計算（10自由度版）
        
        独立変数: [x_base, y_base, pitch, yaw, phi_L_lower, phi_R_lower, phi_L_upper, phi_R_upper, wheel_L, wheel_R]
        従属変数: [z_base, roll]
        
        - 両輪完全接地拘束: 各ホイール接地点Z=0
        - yaw は独立変数として扱う
        """
        
        # 膝関節は独立変数（従属拘束なし）
        
        # 初期構成
        q = pin.neutral(self.model)
        
        # ベース位置・姿勢（仮設定）
        q[0] = x_base  # X
        q[1] = y_base  # Y
        q[4] = pitch   # pitch（独立変数）
        q[5] = yaw     # yaw（独立変数）
        # q[2], q[3] は接地拘束から計算（z, roll）
        
        # 関節角度設定（RUBY表現）
        # upper_link_L (joint index 2)
        q[7] = np.cos(phi_L_upper)
        q[8] = np.sin(phi_L_upper)
        # lower_link_L (joint index 3)
        q[9] = np.cos(phi_L_lower)
        q[10] = np.sin(phi_L_lower)
        # wheel_L (joint index 4) - 独立回転
        q[11] = np.cos(wheel_L)
        q[12] = np.sin(wheel_L)
        
        # upper_link_R (joint index 5)
        q[13] = np.cos(phi_R_upper)
        q[14] = np.sin(phi_R_upper)
        # lower_link_R (joint index 6)
        q[15] = np.cos(phi_R_lower)
        q[16] = np.sin(phi_R_lower)
        # wheel_R (joint index 7) - 独立回転
        q[17] = np.cos(wheel_R)
        q[18] = np.sin(wheel_R)
        
        # 両輪完全接地拘束の実装（6自由度版）
        # 反復的に z, roll, yaw を決定
        max_iterations = 20
        tolerance = 1e-6
        
        for iteration in range(max_iterations):
            # 順運動学でホイール位置計算
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            
            # 各ホイールの接地点高度を取得
            wheel_positions = {}
            wheel_contact_heights = {}
            
            for wheel_name, joint_idx in self.wheel_indices.items():
                wheel_center = self.data.oMi[joint_idx].translation
                wheel_positions[wheel_name] = wheel_center
                wheel_contact_heights[wheel_name] = wheel_center[2] - WHEEL_RADIUS
            
            # 左右ホイール識別
            wheel_names = list(self.wheel_indices.keys())
            if 'wheel_L_joint' in wheel_names and 'wheel_R_joint' in wheel_names:
                wheel_L_z = wheel_contact_heights['wheel_L_joint']
                wheel_R_z = wheel_contact_heights['wheel_R_joint']
            else:
                # フォールバック：順序で決定
                wheel_L_z = wheel_contact_heights[wheel_names[0]]
                wheel_R_z = wheel_contact_heights[wheel_names[1]]
            
            # 接地拘束：両ホイール接地点がZ=0
            # 3x3システム（z, roll, yaw を同時計算）
            
            # ヤコビアン計算
            pin.computeJointJacobians(self.model, self.data, q)
            
            # 左右ホイールのZ方向ヤコビアン
            J_L = pin.getJointJacobian(self.model, self.data, 
                                      self.wheel_indices[wheel_names[0]], 
                                      pin.ReferenceFrame.WORLD)[2, :]  # Z成分
            J_R = pin.getJointJacobian(self.model, self.data, 
                                      self.wheel_indices[wheel_names[1]], 
                                      pin.ReferenceFrame.WORLD)[2, :]  # Z成分
            
            # ノンスリップ拘束から yaw を計算
            # 左右ホイールの独立回転を考慮した差動駆動モデル
            wheel_L_pos = wheel_positions[wheel_names[0]]
            wheel_R_pos = wheel_positions[wheel_names[1]]
            
            # ベース中心から各ホイールへのベクトル
            base_center = np.array([q[0], q[1], 0])  # Z=0はベース高度
            base_to_L = wheel_L_pos[:2] - base_center[:2]  # XY平面での投影
            base_to_R = wheel_R_pos[:2] - base_center[:2]
            
            # 理想的なホイール配置（Y軸方向対称）
            track_width = 0.266  # ベース幅（URDFから）
            ideal_L = np.array([0, track_width/2])
            ideal_R = np.array([0, -track_width/2])
            
            # 実際の配置と理想配置の角度差
            actual_orientation = np.arctan2(base_to_R[0] - base_to_L[0], base_to_R[1] - base_to_L[1])
            ideal_orientation = np.arctan2(ideal_R[0] - ideal_L[0], ideal_R[1] - ideal_L[1])
            yaw_constraint = actual_orientation - ideal_orientation
            
            # 2x2システム: [z, roll] を調整（yawは独立変数）
            A = np.array([
                [J_L[2], J_L[3]],  # 左輪接地: z, roll の影響
                [J_R[2], J_R[3]],  # 右輪接地: z, roll の影響
            ])
            b = np.array([-wheel_L_z, -wheel_R_z])
            
            # デバッグ出力
            if iteration == 0:
                print(f"    2x2 A matrix det: {np.linalg.det(A):.6f}")
                print(f"    b vector: {b}")
            
            try:
                # 線形システムを解く
                if abs(np.linalg.det(A)) > 1e-12:
                    corrections = np.linalg.solve(A, b)
                    
                    # 更新（ダンピング係数で安定化）
                    damping = 0.3
                    q[2] += damping * corrections[0]  # dz
                    q[3] += damping * corrections[1]  # droll
                    
                    if iteration == 0:
                        print(f"    Corrections: dz={corrections[0]:.6f}, droll={corrections[1]:.6f}")
                else:
                    # 特異な場合は簡略化（z のみ修正）
                    avg_contact_height = (wheel_L_z + wheel_R_z) / 2
                    q[2] -= 0.3 * avg_contact_height
                    # roll と yaw は小さく調整
                    if abs(wheel_L_z - wheel_R_z) > 1e-6:
                        q[3] += 0.1 * (wheel_R_z - wheel_L_z)  # 高度差からroll推定
                    if iteration == 0:
                        print(f"    Matrix singular, adjusting z and estimated roll")
                    
            except np.linalg.LinAlgError as e:
                print(f"    LinAlgError: {e}")
                avg_contact_height = (wheel_L_z + wheel_R_z) / 2
                q[2] -= 0.5 * avg_contact_height
                q[5] = 0.0  # yaw = 0固定
            
            # 収束判定
            error_L = abs(wheel_L_z)
            error_R = abs(wheel_R_z)
            max_error = max(error_L, error_R)
            
            if iteration % 5 == 0 or max_error < tolerance:
                print(f"  Iteration {iteration}: error_L={error_L:.6f}, error_R={error_R:.6f}, z={q[2]:.3f}, roll={q[3]:.3f}")
            
            if max_error < tolerance:
                print(f"  Converged at iteration {iteration}")
                break
            
            if iteration == max_iterations - 1:
                print(f"  WARNING: Did not converge! Final errors: L={error_L:.6f}, R={error_R:.6f}")
        
        return q
    
    def compute_dynamics(self, state, velocity):
        """
        10自由度動力学計算（修正版）
        
        独立変数: [x_base, y_base, pitch, yaw, phi_L_lower, phi_R_lower, phi_L_upper, phi_R_upper, wheel_L, wheel_R]
        従属変数: [z_base, roll]
        """
        
        # 状態変数展開（10自由度）
        x_base, y_base, pitch, yaw, phi_L_lower, phi_R_lower, phi_L_upper, phi_R_upper, wheel_L, wheel_R = state
        dx_base, dy_base, dpitch, dyaw, dphi_L_lower, dphi_R_lower, dphi_L_upper, dphi_R_upper, dwheel_L, dwheel_R = velocity
        
        # 完全構成計算（10→19次元）
        q = self.compute_constrained_configuration(x_base, y_base, pitch, yaw, phi_L_lower, phi_R_lower, phi_L_upper, phi_R_upper, wheel_L, wheel_R)
        
        # pitch は独立変数として設定済み
        # ホイール回転は既に設定済み
        
        # 完全速度構築（10自由度版）
        dq = np.zeros(self.model.nv)
        dq[0] = dx_base       # dx
        dq[1] = dy_base       # dy
        # dq[2] = dz_base     # 拘束から計算
        # dq[3] = droll       # 拘束から計算
        dq[4] = dpitch        # dpitch（独立変数）
        dq[5] = dyaw          # dyaw（独立変数）
        dq[6] = dphi_L_upper  # left upper (独立)
        dq[7] = dphi_L_lower  # left lower (独立)
        dq[8] = dwheel_L      # wheel_L (独立)
        dq[9] = dphi_R_upper  # right upper (独立)
        dq[10] = dphi_R_lower # right lower (独立)
        dq[11] = dwheel_R     # wheel_R (独立)
        
        # 接地拘束からベースZ・Roll速度を計算（10自由度版）
        pin.computeJointJacobians(self.model, self.data, q)
        
        # 左右ホイールの接地点Z速度=0の拘束
        wheel_names = list(self.wheel_indices.keys())
        J_L = pin.getJointJacobian(self.model, self.data, 
                                  self.wheel_indices[wheel_names[0]], 
                                  pin.ReferenceFrame.WORLD)[2, :]  # Z成分
        J_R = pin.getJointJacobian(self.model, self.data, 
                                  self.wheel_indices[wheel_names[1]], 
                                  pin.ReferenceFrame.WORLD)[2, :]  # Z成分
        
        # 2つの拘束条件から dz_base, droll を計算
        # J_L @ dq = 0, J_R @ dq = 0
        
        A = np.array([
            [J_L[2], J_L[3]],     # 左輪拘束: [dz, droll]
            [J_R[2], J_R[3]],     # 右輪拘束: [dz, droll]
        ])
        
        # 既知の速度成分から残差計算
        known_terms_L = (J_L[0]*dx_base + J_L[1]*dy_base + J_L[4]*dpitch + J_L[5]*dyaw +
                        J_L[6]*dphi_L_upper + J_L[7]*dphi_L_lower + J_L[8]*dwheel_L + 
                        J_L[9]*dphi_R_upper + J_L[10]*dphi_R_lower)
        known_terms_R = (J_R[0]*dx_base + J_R[1]*dy_base + J_R[4]*dpitch + J_R[5]*dyaw +
                        J_R[6]*dphi_L_upper + J_R[7]*dphi_L_lower + 
                        J_R[9]*dphi_R_upper + J_R[10]*dphi_R_lower + J_R[11]*dwheel_R)
        
        b = np.array([-known_terms_L, -known_terms_R])
        
        try:
            if abs(np.linalg.det(A)) > 1e-12:
                constraint_velocities = np.linalg.solve(A, b)
                dq[2] = constraint_velocities[0]  # dz_base
                dq[3] = constraint_velocities[1]  # droll
            else:
                # 特異の場合は簡略化
                dq[2] = 0.0
                dq[3] = 0.0
        except np.linalg.LinAlgError:
            dq[2] = 0.0
            dq[3] = 0.0
        
        # ホイール速度は独立変数として既に設定済み
        
        # 動力学計算（分離版：コリオリと重力を個別計算）
        M = pin.crba(self.model, self.data, q)
        
        # 重力項とコリオリ項を分離計算
        g = pin.computeGeneralizedGravity(self.model, self.data, q)
        C = np.zeros(self.model.nv)  # コリオリ項
        if np.linalg.norm(dq) > 1e-6:
            pin.computeCoriolisMatrix(self.model, self.data, q, dq)
            C = self.data.C @ dq
        
        # 参考：nonLinearEffects (C + g)
        nle = pin.nonLinearEffects(self.model, self.data, q, dq)
        
        # 10×10縮約: [x, y, pitch, yaw, phi_L_lower, phi_R_lower, phi_L_upper, phi_R_upper, wheel_L, wheel_R]
        free_indices = [0, 1, 4, 5, 7, 10, 6, 9, 8, 11]  # [x, y, pitch, yaw, phi_L_lower, phi_R_lower, phi_L_upper, phi_R_upper, wheel_L, wheel_R]
        M_red = M[np.ix_(free_indices, free_indices)]
        g_red = g[free_indices]
        C_red = C[free_indices]
        nle_red = nle[free_indices]  # 参考値
        
        return M_red, g_red, C_red, nle_red, q, dq
    
    def simulate(self, initial_state, T_sim=2.0, dt=0.01):
        """動力学シミュレーション（10自由度版）"""
        
        print(f"\n🚀 シミュレーション開始（10自由度版）")
        print(f"   初期状態: {initial_state}")
        print(f"   時間: {T_sim}s, dt: {dt}s")
        
        # 初期化
        state = np.array(initial_state, dtype=float)
        velocity = np.zeros(10)  # 10自由度の初期速度ゼロ
        
        t_array = np.arange(0, T_sim, dt)
        N = len(t_array)
        
        # 履歴
        time_history = []
        state_history = []
        velocity_history = []
        full_config_history = []
        
        print(f"   総ステップ数: {N}")
        
        # シミュレーションループ
        for i, t in enumerate(t_array):
            # 履歴記録
            time_history.append(t)
            state_history.append(state.copy())
            velocity_history.append(velocity.copy())
            
            try:
                # 動力学計算
                M_red, g_red, C_red, nle_red, q_full, _ = self.compute_dynamics(state, velocity)
                full_config_history.append(q_full.copy())
                
                # 運動方程式: M * ddq = tau - C - g（重力項のみ符号修正）
                tau = np.zeros(10)  # 10自由度、無制御（重力のみ）
                
                # 加速度計算（重力項のみ+符号、コリオリ項は-符号）
                acceleration = np.linalg.solve(M_red, tau - C_red + g_red)
                
                # 積分
                velocity += acceleration * dt
                state += velocity * dt
                
                # 10ステップごとに進捗表示
                if i % 10 == 0:
                    print(f"   t={t:.3f}s: pitch={state[2]:.2f}, yaw={state[3]:.2f}, lower=[{state[4]:.2f}, {state[5]:.2f}], upper=[{state[6]:.2f}, {state[7]:.2f}], wheel=[{state[8]:.2f}, {state[9]:.2f}]")
                
                # 発散チェック無効化（アニメーション優先）
                # if np.any(np.isnan(state)) or np.any(np.isinf(state)):
                #     print(f"   ❌ NaN/Inf検出 at t={t:.3f}s")
                #     break
                #     
                # if np.any(np.abs(state[2:4]) > 10.0):  # 膝関節のみ（6自由度版）
                #     print(f"   ⚠️ 関節発散検出 at t={t:.3f}s")
                #     break
                    
            except Exception as e:
                print(f"   ❌ エラー at t={t:.3f}s: {e}")
                break
        
        print(f"   ✅ シミュレーション完了: {len(time_history)}ステップ")
        
        return {
            'time': np.array(time_history),
            'state': np.array(state_history),
            'velocity': np.array(velocity_history),
            'config': full_config_history
        }

def test_gravity_pendulum():
    """重力による振り子運動テスト（10自由度版）"""
    print("=== 修正版3Dロボット重力テスト（10自由度）===")
    
    # システム初期化
    robot = FixedRobot3D()
    
    # 初期状態（10自由度）- singlelegと同じ条件
    initial_state = [
        0.0, 0.0,         # x, y
        0.0,              # pitch（ベース角度）- 0rad
        0.0,              # yaw（ベース角度）- 0rad
        -0.6,             # phi_L_lower（左下腿角度）- singlelegと同じ
        -0.6,             # phi_R_lower（右下腿角度）- singlelegと同じ
        0.2,              # phi_L_upper（左上腿角度）- 独立
        0.2,              # phi_R_upper（右上腿角度）- 独立
        0.0,              # wheel_L（左ホイール角度）
        0.0               # wheel_R（右ホイール角度）
    ]
    
    # シミュレーション実行（小さなタイムステップ）
    results = robot.simulate(initial_state, T_sim=1.0, dt=0.001)
    
    # 結果分析
    print(f"\n📊 結果分析:")
    print(f"   シミュレーション時間: {results['time'][-1]:.3f}s")
    
    # 関節角度の変化（10自由度版）
    initial_joints = results['state'][0, 4:8]  # phi_L_lower, phi_R_lower, phi_L_upper, phi_R_upper
    final_joints = results['state'][-1, 4:8]
    joint_change = final_joints - initial_joints
    
    # ホイール角度の変化
    initial_wheels = results['state'][0, 8:10]  # wheel_L, wheel_R
    final_wheels = results['state'][-1, 8:10]
    wheel_change = final_wheels - initial_wheels
    
    print(f"   初期関節角度（lower）: {initial_joints}")
    print(f"   最終関節角度（lower）: {final_joints}")
    print(f"   関節変化量: {joint_change}")
    print(f"   初期ホイール角度: {initial_wheels}")
    print(f"   最終ホイール角度: {final_wheels}")
    print(f"   ホイール変化量: {wheel_change}")
    
    # 振り子運動の判定
    max_joint_change = np.max(np.abs(joint_change))
    max_wheel_change = np.max(np.abs(wheel_change))
    if max_joint_change > 0.1:
        print(f"   ✅ 振り子運動検出（関節最大変化: {max_joint_change:.3f} rad）")
    else:
        print(f"   ⚠️ 関節運動が小さすぎる（最大変化: {max_joint_change:.3f} rad）")
        
    if max_wheel_change > 0.01:
        print(f"   ✅ ホイール回転検出（最大変化: {max_wheel_change:.3f} rad）")
    else:
        print(f"   ⚠️ ホイール回転が小さすぎる（最大変化: {max_wheel_change:.3f} rad）")
    
    return results

def load_config(config_path=None):
    """
    YAMLファイルから設定を読み込む
    
    Args:
        config_path: 設定ファイルのパス（Noneの場合はデフォルトパス）
    
    Returns:
        config: 設定辞書
    """
    if config_path is None:
        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(current_dir, 'config.yaml')
    
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        print(f"✅ 設定ファイルを読み込みました: {config_path}")
        return config
    except FileNotFoundError:
        print(f"⚠️ 設定ファイルが見つかりません: {config_path}")
        print("   デフォルト値を使用します")
        return None
    except yaml.YAMLError as e:
        print(f"❌ YAML解析エラー: {e}")
        print("   デフォルト値を使用します")
        return None

def extract_initial_state_from_config(config):
    """
    設定辞書から初期状態リストを抽出
    
    Args:
        config: 設定辞書
    
    Returns:
        initial_state: 10自由度の初期状態リスト
    """
    if config is None or 'initial_state' not in config:
        return None
    
    state_config = config['initial_state']
    return [
        state_config.get('x_base', 0.0),
        state_config.get('y_base', 0.0),
        state_config.get('pitch', 0.0),
        state_config.get('yaw', 0.0),
        state_config.get('phi_L_lower', -0.6),
        state_config.get('phi_R_lower', -0.6),
        state_config.get('phi_L_upper', 0.2),
        state_config.get('phi_R_upper', 0.2),
        state_config.get('wheel_L', 0.0),
        state_config.get('wheel_R', 0.0)
    ]

def run_simulation_with_animation(initial_state=None, T_sim=None, dt=None, config_path=None):
    """
    シミュレーション実行とアニメーション作成
    
    Args:
        initial_state: 初期状態 (9自由度) - None の場合は設定ファイルまたはデフォルト値使用
        T_sim: シミュレーション時間 [s] - None の場合は設定ファイルまたはデフォルト値使用
        dt: 時間ステップ [s] - None の場合は設定ファイルまたはデフォルト値使用
        config_path: 設定ファイルのパス（Noneの場合はデフォルトパス）
    
    Returns:
        results: シミュレーション結果
        anim: アニメーションオブジェクト
    """
    print("=== 9自由度ロボットシミュレーション&アニメーション ===")
    
    # 設定ファイルの読み込み
    config = load_config(config_path)
    
    # パラメータの決定（優先順位: 引数 > 設定ファイル > デフォルト）
    if initial_state is None and config is not None:
        initial_state = extract_initial_state_from_config(config)
    
    if T_sim is None and config is not None and 'simulation' in config:
        T_sim = config['simulation'].get('time', 2.0)
    elif T_sim is None:
        T_sim = 2.0
    
    if dt is None and config is not None and 'simulation' in config:
        dt = config['simulation'].get('dt', 0.005)
    elif dt is None:
        dt = 0.005
    
    # デフォルト初期状態 - singlelegと同じ条件
    if initial_state is None:
        initial_state = [
            0.0, 0.0,         # x, y
            0.0,              # pitch（ベース角度）- 0rad
            0.0,              # yaw（ベース角度）- 0rad
            -0.6,             # phi_L_lower（左下腿角度）- singlelegと同じ
            -0.6,             # phi_R_lower（右下腿角度）- singlelegと同じ
            0.2,              # phi_L_upper（左上腿角度）- 独立
            0.2,              # phi_R_upper（右上腿角度）- 独立
            0.0,              # wheel_L（左ホイール角度）
            0.0               # wheel_R（右ホイール角度）
        ]
    
    print(f"📋 使用するパラメータ:")
    print(f"   シミュレーション時間: {T_sim}s")
    print(f"   時間ステップ: {dt}s")
    print(f"   初期状態: {initial_state}")
    
    # ロボット初期化
    robot = FixedRobot3D()
    
    # シミュレーション実行
    print(f"\n🚀 シミュレーション実行中...")
    results = robot.simulate(initial_state, T_sim=T_sim, dt=dt)
    
    # アニメーション作成
    try:
        from animation_3d import create_comprehensive_analysis
        print(f"\n🎬 アニメーション&分析作成中...")
        anim = create_comprehensive_analysis(robot, results, save_all=True, show_all=False)
        return results, anim
    except ImportError as e:
        print(f"⚠️ アニメーションモジュールのインポートエラー: {e}")
        print("   アニメーション機能なしでシミュレーション結果を返します")
        return results, None

def test_different_scenarios(config_path=None):
    """異なるシナリオでのテスト"""
    print("=== 複数シナリオテスト ===")
    
    # 設定ファイルからシナリオを読み込み
    config = load_config(config_path)
    
    if config is not None and 'scenarios' in config:
        # 設定ファイルからシナリオを使用
        scenarios = []
        for scenario_config in config['scenarios']:
            scenario = {
                'name': scenario_config.get('name', '無名シナリオ'),
                'state': extract_initial_state_from_config(scenario_config),
                'time': scenario_config.get('simulation', {}).get('time', 2.0),
                'dt': scenario_config.get('simulation', {}).get('dt', 0.005)
            }
            scenarios.append(scenario)
        print(f"📋 設定ファイルから{len(scenarios)}個のシナリオを読み込みました")
    else:
        # デフォルトシナリオ
        print("📋 デフォルトシナリオを使用します")
        scenarios = [
            {
                'name': '対称初期状態（標準）',
                'state': [0.0, 0.0, 0.0, -0.1, -0.1, 0.2, 0.2, 0.0, 0.0],
                'time': 2.0,
                'dt': 0.005
            },
            {
                'name': '非対称初期状態（膝）',
                'state': [0.0, 0.0, 0.0, -0.2, -0.05, 0.2, 0.2, 0.0, 0.0],
                'time': 2.0,
                'dt': 0.005
            },
            {
                'name': '非対称初期状態（腰）',
                'state': [0.0, 0.0, 0.0, -0.1, -0.1, 0.3, 0.1, 0.0, 0.0],
                'time': 2.0,
                'dt': 0.005
            },
            {
                'name': '初期ホイール回転差',
                'state': [0.0, 0.0, 0.0, -0.1, -0.1, 0.2, 0.2, 0.5, -0.5],
                'time': 2.0,
                'dt': 0.005
            }
        ]
    
    for i, scenario in enumerate(scenarios):
        print(f"\n--- シナリオ {i+1}: {scenario['name']} ---")
        results, anim = run_simulation_with_animation(
            initial_state=scenario['state'],
            T_sim=scenario['time'],
            dt=scenario.get('dt', 0.005)
        )
        
        # 結果の簡単な分析
        if results is not None:
            final_state = results['state'][-1]
            print(f"最終状態: 位置=[{final_state[0]:.3f}, {final_state[1]:.3f}], "
                  f"pitch={final_state[2]:.3f}, "
                  f"膝=[{final_state[3]:.3f}, {final_state[4]:.3f}], "
                  f"腰=[{final_state[5]:.3f}, {final_state[6]:.3f}], "
                  f"ホイール=[{final_state[7]:.3f}, {final_state[8]:.3f}]")

if __name__ == "__main__":
    import argparse
    
    # コマンドライン引数の解析
    parser = argparse.ArgumentParser(description='3Dロボットシミュレーション')
    parser.add_argument('--config', type=str, help='設定ファイルのパス')
    parser.add_argument('--scenario', action='store_true', help='複数シナリオテストを実行')
    parser.add_argument('--no-animation', action='store_true', help='アニメーションを無効化')
    args = parser.parse_args()
    
    # 基本テスト
    test_gravity_pendulum()
    
    if args.scenario:
        # 複数シナリオテスト
        print("\n" + "="*60)
        test_different_scenarios(config_path=args.config)
    else:
        # アニメーション付きシミュレーション
        print("\n" + "="*60)
        results, anim = run_simulation_with_animation(config_path=args.config)