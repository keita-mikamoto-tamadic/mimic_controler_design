# Pinocchioフローティングベース研究

## 概要
Pinocchioライブラリで既存の固定ベースURDFをフローティングベース（floating base）として扱う方法を調査し、実装例を提供する。

## 1. フローティングベースの有効化方法

### 1.1 pin.buildModelFromUrdf()の使用方法

#### 基本的な使用方法
```python
import pinocchio as pin

# 方法1: JointModelFreeFlyer()を指定
model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
data = model.createData()

# 方法2: floating jointをURDFで定義（推奨）
model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()
```

#### より詳細な例
```python
import pinocchio as pin
import numpy as np

def load_floating_base_model(urdf_path):
    """フローティングベースモデルの読み込み"""
    # JointModelFreeFlyer()を明示的に指定
    model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
    data = model.createData()
    
    # 重力設定
    model.gravity.linear = np.array([0, 0, -9.81])
    
    print(f"モデル情報:")
    print(f"  構成空間次元 (nq): {model.nq}")
    print(f"  速度空間次元 (nv): {model.nv}")
    print(f"  関節数: {model.njoints}")
    
    return model, data
```

### 1.2 URDF内でのfloating joint定義（推奨方法）

```xml
<?xml version="1.0" ?>
<robot name="robot_name">
  <!-- World link -->
  <link name="world"/>
  
  <!-- Floating base joint (6DOF) -->
  <joint name="floating_base" type="floating">
    <parent link="world"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
  </joint>
  
  <!-- Base link -->
  <link name="base_link">
    <!-- inertial, visual, collision properties -->
  </link>
  
  <!-- 以下、通常の関節とリンク定義 -->
</robot>
```

## 2. 構成ベクトル（Configuration Vector）の扱い方

### 2.1 構成ベクトルの構造

フローティングベースでは、構成ベクトル`q`の構造は以下のようになる：

```python
# q[0:3]: ベース位置 [px, py, pz] (m)
# q[3:7]: ベース姿勢クォータニオン [qx, qy, qz, qw]
# q[7:]: 関節角度 [θ1, θ2, ...]

# 速度ベクトルdqの構造：
# dq[0:3]: ベース線速度 [vx, vy, vz] (m/s)
# dq[3:6]: ベース角速度 [ωx, ωy, ωz] (rad/s)  
# dq[6:]: 関節角速度 [θ̇1, θ̇2, ...]
```

### 2.2 構成ベクトルの初期化

```python
def create_initial_configuration(model, base_position=[0,0,1], base_orientation=[0,0,0,1]):
    """初期構成ベクトルの作成"""
    # 中立構成から開始
    q = pin.neutral(model)
    
    # ベース位置設定
    q[0:3] = base_position
    
    # ベース姿勢（クォータニオン）設定
    q[3:7] = base_orientation / np.linalg.norm(base_orientation)
    
    # 関節角度は適宜設定
    # q[7:] = joint_angles
    
    return q

# 使用例
model, data = load_floating_base_model("robot.urdf")
q = create_initial_configuration(model, base_position=[0, 0, 0.8])
dq = np.zeros(model.nv)  # 初期速度は零
```

### 2.3 次元の関係

```python
# 重要な関係式
# nq = nv + quaternion_dimension_difference
# フローティングベースの場合: nq = nv + 1
# (クォータニオン4次元 vs 角速度3次元の差)

print(f"構成空間次元 nq: {model.nq}")
print(f"速度空間次元 nv: {model.nv}")
print(f"次元差: {model.nq - model.nv}")  # フローティングベースなら1
```

## 3. フローティングベースでの動力学計算

### 3.1 基本的な動力学計算

```python
def compute_floating_base_dynamics(q, dq, tau, model, data):
    """フローティングベース動力学計算"""
    # 順運動学
    pin.forwardKinematics(model, data, q, dq)
    pin.updateFramePlacements(model, data)
    
    # 慣性行列
    pin.crba(model, data, q)
    M = data.M.copy()
    
    # 重力項
    pin.computeGeneralizedGravity(model, data, q)
    g = data.g.copy()
    
    # コリオリ・遠心力項
    pin.computeCoriolisMatrix(model, data, q, dq)
    C = data.C @ dq
    
    # 運動方程式: M * ddq = tau - g - C
    ddq = np.linalg.solve(M, tau - g - C)
    
    return ddq, M, g, C
```

### 3.2 制約付き動力学（接地拘束など）

```python
def compute_constrained_dynamics(q, dq, tau, model, data, contact_frames):
    """制約付き動力学計算"""
    # 順運動学とヤコビアン計算
    pin.forwardKinematics(model, data, q, dq)
    pin.computeJointJacobians(model, data, q)
    pin.updateFramePlacements(model, data)
    
    # 接触ヤコビアン
    contact_jacobians = []
    for frame_id in contact_frames:
        J = pin.getFrameJacobian(model, data, frame_id, pin.LOCAL_WORLD_ALIGNED)
        contact_jacobians.append(J)
    
    if contact_jacobians:
        Jc = np.vstack(contact_jacobians)
        
        # 制約付き動力学
        pin.crba(model, data, q)
        M = data.M
        
        pin.computeGeneralizedGravity(model, data, q)
        g = data.g
        
        # 制約力を考慮した動力学解
        # (詳細な実装は制約の種類による)
        
    return ddq
```

### 3.3 数値積分（特にクォータニオン）

```python
def integrate_configuration(q, dq, dt, model):
    """構成ベクトルの数値積分（クォータニオン対応）"""
    q_new = q.copy()
    
    # ベース位置の積分
    q_new[0:3] = q[0:3] + dq[0:3] * dt
    
    # ベース姿勢（クォータニオン）の積分
    omega = dq[3:6]  # 角速度
    omega_norm = np.linalg.norm(omega)
    
    if omega_norm > 1e-8:
        # 角速度からクォータニオン更新
        axis = omega / omega_norm
        angle = omega_norm * dt
        
        dq_quat = np.array([
            axis[0] * np.sin(angle/2),
            axis[1] * np.sin(angle/2), 
            axis[2] * np.sin(angle/2),
            np.cos(angle/2)
        ])
        
        # クォータニオン積
        q_curr = q[3:7]
        q_new[3:7] = pin.quaternion.quaternionProduct(dq_quat, q_curr)
        q_new[3:7] = q_new[3:7] / np.linalg.norm(q_new[3:7])  # 正規化
    
    # 関節角度の積分
    q_new[7:] = q[7:] + dq[6:] * dt
    
    return q_new
```

## 4. 実装例：完全なシミュレーション

### 4.1 シミュレーションループ

```python
def simulate_floating_base_robot(model, data, q0, dq0, T_sim=5.0, dt=0.01):
    """フローティングベースロボットシミュレーション"""
    t_array = np.arange(0, T_sim, dt)
    N = len(t_array)
    
    # 状態履歴
    q_history = np.zeros((N, model.nq))
    dq_history = np.zeros((N, model.nv))
    
    q, dq = q0.copy(), dq0.copy()
    
    for i, t in enumerate(t_array):
        # 状態記録
        q_history[i] = q
        dq_history[i] = dq
        
        # 制御入力（この例では無制御）
        tau = np.zeros(model.nv)
        
        # 動力学計算
        ddq, M, g, C = compute_floating_base_dynamics(q, dq, tau, model, data)
        
        # 数値積分
        dq = dq + ddq * dt
        q = integrate_configuration(q, dq, dt, model)
        
        # 安全チェック
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(dq)):
            print(f"数値発散 at t={t:.3f}")
            break
    
    return t_array, q_history, dq_history
```

### 4.2 使用例

```python
def main():
    """メイン実行例"""
    # モデル読み込み
    urdf_path = "path/to/robot.urdf"
    model, data = load_floating_base_model(urdf_path)
    
    # 初期状態設定
    q0 = create_initial_configuration(model, base_position=[0, 0, 1.0])
    dq0 = np.zeros(model.nv)
    
    # シミュレーション実行
    t_array, q_hist, dq_hist = simulate_floating_base_robot(
        model, data, q0, dq0, T_sim=5.0, dt=0.01
    )
    
    # 結果解析
    base_positions = q_hist[:, 0:3]
    print(f"初期高度: {base_positions[0, 2]:.3f}m")
    print(f"最終高度: {base_positions[-1, 2]:.3f}m")
    print(f"落下距離: {base_positions[0, 2] - base_positions[-1, 2]:.3f}m")

if __name__ == "__main__":
    main()
```

## 5. 注意点とベストプラクティス

### 5.1 数値安定性
- クォータニオンは常に正規化する
- 特異値の回避（M行列の条件数チェック）
- 適切な時間刻み幅の選択

### 5.2 制約処理
- 接地制約等は適切なヤコビアンで処理
- ラグランジュ乗数法やペナルティ法の使い分け

### 5.3 効率化
- 必要な計算のみ実行（例：static関数の使用）
- メモリ割り当ての最小化

## 6. 実装テスト結果

### 6.1 テスト環境
- Pinocchio版数: 3.7.0
- プロジェクト内URDFファイル: mimic_v1_single_leg_floating.urdf

### 6.2 テスト成功項目
✅ **URDFからのフローティングベースモデル読み込み**
- URDF内の`<joint type="floating">`定義が正しく認識
- JointModelFreeFlyer (nq=7, nv=6) として正常に読み込まれる

✅ **構成ベクトルの正しい解釈**
- q[0:3]: ベース位置 [x, y, z] 
- q[3:7]: ベース姿勢クォータニオン [qx, qy, qz, qw]
- q[7:]: 関節角度（RUBYジョイント: cos-sin表現で2次元/関節）

✅ **積分処理**
- `pin.integrate(model, q, dq*dt)` が全ての関節タイプで正常動作
- クォータニオン積分を含む複雑な構成空間も適切に処理

✅ **動力学計算**
- 順運動学: pin.forwardKinematics() 正常
- 慣性行列: pin.crba() 正常 (9×9行列)
- 重力項: pin.computeGeneralizedGravity() 正常
- Z軸加速度が期待値 -9.81m/s² と一致

✅ **JointModelFreeFlyer()による代替方法**
- `pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())` も正常動作
- 固定ベースURDFをフローティングベース化可能

### 6.3 重要な発見事項

#### 関節構成の詳細
```
floating_base: JointModelFreeFlyer (nq=7, nv=6)
  - 構成空間: 位置3次元 + クォータニオン4次元 = 7次元
  - 速度空間: 線速度3次元 + 角速度3次元 = 6次元

upper_link_R_joint: JointModelRUBY (nq=2, nv=1) 
  - cos-sin表現のため構成空間が2次元
  - 速度空間は1次元（角速度のみ）
```

#### 総次元
- 総構成空間次元: nq = 7 + 2×3 = 13
- 総速度空間次元: nv = 6 + 1×3 = 9
- 次元差: nq - nv = 4 (フローティングベース1 + RUBYジョイント3)

### 6.4 ベストプラクティス

1. **積分にはpin.integrate()を使用**
   - 手動のクォータニオン積分は不要
   - すべての特殊関節タイプに対応

2. **URDFでのfloating joint定義推奨**
   ```xml
   <joint name="floating_base" type="floating">
     <parent link="world"/>
     <child link="base_link"/>
   </joint>
   ```

3. **数値安定性**
   - 小さな時間刻み(dt < 0.01)を使用
   - 慣性行列の条件数監視 (5.7e+04程度は正常)

## 7. 参考リンク

- [Pinocchio公式ドキュメント](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/devel/doxygen-html/)
- [GitHubリポジトリ](https://github.com/stack-of-tasks/pinocchio)
- [floating-base-velocity-viewer.py例](https://github.com/stack-of-tasks/pinocchio/blob/master/examples/floating-base-velocity-viewer.py)
- [プロジェクト内テストコード](./simple_floating_test.py)