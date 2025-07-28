# URDF 2D ロボット可視化システム

## プロジェクト概要

URDFファイルから複雑な4脚ロボットモデルを読み込み、Pinocchio順運動学を使って指定された関節角度での各リンク位置を2D平面で可視化するシステム。

## 物理モデル

### ロボット構成
- **ベースリンク**: フローティングベース（6DOF）
- **左右対称4脚構造**: 各脚3関節チェーン
  - `upper_link` (大腿部): hip_pitch関節
  - `lower_link` (下腿部): knee関節  
  - `wheel` (足輪): wheel関節

### 座標系定義
```
z ^
  |
  +----> x

フローティングベース: [x, y, z, roll, pitch, yaw]
関節角度: [hip_pitch_R, knee_R, wheel_R, hip_pitch_L, knee_L, wheel_L]
```

### 指定動作条件
- **ベース位置**: `[0, 0, 0]`
- **ベース姿勢**: 無回転（単位行列）
- **右脚関節角度**:
  - `upper_link_R_joint` (hip_pitch): 52° = 0.9076 rad
  - `lower_link_R_joint` (knee): -104° = -1.8151 rad
  - `wheel_R_joint`: 0° (初期位置)

## 実装ファイル構成

```
src/test/urdf_robot_2d/
├── urdf_robot_2d.md                    # 本プラン文書
├── load_urdf_model.py                  # URDF読み込み・モデル構築
├── visualize_2d_robot.py               # 2D可視化システム
└── test_urdf_forward_kinematics.py     # 順運動学テスト実行
```

## 実装仕様

### 1. URDF読み込み機能 (`load_urdf_model.py`)

```python
def load_mimic_robot_model(urdf_path):
    """
    URDF から mimic ロボットモデルを構築
    
    Returns:
        model: Pinocchio モデル
        data: Pinocchio データ
        joint_names: 関節名リスト
        link_names: リンク名リスト
    """

def get_joint_positions_2d(model, data, q_config):
    """
    指定された設定での全関節位置を計算（2D射影）
    
    Args:
        q_config: 完全な関節設定 [floating_base(6) + joints]
    
    Returns:
        joint_positions: 各関節の2D位置 {joint_name: [x, z]}
        link_positions: 各リンクの2D位置 {link_name: [x, z]}
    """
```

### 2. 2D可視化システム (`visualize_2d_robot.py`)

```python
def plot_robot_2d(joint_positions, link_positions, title="URDF Robot 2D"):
    """
    ロボットの2D可視化
    
    表示要素:
    - リンク間の接続線
    - 関節位置（マーカー）
    - ホイール位置（強調表示）
    - 座標軸とグリッド
    """

def animate_robot_configuration(model, data, q_configs, titles):
    """
    複数設定での比較アニメーション
    """
```

### 3. 順運動学テスト (`test_urdf_forward_kinematics.py`)

```python
def demo_urdf_forward_kinematics():
    """
    指定角度での順運動学デモ
    
    テスト設定:
    1. 初期姿勢（全関節0°）
    2. 指定姿勢（hip=52°, knee=-104°）
    3. 比較表示と妥当性確認
    """
```

## 期待される結果

### 技術的達成目標
1. **URDF正常読み込み**: `pin.buildModelFromUrdf()` 成功
2. **順運動学計算**: 指定角度での各関節位置の正確な計算
3. **2D可視化**: X-Z平面での直感的な表示
4. **物理的妥当性**: 関節位置の幾何学的整合性確認

### 出力例
```
=== URDF Robot 2D Forward Kinematics ===
Model loaded: 6 DOF floating base + 6 joints
Configuration: hip_pitch=52°, knee=-104°

Joint Positions (2D):
  base_link: [0.000, 0.000]
  upper_link_R: [0.000, -0.133]
  lower_link_R: [0.118, -0.259]
  wheel_R: [0.118, -0.409]

Display: 2D visualization showing robot pose
```

## 使用するPinocchio関数

- `pin.buildModelFromUrdf(urdf_path)`: URDFからモデル構築
- `pin.forwardKinematics(model, data, q)`: 順運動学計算
- `pin.updateFramePlacements(model, data)`: フレーム位置更新
- `data.oMi[joint_id].translation`: 関節位置取得
- `model.getJointId(joint_name)`: 関節IDの取得

## 開発フロー

### Phase 1: 基本読み込み機能
1. URDF読み込みと基本情報表示
2. フローティングベース設定確認
3. 関節・リンク名の整理

### Phase 2: 順運動学実装
1. 指定角度での設定ベクトル構築
2. 順運動学計算と結果取得
3. 2D射影と座標変換

### Phase 3: 可視化システム
1. 基本的な2Dプロット機能
2. リンク・関節の表示
3. アニメーション機能（オプション）

### Phase 4: テスト・検証
1. 指定角度での動作確認
2. 物理的妥当性の検証
3. simple_testとの比較基準

## 次のステップ予定

本システム完成後:
1. **advanced constraint system**: URDFモデルでの拘束処理
2. **動力学シミュレーション**: 複雑モデルでの動的解析
3. **制御システム統合**: フィードバック制御の実装