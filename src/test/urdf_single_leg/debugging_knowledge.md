# URDFシングルレッグ デバッグ知見集

## 🔧 修正完了した問題

### 1. 重力方向の修正
**問題**: 重力が上向き（+9.81）で設定されていた
```python
# 修正前（3ファイルすべて）
model.gravity.linear = np.array([0, 0, +9.81])  # 上向き重力でテスト

# 修正後
model.gravity.linear = np.array([0, 0, -9.81])  # 下向き重力（正しい物理）
```

### 2. 運動方程式の符号修正
**問題**: 運動方程式で重力項の符号が間違っていた
```python
# 修正前
dd_state = np.linalg.solve(M_red, tau - g_red - C_red)

# 修正後
dd_state = np.linalg.solve(M_red, tau + g_red - C_red)
```

### 3. Pinocchioクォータニオン順序の修正
**問題**: Pinocchioのクォータニオン順序 `[x,y,z,w]` を `[w,x,y,z]` として設定していた

```python
# 修正前（間違った順序）
q[3] = quat_xyzw[3]  # w
q[4] = quat_xyzw[0]  # x
q[5] = quat_xyzw[1]  # y
q[6] = quat_xyzw[2]  # z

# 修正後（正しい順序）
q[3] = quat_xyzw[0]  # x
q[4] = quat_xyzw[1]  # y
q[5] = quat_xyzw[2]  # z
q[6] = quat_xyzw[3]  # w
```

**重要**: Pinocchioの中立構成確認
```python
q_neutral = pin.neutral(model)  # [0, 0, 0, 0, 0, 0, 1, ...]
# クォータニオン部分: q[3:7] = [0, 0, 0, 1] = [x, y, z, w]
```

### 4. 接地拘束計算の統一
**問題**: `simple_constrained.py`と`fixed_pitch_constrained.py`で異なる接地拘束計算手法

**simple_constrained.py（正しい方法）**:
```python
def compute_base_height(phi1, phi2, model, data):
    # 仮構成で順運動学
    pin.forwardKinematics(model, data, q)
    wheel_pos = data.oMi[model.njoints-1].translation
    
    # シンプルで正確な計算
    base_height = WHEEL_RADIUS - wheel_pos[2]
    return base_height
```

**fixed_pitch_constrained.py（修正前の複雑な方法）**:
```python
# 二分探索による反復計算（不要に複雑）
for _ in range(50):
    z_mid = (z_low + z_high) / 2
    wheel_bottom = compute_wheel_bottom_z_at_base_height(z_mid)
    # ... 複雑な条件分岐
```

**修正**: `simple_constrained.py`と同じ手法に統一

### 5. 関節角度の物理的意味
**確認された定義**:
- `phi1 = 0°`: 上リンクが垂直下向き → 自然な立位
- `phi1 = 90°`: 上リンクが水平前向き
- `phi1 = 180°`: 上リンクが垂直上向き → 逆さ状態

- `phi2 = 0°`: 下リンクが上リンクと同じ方向（伸ばした状態）
- `phi2 = 180°`: 下リンクが上リンクと逆方向（折り曲げた状態）
- `phi2 = -90°`: 下リンクが上リンクに対して90度曲がる

**最も自然な立位**: `phi1 = 0°, phi2 = 0°`

## ✅ 解決済み問題 (追加)

### 6. base_pitchが0度以外での姿勢計算問題
**問題**: pitch角度を0以外に設定すると、接地拘束が満たされない

**原因**: 
1. **ワールド座標とローカル座標の混同**: ベースのローカルZ座標を変更していたが、正しくはワールド座標でのオフセットが必要
2. **Euler角変換のdegrees/radians問題**: `degrees=True`と`np.rad2deg()`の重複使用

**修正前**:
```python
# 間違った方法: ローカル座標での計算
base_height = WHEEL_RADIUS - wheel_pos[2]
q[2] = base_height  # ベースのローカルZ座標を直接設定
```

**修正後**:
```python
# 正しい方法: ワールド座標でのオフセット計算
wheel_bottom_z = wheel_pos[2] - WHEEL_RADIUS
base_height = q[2] - wheel_bottom_z  # 現在位置からの必要なオフセット量
```

**Euler角変換の統一**:
```python
# 修正前（冗長）
r = Rotation.from_euler('y', np.rad2deg(pitch_base), degrees=True)

# 修正後（シンプル）
r = Rotation.from_euler('y', pitch_base, degrees=False)
```

**結果**: 
- 接地拘束誤差: 4.86e-17 (機械精度レベル)
- pitch=10度でも完全に動作
- ワールド座標オフセットの線形性が確認された

## 📋 次のステップ

1. ✅ **pitch≠0での接地拘束計算の詳細調査** → 完了
2. ✅ **pitch角度を含む運動学の検証** → 完了  
3. **ノンスリップ拘束の精度向上** → 残存課題（誤差: 最大6.28）
4. **動力学シミュレーションの安定性向上**
5. **より複雑なpitch角度での動作検証**

## 🛠️ デバッグ手法

### 構成ベクトル確認
```python
print(f'q[3:7] (quaternion): {q[3:7]}')  # [x,y,z,w]順序
print(f'Expected neutral: [0,0,0,1]')
```

### 接地拘束確認
```python
wheel_bottom_z = wheel_pos[2] - WHEEL_RADIUS
print(f'Ground constraint error: {wheel_bottom_z:.6f}m')
```

### ベース・ホイール位置関係確認
```python
if base_pos[2] > wheel_pos[2]:
    print('✅ 正常: ベース > ホイール')
else:
    print('⚠️ 異常: ベース < ホイール（逆さ）')
```

## 💡 重要な教訓

1. **Pinocchioクォータニオン順序**: 必ず `[x,y,z,w]` 順序を使用
2. **接地拘束計算**: 複雑な反復計算より単純な解析解が正確
3. **物理量の符号**: 重力方向と運動方程式の符号は慎重に確認
4. **段階的デバッグ**: pitch=0で完全に動作してからpitch≠0に進む
5. **座標系の理解**: ワールド座標とローカル座標を明確に区別する
6. **シンプルなオフセット**: pitch≠0でもワールド座標での線形オフセットが有効
7. **角度単位の統一**: degrees/radiansの変換を重複させない

このドキュメントは今後の開発とデバッグの重要な参考資料として活用してください。