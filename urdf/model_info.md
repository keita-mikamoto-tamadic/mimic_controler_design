## urdfに明示されていないパラメータ
タイヤの半径：wheel_radius = (77.95 / 2) / 1000 [mm]

- 運動学的ツリー構造
```
universe (JointModelRX)
    └─ root_joint (JointModelFreeFlyer)
        ├── upper_link_R_joint (JointModelRUBY)
        │   └─ lower_link_R_joint (JointModelRUBY)
        │       └─ wheel_R_joint (JointModelRUBY)
        └─ upper_link_L_joint (JointModelRUBY)
            └─ lower_link_L_joint (JointModelRUBY)
                └─ wheel_L_joint (JointModelRUBY)
                

  base_link (胴体) ← フローティングベース
      |
      | 股関節 (Y軸周り回転)
      |
  upper_link_R (太もも)
      |
      | 膝関節 (Y軸周り回転)
      |
  lower_link_R (すね)
      |
      | ホイール関節 (Y軸周り回転)
      |
  wheel_R (ホイール)