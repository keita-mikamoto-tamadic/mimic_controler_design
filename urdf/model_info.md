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