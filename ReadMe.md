**説明**
ROS2-TMS for ConstructionからOPERAを介して建設機械を操作する際のドライバ。
tms_if_for_ooperaからaction通信を介して受け取った指令値をtopic通信に変換し、シミュレータに投げて動作させる。
実機では車載。pwri-operaで公開されたリポジトリに本機能がなかったため追加実装。
シミュレータ上のブル(d37pxi)のブレード操作、上部旋回型ダンプ(mst110cr)の旋回操作・ベッセル開閉操作時に必須。


**動作検証の実施状況**
*Crawlerdump*
vessel_control：動作検証済
swing_control：動作検証未実施（OperaSim-Physx最新版にswing軸操作用subscriber /mst110cr/swing/cmdが未実装のため）

*Bulldozer*
・blade_control：動作検証済

**動作検証用コマンド**

*Crawlerdump*
vessel_control：動作検証済（OperaSim-PhysX）

```ros2 action send_goal /set_swing_angle tms_msg_rp/action/TmsRpCrawlerDumpSwingAngle "{target_angle: 1.0, control_type: 1, velocity: 0.01, effort: 0.0}"```

・swing_control：動作検証未実施

```ros2 action send_goal /set_vessel_angle tms_msg_rp/action/TmsRpCrawlerDumpDumpAngle "{target_angle: -1.0, control_type: 0, velocity: 0.01, effort: 0.0}"```


*Bulldozer*
・blade_control：動作検証済 (operaSim-PAGX)

ros2 action send_goal /set_d37pxi_blade tms_msg_rp/action/TmsRpBulldozerBlade "{joint_name: ['lift_joint','tilt_joint','angle_joint'], control_type: 0, goal_position: [-0.1, 0.0, 0.0], velocity: [], effort: []}"


