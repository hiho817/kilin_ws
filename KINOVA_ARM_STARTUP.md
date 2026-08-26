# Kinova Gen3 手臂啟動順序

本文件適用於：

- Kinova Gen3 7-DoF 真機
- 手臂 IP：`192.168.1.10`
- `ros2_kortex_ws`：`~/ros2_kortex_ws`
- Kilin ROS 2 workspace：`~/kilin_ws/kilin_ros_ws`
- PTP action：`/kinova_joint_ptp`

## 0. 真機操作前檢查

1. 確認手臂周圍沒有障礙物、人員或可能纏繞的線材。
2. 確認急停按鈕可立即操作。
3. 第一次測試只做小幅度關節運動。
4. `kinova_joint_ptp` 不提供碰撞規劃、自碰撞檢查或環境避障。
5. 不要同時從其他節點或 Kinova Web App 發送運動命令。

## 1. 檢查電腦與手臂網路

連接 Kinova 的乙太網路介面應設定在 `192.168.1.0/24` 網段，例如：

```text
電腦乙太網路 IP：192.168.1.11
Netmask：255.255.255.0
Gateway：留空
DNS：留空
Kinova IP：192.168.1.10
```

確認封包會從乙太網路送出：

```bash
ip route get 192.168.1.10
```

預期結果應包含類似內容：

```text
192.168.1.10 dev eno1 src 192.168.1.11
```

確認手臂可以連線：

```bash
ping -c 3 192.168.1.10
nc -vz -w 2 192.168.1.10 10000
nc -vz -w 2 192.168.1.10 10001
```

若 ping 和連接埠測試失敗，不要繼續啟動 ROS 2 driver。

## 2. 啟動 ros2_kortex

開啟第一個 terminal：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_kortex_ws/install/setup.bash

ros2 launch kortex_bringup gen3.launch.py \
  robot_ip:=192.168.1.10 \
  dof:=7 \
  robot_controller:=joint_trajectory_controller
```

等待 hardware interface 完成連線，以及 controllers 完成載入。若畫面一直停在：

```text
Connecting to robot at 192.168.1.10
```

通常表示電腦到手臂的網路尚未連通。

## 3. 驗證 Kortex controllers

開啟第二個 terminal：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_kortex_ws/install/setup.bash

ros2 control list_controllers
```

確認至少看到：

```text
joint_state_broadcaster ... active
joint_trajectory_controller ... active
```

確認 trajectory action 存在：

```bash
ros2 action list | grep follow_joint_trajectory
```

預期包含：

```text
/joint_trajectory_controller/follow_joint_trajectory
```

確認真機關節狀態持續發布：

```bash
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
```

`/joint_states` 內的關節排列順序可能不同，不代表關節對應錯誤；ROS 訊息會以 `name` 和 `position` 的相同索引配對。

## 4. 啟動 kinova_joint_ptp

保持 Kortex terminal 運行，再開啟第三個 terminal：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_kortex_ws/install/setup.bash
source ~/kilin_ws/kilin_ros_ws/install/setup.bash

ros2 launch kinova_joint_ptp joint_ptp.launch.py
```

確認 PTP action 已出現：

```bash
ros2 action list | grep kinova_joint_ptp
ros2 action info /kinova_joint_ptp
```

## 5. 第一次小幅運動測試

先執行 `ros2 topic echo /joint_states --once` 取得最新七軸位置。測試時只改動一個關節約 `0.05` 至 `0.10 rad`，其他關節填入最新位置。

PTP server 預設要求 goal 包含全部七個關節：

```bash
ros2 action send_goal --feedback \
  /kinova_joint_ptp \
  kinova_ptp_interfaces/action/JointPtp \
  "{
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
    positions: [J1, J2, J3, J4, J5, J6, J7],
    velocities: [],
    duration_sec: 5.0
  }"
```

將 `J1` 至 `J7` 換成實際數值。命令成功時，結果應包含：

```text
success: true
error_code: 0
```

若需要中止，使用另一個已載入相同環境的 terminal：

```bash
ros2 action list
```

真機出現不安全動作時，優先使用實體急停，不要只依賴 ROS 2 cancel。

## 6. Stair controller 的 Standard Pose

Kilin standard pose 的順序為 `joint_1` 至 `joint_7`。

角度：

```text
[0.0, -85.94, 0.0, 147.0, 0.0, 22.92, 0.0] deg
```

弧度約為：

```text
[0.0, -1.4999, 0.0, 2.5656, 0.0, 0.4000, 0.0] rad
```

發送命令：

```bash
ros2 action send_goal --feedback \
  /kinova_joint_ptp \
  kinova_ptp_interfaces/action/JointPtp \
  "{
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
    positions: [0.0, -1.4999, 0.0, 2.5656, 0.0, 0.4000, 0.0],
    velocities: [],
    duration_sec: 15.0
  }"
```

Standard pose 可能與手臂當前姿態相距很遠。發送前必須逐軸比較最新 `/joint_states`，並確認整段運動不會碰撞。Standard pose 不是七軸全零姿態。

## 7. 啟動 stair controller

確認 Kortex driver、`joint_trajectory_controller`、`/joint_states` 和 `/kinova_joint_ptp` 都正常後，再啟動 stair controller。

若由 stair controller launch 同時啟動 PTP server：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_kortex_ws/install/setup.bash
source ~/kilin_ws/kilin_ros_ws/install/setup.bash

ros2 launch kilin_stair_controller launch.py
```

若已經在第三個 terminal 手動啟動 `kinova_joint_ptp`，避免建立重複的 action server：

```bash
ros2 launch kilin_stair_controller launch.py start_ptp:=false
```

## 8. 關閉順序

1. 停止 stair controller，避免再產生新的手臂目標。
2. 停止 `kinova_joint_ptp`。
3. 確認手臂已靜止。
4. 停止 `kortex_bringup`。
5. 依現場安全程序關閉手臂電源。

