# 自动验证与三组短时对照

以下命令均在分享仓库根目录执行。自动测试自行启动临时服务和独立话题，无需同时开手工服务/桥接。

## TCP 与安全测试

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/lab-simulation-mpl \
  .my_venv/bin/python -m unittest tools.test_exo_tcp -v
```

预期 13 项测试通过，覆盖真实状态读取、左右通道、+1/0/−1 映射、限幅、异常 JSON / NaN / Inf、半包粘包、序号/版本/epoch、重复性、超时关闭、断线、终止、异常和 stop。测试不使用随机肌肉输入作为控制策略。

## 真实 ROS 2 闭环

```bash
# 无窗口检查：脚本自动 source ROS 并使用独立测试域
bash scripts/test_ros.sh

# 窗口与用户演示也一并验证，需要桌面显示
bash scripts/test_ros.sh --window --demo
```

预期末尾打印 `ROS -> TCP -> MuJoCo -> ROS validation passed`。检查真实 ctrl / actuator_force、仿真时间、六关节反馈，看门狗与桥接退出清零；逐条将 ROS 状态与服务原始日志比较。窗口测试还检查模型/数据共享和显示快照一致，演示检查左右 −5 N·m 及关闭状态。

自动测试用 ROS_DOMAIN_ID=73；可设置 `MYOASSIST_TEST_DOMAIN_ID`。输出在 `results/ros_tcp_validation*` 或 `results/ros_live_validation*`，默认文件可能被重复测试覆盖。分享包只附精选验证摘要，不打包每次运行的大日志。

## 三组短时动力学对照

```bash
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=74
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/lab-simulation-mpl \
  /usr/bin/python3 -m tools.test_exo_comparison
```

需要窗口时添加 `--window`。程序自行启动临时服务、桥接及对照客户端；每组 12 个控制步，两次重复：

1. 两侧关闭，基线。
2. 右侧短时 −5 N·m。
3. 左侧短时 −5 N·m。

核验完整初态、肌肉输入、步数和脉冲前状态一致，再比较相对基线的角度/速度差值。生成 states.csv、deltas.csv、samples.json、summary.json、joint_deltas.png、device_torques.png 和反馈日志。成功输出 `Three-arm ROS/MuJoCo comparison PASSED`。

这是验证设备输入确实改变动力学响应的短时实验，不是助力效果或健康步态评价。

## 单独检查模型接口、录像

```bash
.my_venv/bin/python -m tools.inspect_exo_interface --help
.my_venv/bin/python -m tools.demo_exo_video --window --seconds 3
MUJOCO_GL=egl .my_venv/bin/python -m tools.demo_exo_video \
  --seconds 3 --width 640 --height 480 --output results/dephy_demo.mp4
```

`demo_exo_video` 是独立的模型展示程序，设备安全关闭，没有 ROS 控制器。观察 ROS 控制必须使用 TCP 服务的 `--window`，不能拿这个独立窗口冒充同一实例。

## 分享文件完整性

```bash
python3 scripts/verify_package.py
```

只核对分发文件 SHA256、展示已存摘要；不重新运行仿真，也不将文件完整性当成控制验收。
