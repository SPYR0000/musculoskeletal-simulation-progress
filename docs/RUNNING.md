# MS-Human-700 实时仿真运行说明

所有命令在本分享目录根下执行。此入口只复现 MS-Human-700 / msgym，不需要运行 ROS 2 或 MyoAssist 服务。

本地原环境已经完成运行；以下是整理后的新机器准备步骤，本次没有在一台空白机器上重新安装验证。分享脚本会使用下载到 `workspace/` 的官方内容，不随包分发模型和权重。下载及运行需要联网和可用的 Python 3.12；窗口需要桌面图形环境。

## 一键准备入口

具备 uv、git、curl 后，可运行 `bash scripts/setup_ms700.sh`。它按本说明固定提交、核对 ZIP / 策略 / 归一化 / 配置哈希，在独立 `.venv_ms700` 中安装。此安装脚本完成语法与帮助入口检查；本次没有重新下载并全新安装这条路线，已验证的推理使用既有兼容环境。下面保留手工准备步骤，便于排查具体环节。

## 1. 准备固定版本的官方源码

```bash
mkdir -p workspace/sources workspace/assets
git clone https://github.com/LNSGroup/msgym.git workspace/sources/msgym
git -C workspace/sources/msgym checkout ad3aac166dab3577d7d16b4a03faccef94c7aac7
git -C workspace/sources/msgym submodule update --init --recursive
```

核对模型子模块提交：

```bash
git -C workspace/sources/msgym/msgym/MS-Human-700 rev-parse HEAD
```

应为 `2d686957aefd5739cf4d2859a6acfd94c8f84400`。若不一致，先查清版本，不替换模型文件凑运行结果。

## 2. 下载官方检查点并核验

```bash
curl -fL --retry 3 \
  https://github.com/LNSGroup/msgym/releases/download/Checkpoints/LocomotionFull.zip \
  -o workspace/assets/LocomotionFull.zip
curl -fL --retry 3 \
  https://github.com/LNSGroup/msgym/releases/download/Checkpoints/LocomotionLegs.zip \
  -o workspace/assets/LocomotionLegs.zip
```

下面先检查官方 ZIP 的 SHA256，一致才解压；若目标已经存在则拒绝覆盖。仅加载可信官方检查点及其归一化文件。

```bash
python3 - <<'PY'
import hashlib, json, zipfile
from pathlib import Path
base = Path('workspace')
meta = json.loads(Path('results/ms700_summary.json').read_text())
for name in ('LocomotionFull', 'LocomotionLegs'):
    archive = base / 'assets' / (name + '.zip')
    expected = meta['asset_sha256']['assets/' + archive.name]
    if hashlib.sha256(archive.read_bytes()).hexdigest() != expected:
        raise SystemExit('SHA256 mismatch: ' + str(archive))
    if (base / 'assets' / name).exists():
        raise SystemExit('Refusing to overwrite: ' + name)
    with zipfile.ZipFile(archive) as z:
        z.extractall(base / 'assets')
    print('Verified and extracted:', name)
PY
```

解压后应有：

```text
workspace/
  sources/msgym/                         固定提交的官方源码
  assets/LocomotionFull/
    locomotionFull.json
    checkpoint/best_model.zip            策略
    checkpoint/best_env.zip              观测归一化统计
  assets/LocomotionLegs/
    locomotionLegs.json
    checkpoint/best_model.zip
    checkpoint/best_env.zip
```

## 3. 独立 Python 环境

已运行的核心版本：Python 3.12.15、MuJoCo 3.4.0、CPU PyTorch 2.6.0、SB3 2.7.1、Gymnasium 1.2.3、NumPy 2.4.2。`requirements-ms700.txt` 是本机包版本记录，已去掉个人绝对路径，不是所有操作系统的通用安装保证。

```bash
python3.12 -m venv .venv_ms700
.venv_ms700/bin/python -m pip install torch==2.6.0+cpu \
  --index-url https://download.pytorch.org/whl/cpu
.venv_ms700/bin/python -m pip install -r requirements-ms700.txt
.venv_ms700/bin/python -m pip install --no-deps -e workspace/sources/msgym
.venv_ms700/bin/python -m pip check
```

安装时若某个固定版本不可取得或发生冲突，保留错误并核对版本，不直接升级原研究环境。训练检查点的软件栈和本机 CPU 推理栈不完全相同，不能宣称已复刻训练硬件。

## 4. 打开实时窗口

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MUJOCO_GL=glfw \
  .venv_ms700/bin/python scripts/run_ms700.py \
  --model full --seconds 10 --seed 0 --window
```

看下肢模型，把 `--model full` 换成 `--model legs`。

**预期：** 窗口显示肌肉驱动的运动；默认三周期约 3.52 s 后结束，终端输出 `reason: cycle_limit`、`kinematic_play: false`。`--seconds 10` 是上限，不保证运行满 10 秒。若没有图形桌面或 GLFW 无法创建窗口，应先解决显示问题；程序不会改成参考回放。

默认是确定性动作。需要官方采样模式时添加 `--stochastic`；它与均值模式的实验条件不同。

## 5. 不开窗口的短测、录像和输出

```bash
# 10 个控制步，验证依赖与真实动作链
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  .venv_ms700/bin/python scripts/run_ms700.py --model full --seconds 0.2

# 需要本机 EGL 与编码器支持；此项失败不能记作录像通过
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MUJOCO_GL=egl \
  .venv_ms700/bin/python scripts/run_ms700.py --model full --seconds 10 --video
```

输出默认位于 `runs/<model>_<时间戳>/`。也可指定新的 `--output` 路径；已有目录不覆盖。

| 输出 | 内容 |
|---|---|
| `summary.json` | 控制次数、真实时间、结束原因、版本相关哈希、跟踪误差 |
| `states.npz` | Actor 输出、动作、ctrl、act、肌肉力、qpos/qvel、参考目标、骨盆、足接触力 |
| `joints.csv` | 时间 s、六关节角 rad / 速度 rad/s、骨盆位置 m、足部法向力 N |
| `simulation.mp4` | 仅启用 `--video` 时生成 |
| `failure.json` / `partial_states.npz` | 发生异常时的错误与已记录步骤（如有） |

兼容原记录器，运行输出中的 `model_sha256` 指的是**策略检查点 ZIP**，不是物理模型哈希；分享摘要已经改名为 `policy_checkpoint_sha256`。足法向力是接触记录，不自动等于已提取步态事件。

已有官方工作目录时，可设置 `MS700_WORKSPACE=/absolute/path/to/existing/workspace`，其中应有 `sources/msgym/` 与 `assets/`；仍使用该目录对应的兼容 Python 解释器，不修改旧环境。

## 6. 长时试验边界

`--seconds 30 --cycles 100` 只扩大回合上限。已记录结果为全身 3.64 s、下肢 3.88 s 跟踪误差终止，不能用该命令保证长时行走。本次分享不删除终止条件、不加入支撑外力、不进行重新训练。
