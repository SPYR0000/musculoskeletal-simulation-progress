# 分享范围、源码来源与归属

## 已提供的可运行内容

- MyoAssist 组合模型所需的 `compose.py`、`MyoAssistLegBase`、运行配置及数据工具。
- 配置化的 Dephy 两通道 TCP 服务与标准库客户端。
- ROS 2 Humble 桥接、真实反馈、同实例窗口和左右力矩演示。
- TCP / ROS 自动测试、零助力 / 右侧 / 左侧三组短时对照。
- MS-Human-700 官方策略的独立运行器及模型 / 权重获取说明。
- 安装、三终端启动、接口定义、图表、实测摘要与验证记录。

`rl_train/` 是支持本次 base 环境运行的必要源码子集，不是全部 MyoAssist 训练框架。除两份测试的解释器选择及元数据增加环境变量入口外，打包的 MyoAssist / TCP / ROS 运行代码是现有已测试源码的原样副本。逐文件哈希见 [runtime_source_provenance.json](../results/runtime_source_provenance.json)。原项目没有被改动。

MS-Human-700 运行器只调整工作与输出目录，不改变官方物理模型、策略或动作映射。MyoAssist 使用的 22 肌肉模型与 MS-Human-700 700/100 肌肉模型、MuscleMimic 模型相互独立，不能互换权重或数组索引。ROS 控制当前只适用于 MyoAssist + Dephy 路线。

## 获取方式与未分发内容

运行时模型通过固定版本 `myo-sim`、`assist-sim` 等官方包安装；MS-Human-700 源码、网格及检查点按官方渠道下载并检查哈希。仓库不携带虚拟环境或第三方大文件。

本次仍未分发 V1 原始数据 / GRU、健康受试者原始文件及受限片段、实验室髋关节样机 URDF/CAD/内部参数、完整训练框架及全部历史调试记录。此前模型、权重、ROS 代码与健康数据原件均保留。

自研髋关节样机需要完成模型组合、连接与执行器映射核验，尚不能以此包宣称已经支持该样机。

## 上游项目

| 来源 | 使用范围 |
|---|---|
| [MyoAssist](https://github.com/neumovelab/myoassist) | 人体与辅助设备组合、环境基类与运行配置；本地基线提交 `b1baf69da59dc6728515d0977cbc8fbf00b9225a` |
| [MyoSuite](https://github.com/MyoHub/myosuite) | MyoAssist 仿真底层；本路线 2.8.4，不是另一路研究用 3.0 环境 |
| [MS-Human-700](https://github.com/LNSGroup/MS-Human-700) | 全身和下肢肌骨模型 |
| [msgym](https://github.com/LNSGroup/msgym) | 官方环境和 DynSyn-SAC 实现 |
| [msgym 官方权重](https://github.com/LNSGroup/msgym/releases/tag/Checkpoints) | 策略与对应观测归一化文件 |
| [MuscleMimic](https://github.com/amathislab/musclemimic) | 既有工作概述来源，本次未分发其 runner / 权重 |
| [健康步态数据 DOI](https://doi.org/10.17632/3dmymnb65h.1) | 既有参考审计来源，本次不再分发数据 |

msgym 和模型提交、策略与归一化文件哈希在 [MS-Human-700 摘要](../results/ms700_summary.json)。模型和算法属于各自上游；本工作是环境复现、通信连接、状态记录、诊断及实验评价，没有把上游算法宣称为自行开发。

分享代码沿用原项目 Apache-2.0，见 [LICENSE](../LICENSE) 与 [NOTICE](../NOTICE)。第三方模型、数据、算法和权重仍遵循各自原始许可与引用要求。
