# 模型来源与许可

模型文件及 SHA-256 以 [config/model_manifest.yaml](config/model_manifest.yaml) 为准。
应用源码使用 [Apache-2.0](LICENSE)，版权及第三方声明集中在 [NOTICE](NOTICE)；第三方权重保留其原许可，不能因接入本应用而改标为本项目自训模型。
下表中的项目自训权重统一使用根目录的 `LICENSE`，无需为每个 ONNX 重复附许可证。

| 文件 | 权重来源 | 权重许可 / 状态 |
| --- | --- | --- |
| `1.go2_blind_rough.onnx` | 项目自行训练，平地模型续训粗糙路面，`model_5999.pt` | 项目自训，Apache-2.0 |
| `2.go2_terrain.onnx` | Tianyang Wu / wty-yy，`go2_moe_cts_137k_0.6713` | MIT，保留原作者版权声明和许可 |
| `11.go2_forward_jump.onnx` | 项目自行训练，`landing_v2_candidate`，`model_200.pt` | 项目自训，Apache-2.0 |
| `10.go2_wave.onnx` | 项目自行训练，命令招手鲁棒性 V2，`model_1999.pt` | 项目自训，Apache-2.0 |
| `12.go2_sit.onnx` | 项目自行训练，指令坐下与起身恢复，`model_4999.pt` | 项目自训，Apache-2.0 |

导航和 AI 模式复用平地行走权重 `1.go2_blind_rough.onnx`，没有额外权重。

## 综合地形

训练项目为 [wty-yy/go2_rl_gym](https://github.com/wty-yy/go2_rl_gym/tree/30e74dc507bec)。
当前综合地形 ONNX 来自 [unitree_cpp_deploy 的模型目录](https://github.com/wty-yy/unitree_cpp_deploy/tree/a422ec4c5268/logs/go2)：

- 综合地形：`go2_moe_cts_137k_0.6713/exported/policy.onnx`。

本项目仅重命名上述文件用于部署，文件内容与该版本上游模型一致。作者的
[模型发布页](https://huggingface.co/wty-yy/go2_rl_gym_data) 标注权重采用 MIT。
版权人为原作者；模型分发材料需保留相应版权声明及许可文本。

原始声明保存在 [LICENSES/wty-yy-go2-rl-gym.txt](LICENSES/wty-yy-go2-rl-gym.txt)
和 [LICENSES/wty-yy-unitree-cpp-deploy.txt](LICENSES/wty-yy-unitree-cpp-deploy.txt)。
其中还保留了上游随附的 Unitree 派生代码声明，不表示所有训练代码许可自动适用于权重。

相关研究为 Tianyang Wu 等人的 *Toward Reliable Sim-to-Real Predictability for
MoE-based Robust Quadrupedal Locomotion*，Robotics: Science and Systems 2026，
[DOI: 10.15607/RSS.2026.XXII.156](https://doi.org/10.15607/RSS.2026.XXII.156)。

## 自训模型

平地行走、前跳、招手和坐下模型的训练工程、checkpoint 及导出记录维护于
[unitree-go2-simple-train](https://github.com/spideman2/unitree-go2-simple-train)。
这些是项目自行训练的权重，训练代码使用或参考了
[Unitree RL Lab](https://github.com/unitreerobotics/unitree_rl_lab) 和 Isaac Lab。
训练代码的许可参考保存在 [LICENSES/unitree-rl-lab.txt](LICENSES/unitree-rl-lab.txt)，
该文件说明训练代码的来源，不作为这些自训权重的额外许可条件。

模型出处分别为：

- 平地行走：`self_train/1.locomotion/2.robust_locomotion/2.go2_blind_rough`，提交 `c9872e8`，`model_5999.pt`；无地形感知输入。
- 前跳：`isaaclab_separate/16.go2_jump_new_train`，提交 `adcf98d` 的 `landing_v2_candidate/model_200`。
- 坐下：`isaaclab_separate/15.go2_sit_train/experiments/go2_sit_command_recovery_v1`，提交 `b07eed4`，`model_4999.pt`。
- 招手：`isaaclab_separate/11.go2_wave_train/models/go2_command_wave/robust_v2/20261022_iter1999`，提交 `9793044`。

分发训练代码或第三方机器人资源时，还需保留对应工程和资源随附的声明。

## 分发材料

模型包只保存当前部署的 ONNX 文件。模型来源和许可声明统一保留在本代码仓库的
`MODEL_SOURCES.md`、`NOTICE`、`LICENSE` 与 `LICENSES/`，随 SDK 与模型配套提供；配置和参考动作也由代码仓库维护。
训练代码及第三方机器人资源的许可保留在各自源码工程中。
替换模型后同步更新来源、checkpoint、文件大小和 SHA-256，声明以实际部署的权重为准。
