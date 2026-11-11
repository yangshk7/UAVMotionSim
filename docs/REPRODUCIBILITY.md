# v1.1.0 复现与新机器部署

目标是按固定版本和配置重新得到可验收的仿真结果，不要求跨操作系统的每个浮点位或二进制文件逐字节相同。原始随机种子、配置、坐标、单位、来源与验收指标必须保留。

## 1. 获取固定源码与Python环境

```powershell
git clone --branch v1.1.0 https://github.com/yangshk7/UAVMotionSim.git
cd UAVMotionSim
powershell -ExecutionPolicy Bypass -File scripts/setup.ps1
uv lock --check
uv run --no-sync python scripts/reproduce_baseline.py
uv run --no-sync pytest -q
```

先安装uv（本版使用0.12.1）。`.python-version`和setup固定Python **3.11.15**，`uv.lock`固定完整Python依赖；不要用手工安装的“最新版”替代锁文件。setup会联网下载Python和依赖。`requirements.lock`是由uv.lock导出的Python3.11辅助清单，包含RotorPy全部运行/开发依赖；uv.lock为环境权威来源。构建后端固定setuptools80.9.0。Windows上位机还需要系统Tk/.NET支持；`main/UAVMotionSim.exe`是启动器，必须保留在完整项目中。

解析复现基准使用实际RotorPy、固定双航次与2 s无扰动直线，独立检查APC位置与解析参考的最大差不超过`1e-8 m`，同时检查状态/时间与标准导出。结果在新`results/reproduction_*`目录。它是部署检查，不代替完整科学或飞控验收。

### 网络受限时的同版本安装

如果官方文件服务器超时，可在已安装Python3.11.15后使用以下备用流程。下载器逐项核验`uv.lock`原SHA256，构建后端使用`deployment/build_backend.lock.json`的固定哈希，不允许镜像替换依赖内容。先用`uv venv --python 3.11.15`创建虚拟环境，再执行（Windows使用`.venv/Scripts/python.exe`，Linux改为`.venv/bin/python`）：

```powershell
uv run --no-project --python 3.11.15 python scripts/fetch_locked_wheels.py --mirror https://mirrors.aliyun.com/pypi --output wheelhouse
uv pip sync --python .venv/Scripts/python.exe --no-index --find-links wheelhouse requirements.lock
uv pip install --python .venv/Scripts/python.exe --no-index --find-links wheelhouse setuptools==80.9.0
uv pip install --python .venv/Scripts/python.exe --no-deps --no-build-isolation --editable .
uv pip check --python .venv/Scripts/python.exe
uv run --no-sync python scripts/reproduce_baseline.py
```

本版已在独立Linux源码目录和新Python3.11.15虚拟环境中按该流程安装并通过双航次解析基准；依赖下载使用原哈希校验的镜像文件。`--no-deps`仅用于已按完整锁文件安装并检查依赖后的本地源码安装，不是跳过依赖检查。该备用wheel下载器仅支持Python3.11 x86_64的Windows/Linux。

## 2. MATLAB

安装具有有效许可的MATLAB，并在上位机填写程序路径或将MATLAB加入PATH。无需安装MATLAB Python Engine；物理链路通过batch和MAT文件通信。原MATLAB物理源码已放入`matlab/legacy_uav`私有命名空间，**全链路验收不依赖项目旁的旧MATLAB工程或旧Python项目**。

旧“完整验收”里的历史MATLAB独立项目对照、既有SAR单点回波接口属于额外旁路，需要对应相邻工程；缺少时明确显示未执行。推荐用新的“全链路验收”复现本版定义的五链路适用检查。

## 3. 外部飞控：独立WSL2 Ubuntu 24.04 x86_64

先在Windows启用WSL2并安装Ubuntu 24.04发行版。建议使用专门的发行版，不把安装脚本直接运行在含重要业务软件的Linux环境中。发行版名称可在上位机配置，默认`Ubuntu-24.04`。

```powershell
powershell -ExecutionPolicy Bypass -File deployment/setup_wsl.ps1 -Distribution Ubuntu-24.04
powershell -ExecutionPolicy Bypass -File deployment/setup_wsl.ps1 -Distribution Ubuntu-24.04 -CheckOnly
```

安装脚本需要root、网络、磁盘空间和编译时间；`UAV_BUILD_JOBS`可限制构建并行度，默认2。它不会自动新建或删除WSL发行版，也不会重置已有源码；发现不一致的已存在提交会停止。

- `deployment/environment.lock.json`：PX4、XTDrone2、px4_msgs、Agent和Fast DDS完整提交号、SITL SHA256、Gazebo版本。
- `deployment/apt_packages.lock`：关键ROS/构建软件包的已验收版本。APT仓库必须仍提供这些版本；不可用时安装失败，不静默替换为最新版。
- `deployment/sitl/requirements.lock`、`deployment/px4/requirements.lock`：外部Python环境。
- `deployment/patches/`：Agent/Fast DDS构建兼容和XTDrone2 LAND修复，包含实际环境中的完整必要本地差异。
- SITL下载使用官方固定`stable-4.7.2`归档，并核验SHA256，不再指向滚动stable。
- 路径约定为`/opt/uav_motion`，源码由固定提交构建。系统/compiler/驱动差异可能改变二进制哈希，需以环境检查和飞行验收确认，而不是假定相同文件名即可。

`--check`只检查环境提交、补丁、关键包、二进制与运行文件，不等同于飞行通过。相关安装来源：[ROS 2 Jazzy官方步骤](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst)、[Gazebo Harmonic官方步骤](https://gazebosim.org/docs/harmonic/install_ubuntu/)、[ArduPilot 4.7.2固件归档](https://firmware.ardupilot.org/Copter/stable-4.7.2/SITL_x86_64_linux_gnu/)。

## 4. 复现正式验收

打开上位机，加载`docs/release_evidence/v1.1.0/accepted_config.yaml`，配置本机MATLAB路径和WSL发行版，然后点击“全链路验收”。五套链路分别跑双航次，整体只有全部必需检查通过才显示passed。入口不拿历史结果补齐失败或未执行的平台。1000 m/110 s全流程在验收机器上约45.9 min，其他机器不同。

预期得到5份子报告、整体报告、20组双航次链路间指标、SITL/PX4两航次原生同工况对照，以及各链路的高频真值与100 Hz导航。直接比较的位置、速度、姿态差异没有经过平移/对时拟合，也不被当作与实飞等价的门槛。外部平台不接收内部A/B扰动与控制器增益，限制详见上位机说明。

## 5. 验证边界与证据

最新正式结果以[正式验收记录](FINAL_VALIDATION.md)和`release_evidence/v1.1.0/`为准。发布整理已通过独立Linux源码目录/新Python环境的解析基准（两航次最大APC位置差均约2.775e-12 m，门槛1e-8 m），以及当前WSL的21项锁定清单检查。**整套WSL安装脚本尚未在全新发行版中完成从零构建与飞行验证**；版本仓库或网络不可用、WSL/驱动差异均可能阻止重建。出现这些问题应保留日志，不得更改种子、放宽验收指标或把新环境伪装成已验收环境。

源码仓库不携带MATLAB许可、WSL系统镜像或大体积原始结果。历史结果、用户可选研究配置和默认验收参数各有独立来源；`research_profiles`不是正式默认参数，也未替代本版验收配置。
