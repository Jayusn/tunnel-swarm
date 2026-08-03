# LDOP 一键启动脚本 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在现有雷达定位脚本的基础上增加 LDOP 检测启动标签页，并关闭 Fast-LIO 自带 RViz，使一次命令完成雷达定位、LDOP 检测和 LDOP RViz 启动。

**Architecture:** 保留 `ros_start_lidar_only.sh` 作为四个雷达定位标签页的唯一来源；新增 `ros_start_ldop_detection.sh` 作为包装器，调用前者后打开第五个 LDOP 标签页。LDOP 标签页加载 LDOP 环境，并用 `/cloud_registered` 与 `/Odometry` 的实际消息作为启动条件，避免 LDOP 早于输入链路启动。

**Tech Stack:** Bash、ROS1 `roslaunch`/`rostopic`、GNOME Terminal、Fast-LIO、LDOP catkin 工作空间、Python `unittest` 静态验收脚本、PowerShell `scp`/SSH 同步。

## Global Constraints

- 不重新安装依赖；不在启动脚本中执行 `catkin build`。
- 本地仓库：`E:/比赛功能包/LDOP`；远程源码：`/home/oem/ldop_ws/src/LDOP`。
- 远程脚本目录：`/home/oem/scripts`；本地脚本目录：`E:/比赛功能包/LDOP/scripts`。
- 每次本地脚本或测试变更后，使用 SSH key `C:/Users/Jayus/.ssh/id_ed25519_codex_guet2` 同步到远程对应路径。
- Fast-LIO 必须使用 `roslaunch fast_lio mapping_mid360.launch rviz:=false`；只有 `roslaunch ldop run_ldop.launch` 负责启动 RViz。
- 远程用户和地址：`oem@192.168.31.163`。
- Git 提交备注使用中文。

---

### Task 1: 添加启动脚本静态验收测试

**Files:**
- Create: `test/test_startup_scripts.py`

**Interfaces:**
- Consumes: `scripts/ros_start_lidar_only.sh` 和 `scripts/ros_start_ldop_detection.sh` 的文本内容。
- Produces: 一个可用 `python3 test/test_startup_scripts.py` 执行的静态验收入口，验证文件存在、Fast-LIO 禁用 RViz、包装器复用现有雷达脚本并等待两个关键话题。

- [ ] **Step 1: Write the failing test**

创建以下测试。由于两个脚本尚未创建，第一次运行必须因文件不存在而失败：

```python
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
LIDAR_SCRIPT = ROOT / "scripts" / "ros_start_lidar_only.sh"
LDOP_SCRIPT = ROOT / "scripts" / "ros_start_ldop_detection.sh"


class StartupScriptTest(unittest.TestCase):
    def test_startup_scripts_exist(self):
        self.assertTrue(LIDAR_SCRIPT.is_file())
        self.assertTrue(LDOP_SCRIPT.is_file())

    def test_lidar_script_disables_fastlio_rviz(self):
        text = LIDAR_SCRIPT.read_text(encoding="utf-8")
        self.assertIn(
            "roslaunch fast_lio mapping_mid360.launch rviz:=false",
            text,
        )
        self.assertNotIn(
            "roslaunch fast_lio mapping_mid360.launch\"",
            text,
        )

    def test_ldop_script_wraps_lidar_and_waits_for_topics(self):
        text = LDOP_SCRIPT.read_text(encoding="utf-8")
        self.assertIn("ros_start_lidar_only.sh", text)
        self.assertIn("/home/oem/ldop_ws/ldop_env.sh", text)
        self.assertIn("/cloud_registered", text)
        self.assertIn("/Odometry", text)
        self.assertIn("roslaunch ldop run_ldop.launch", text)
        self.assertNotIn("mapping_mid360.launch", text)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run from `E:/比赛功能包/LDOP`:

```powershell
python test/test_startup_scripts.py
```

Expected: FAIL because `scripts/ros_start_lidar_only.sh` and `scripts/ros_start_ldop_detection.sh` do not yet exist.

- [ ] **Step 3: Sync the failing test to the remote checkout**

Create the remote test directory if necessary and copy the exact file:

```powershell
ssh -i "C:/Users/Jayus/.ssh/id_ed25519_codex_guet2" oem@192.168.31.163 "mkdir -p /home/oem/ldop_ws/src/LDOP/test"
scp -i "C:/Users/Jayus/.ssh/id_ed25519_codex_guet2" "E:/比赛功能包/LDOP/test/test_startup_scripts.py" "oem@192.168.31.163:/home/oem/ldop_ws/src/LDOP/test/test_startup_scripts.py"
```

### Task 2: 修改雷达脚本并实现 LDOP 包装器

**Files:**
- Create: `scripts/ros_start_lidar_only.sh`
- Create: `scripts/ros_start_ldop_detection.sh`

**Interfaces:**
- Consumes: 现有远程 `/home/oem/scripts/ros_start_lidar_only.sh` 的四个启动项、`$HOME/rh_ws`、`/home/oem/ldop_ws/ldop_env.sh`。
- Produces: `ros_start_lidar_only.sh` 启动 Livox、MAVROS、无 RViz 的 Fast-LIO 和定位桥接；`ros_start_ldop_detection.sh` 额外打开 LDOP 标签页并在两个输入话题有消息后启动 LDOP。

- [ ] **Step 1: Copy the current lidar script into the local repository**

将远程现有脚本复制到本地 `scripts/ros_start_lidar_only.sh`，保留四个标签页、工作空间和命令，只把 Fast-LIO 命令改为：

```bash
start_terminal "3_Start fastlio2" "roslaunch fast_lio mapping_mid360.launch rviz:=false"
```

不要添加第二个 RViz 启动命令，也不要改动 Livox、MAVROS 或 `fastlio_fusion` 命令。

- [ ] **Step 2: Add the wrapper script**

创建 `scripts/ros_start_ldop_detection.sh`，内容应遵循以下接口和流程：

```bash
#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LIDAR_SCRIPT="${SCRIPT_DIR}/ros_start_lidar_only.sh"
LDOP_ENV="/home/oem/ldop_ws/ldop_env.sh"

if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "错误：找不到 gnome-terminal。" >&2
    exit 1
fi
if ! command -v timeout >/dev/null 2>&1; then
    echo "错误：找不到 timeout，无法执行带超时的话题等待。" >&2
    exit 1
fi
if [[ ! -x "${LIDAR_SCRIPT}" ]]; then
    echo "错误：雷达启动脚本不存在或不可执行：${LIDAR_SCRIPT}" >&2
    exit 1
fi
if [[ ! -f "${LDOP_ENV}" ]]; then
    echo "错误：LDOP 环境脚本不存在：${LDOP_ENV}" >&2
    exit 1
fi

"${LIDAR_SCRIPT}"

gnome-terminal --tab --title="5_LDOP" -- bash -c '
    set -uo pipefail
    trap "exit 130" INT TERM
    if ! source "/home/oem/ldop_ws/ldop_env.sh"; then
        echo "错误：加载 LDOP 环境失败。"
        exec bash
    fi

    wait_for_topic() {
        local topic="$1"
        echo "等待 ${topic} 发布实际消息..."
        until timeout 3s rostopic echo -n 1 "${topic}" >/dev/null 2>&1; do
            sleep 1
        done
        echo "${topic} 已有实际消息。"
    }

    wait_for_topic /cloud_registered
    wait_for_topic /Odometry
    echo "========== 5_LDOP =========="
    echo "roslaunch ldop run_ldop.launch"
    roslaunch ldop run_ldop.launch
    exec bash
'
```

实现时保持单引号包裹的终端命令，确保函数参数和 ROS 命令在第五个终端中展开；不要在主脚本中预先执行 LDOP 的 `roslaunch`，否则节点会在新终端之外运行。

- [ ] **Step 3: Set executable permissions locally**

在本地仓库执行：

```powershell
git update-index --chmod=+x scripts/ros_start_lidar_only.sh scripts/ros_start_ldop_detection.sh
```

随后使用 Git Bash 或远程 Linux 端确认两个文件的 Unix mode 为 `100755`；Windows 文件系统不保留传统 Unix 权限时，以 Git index 和远程 `chmod +x` 的结果为准。

- [ ] **Step 4: Run the static test to verify it passes**

Run from `E:/比赛功能包/LDOP`:

```powershell
python test/test_startup_scripts.py
```

Expected: `Ran 3 tests` and `OK`。

### Task 3: 同步脚本与验收测试到远程端

**Files:**
- Modify: `/home/oem/scripts/ros_start_lidar_only.sh`
- Create: `/home/oem/scripts/ros_start_ldop_detection.sh`
- Modify: `/home/oem/ldop_ws/src/LDOP/test/test_startup_scripts.py`

**Interfaces:**
- Consumes: Task 2 中本地仓库的两个脚本和 Task 1 中的测试。
- Produces: 远程脚本目录中可直接执行的两个脚本，以及远程源码中的同步验收测试。

- [ ] **Step 1: Copy both scripts to the remote script directory**

执行：

```powershell
scp -i "C:/Users/Jayus/.ssh/id_ed25519_codex_guet2" "E:/比赛功能包/LDOP/scripts/ros_start_lidar_only.sh" "oem@192.168.31.163:/home/oem/scripts/ros_start_lidar_only.sh"
scp -i "C:/Users/Jayus/.ssh/id_ed25519_codex_guet2" "E:/比赛功能包/LDOP/scripts/ros_start_ldop_detection.sh" "oem@192.168.31.163:/home/oem/scripts/ros_start_ldop_detection.sh"
```

- [ ] **Step 2: Set remote executable permissions**

执行：

```powershell
ssh -i "C:/Users/Jayus/.ssh/id_ed25519_codex_guet2" oem@192.168.31.163 "chmod +x /home/oem/scripts/ros_start_lidar_only.sh /home/oem/scripts/ros_start_ldop_detection.sh"
```

- [ ] **Step 3: Copy the static test to the remote LDOP source**

执行：

```powershell
scp -i "C:/Users/Jayus/.ssh/id_ed25519_codex_guet2" "E:/比赛功能包/LDOP/test/test_startup_scripts.py" "oem@192.168.31.163:/home/oem/ldop_ws/src/LDOP/test/test_startup_scripts.py"
```

### Task 4: 运行静态、语法和内容一致性验证

**Files:**
- Test: `test/test_startup_scripts.py`
- Test: `scripts/ros_start_lidar_only.sh`
- Test: `scripts/ros_start_ldop_detection.sh`

**Interfaces:**
- Consumes: 本地和远程同步后的脚本。
- Produces: 可审计的测试输出、Shell 语法结果、权限结果和 SHA-256 一致性结果。

- [ ] **Step 1: Run local Python static tests and shell syntax checks**

```powershell
python test/test_startup_scripts.py
bash -n scripts/ros_start_lidar_only.sh
bash -n scripts/ros_start_ldop_detection.sh
git diff --check
```

Expected: Python 输出 `OK`，两个 `bash -n` 和 `git diff --check` 均返回退出码 0。

- [ ] **Step 2: Compare local and remote SHA-256 hashes**

```powershell
Get-FileHash -Algorithm SHA256 "E:/比赛功能包/LDOP/scripts/ros_start_lidar_only.sh"
Get-FileHash -Algorithm SHA256 "E:/比赛功能包/LDOP/scripts/ros_start_ldop_detection.sh"
ssh -i "C:/Users/Jayus/.ssh/id_ed25519_codex_guet2" oem@192.168.31.163 "sha256sum /home/oem/scripts/ros_start_lidar_only.sh /home/oem/scripts/ros_start_ldop_detection.sh"
```

Expected: 对应文件的哈希值完全一致。

- [ ] **Step 3: Confirm remote executable mode and static test**

```powershell
ssh -i "C:/Users/Jayus/.ssh/id_ed25519_codex_guet2" oem@192.168.31.163 "stat -c '%A %a %n' /home/oem/scripts/ros_start_lidar_only.sh /home/oem/scripts/ros_start_ldop_detection.sh"
ssh -i "C:/Users/Jayus/.ssh/id_ed25519_codex_guet2" oem@192.168.31.163 "cd /home/oem/ldop_ws/src/LDOP && python3 test/test_startup_scripts.py"
```

Expected: 两个脚本为 `-rwxr-xr-x 755`，远程测试输出 `OK`。

### Task 5: 从空闲 ROS 状态验证完整启动

**Files:**
- Test: `/home/oem/scripts/ros_start_ldop_detection.sh`
- Test: `/home/oem/scripts/ros_start_lidar_only.sh`

**Interfaces:**
- Consumes: 已通过静态和语法检查的远程脚本。
- Produces: 运行时节点、话题和 RViz 进程的验证证据。

- [ ] **Step 1: Inspect current ROS state before launching**

先检查现有 ROS 节点和关键进程，避免在已有链路上重复启动：

```powershell
ssh -i "C:/Users/Jayus/.ssh/id_ed25519_codex_guet2" oem@192.168.31.163 "pgrep -af 'roslaunch|fastlio_mapping|ldop_node|rviz' || true"
```

如果已有同一套链路运行，先记录状态并使用 `/home/oem/scripts/ros_stop_all.sh` 清理后再做一次完整启动验证；不重复启动同名节点。

- [ ] **Step 2: Run the wrapper from the remote scripts directory**

在带有图形会话的远程终端中执行：

```bash
cd /home/oem/scripts
./ros_start_ldop_detection.sh
```

预期打开四个雷达定位终端标签页和一个 `5_LDOP` 标签页。LDOP 标签页应先显示等待 `/cloud_registered` 与 `/Odometry` 的状态，收到实际消息后执行 `roslaunch ldop run_ldop.launch`。

- [ ] **Step 3: Verify nodes and topics**

```bash
rosnode list | grep -E '^/(laserMapping|ldop_node|ldop_rviz)$'
rostopic hz /cloud_registered
rostopic hz /Odometry
rostopic hz /ldop/dynamic_object_markers
```

预期存在 `/laserMapping`、`/ldop_node`、`/ldop_rviz`，三个话题均持续有数据；LDOP RViz 存在，Fast-LIO RViz 不应额外存在。

- [ ] **Step 4: Stop the stack using the existing stop script**

```bash
/home/oem/scripts/ros_stop_all.sh
```

确认 ROS 启动进程退出；不修改或扩大 `ros_stop_all.sh` 的清理行为。

### Task 6: 提交本地变更并完成最终同步

**Files:**
- Modify: `scripts/ros_start_lidar_only.sh`
- Create: `scripts/ros_start_ldop_detection.sh`
- Create: `test/test_startup_scripts.py`

**Interfaces:**
- Consumes: Task 1–5 的测试和验证结果。
- Produces: 一个中文 Git 提交，以及本地与远程脚本内容一致的最终状态。

- [ ] **Step 1: Review the final diff**

```powershell
git diff -- scripts test
git status --short
```

确认只包含启动脚本、静态验收测试和本次需要的权限变化，不包含依赖安装、编译产物或无关文件。

- [ ] **Step 2: Stage and commit with a Chinese message**

```powershell
git add -- scripts/ros_start_lidar_only.sh scripts/ros_start_ldop_detection.sh test/test_startup_scripts.py
git commit -m "脚本：新增 LDOP 一键启动"
```

- [ ] **Step 3: Verify the commit and clean worktree**

```powershell
git log -1 --oneline
git status --short --branch
```

Expected: 最新提交备注为中文，工作区干净；远程两个脚本的 SHA-256 仍与本地一致。

## Self-Review Checklist

- 设计文档中的方案 A 已覆盖：包装现有雷达脚本、第五个 LDOP 标签页、两个话题实际消息等待和仅保留 LDOP RViz。
- 错误处理已覆盖：检查 `gnome-terminal`、雷达脚本、LDOP 环境脚本，并对等待过程支持 `Ctrl+C`。
- 非目标已覆盖：不编译、不安装依赖、不改算法、不修改停止脚本。
- 本地/远程同步已拆成明确步骤，包含脚本、测试、权限和 SHA-256 验证。
- 所有提交命令使用中文备注；计划中没有占位符或未定义的接口。
