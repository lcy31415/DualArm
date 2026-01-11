## 问题诊断
- Detect.py 与 ArmControl.py、main.py 并发写入 data/status.json，Windows 在目标文件被其他进程打开时拒绝 os.replace 导致 PermissionError。
- 目前三个进程都在调用 save_status，存在竞态条件；Detect 在捕获阶段频繁调用 save_status 加剧冲突。

## 解决方案（两步并行实施）
### 1) 引入轻量级文件锁，保证原子写
- 在 Block_Palletizing/common_io.py 新增通用 I/O：
  - acquire_lock(path)：使用 lock 文件（path+".lock"），通过 os.open(O_CREAT|O_EXCL) 原子抢占；带超时与重试
  - release_lock(lock_path)：写完后删除 lock 文件
  - safe_write_json(path, obj)：写入到 path.tmp，获取锁后执行 os.replace(path.tmp, path)
- 修改三处写操作统一使用 safe_write_json：
  - save_status（Detect、ArmControl、main）
  - write_blocks（Detect）
- 在 Windows 上可显著降低 PermissionError；仍保留短暂重试/backoff（例如 50ms 间隔，2s 超时）。

### 2) 优化职责，尽量减少写冲突
- 主进程 main 成为唯一的 status.json 写入者：
  - main 负责阶段切换：detect_state、arm_state、cycle、request_next
  - Detect 与 ArmControl 仅读取 status.json；自身状态用事件文件向 main 汇报
- 事件文件（Detect/ArmControl→main）：
  - data/detect_ready.flag：Detect 已就绪
  - data/detect_done.flag：Detect 已写 blocks_<cycle>.json 完成
  - data/arm_ready.flag：ArmControl 已到停止位
  - data/arm_done.flag：ArmControl 分拣完成
- main 轮询这些 flag 文件，进行 status.json 的更新与阶段推进；删除已消费的 flag。
- 好处：从根本上减少对 status.json 的多写，避免潜在竞态。

## 具体改动清单
- 新增 Block_Palletizing/common_io.py（安全写/锁封装）
- 重构：
  - Detect.py：将 save_status 改为事件文件；仅在写 blocks 时用 safe_write_json
  - ArmControl.py：不写 status.json，改写 arm_ready/arm_done flag；其余不变
  - main.py：统一管理 status.json（safe_write_json），消费 flag 推进状态机

## 验证流程
- 在 DobotMagician 目录注入环境后运行 main.py
- 观察：
  - 不再出现 PermissionError；Detect 预览常开
  - main 能正确推进：ready→capturing→done→busy→done→下一轮
- 人工断开/慢速读写情况下，锁与重试仍能正常运行。

## 兼容与后续
- 不引入第三方库，纯标准库实现；与现有 npz/cv/DLL 导入逻辑不冲突
- 若需要进一步可靠性（跨平台文件锁），可后续引入 portalocker，但当前实现满足需求

## 请求确认
- 我将按上述方案进行最小改动：新增 common_io.py、调整三个脚本的写入职责与锁；完成后在终端重新运行并反馈日志结果。