# 底盘＋云台跨机器Agent交接

先读 `source/docs/guides/Agent-底盘云台联调.md`, 工作根目录为 `source/`.

## 当前联调状态

2026-11-11 main_yaw 已按速度PID `0.4/0.35/0`（3A量程归一化值）、位置PID `6.3/0.45/0` 固化到普通联调固件, 完成烧录及实车验收. 参数依据、部署记录和验证边界见 [PID定案与实车验收](results/integration-main035/FINAL.md); 当前配置见 `source/boards/sentinel_f407/include/board_app_integration_config.hpp`.

## 历史交付与校验

原始交付的两份固件放在 `source/build/`对应角色目录, 含ELF/BIN/HEX/MAP; 不含SDK、工具链、环境或依赖安装包. 这些构建产物和原始采集不随源码纳入Git.
`PACKAGE.json`、`SOURCE_BASELINE.json`和`SHA256SUMS.txt`记录收到的原始交付身份与基线, 不代表当前修改后源码或最新固件的校验值; 保留原清单, 不用当前结果覆盖它们. 本机Agent规则未携带, 原工程README中本地参考不属于交付内容.
原始交付的输入比例和重装IMU几何为空; 2026-11-02按当时实录回填保守现场候选, 当时仅编译、未烧录.
2026-11-02候选、原始质量证据、离线回放及构建结果见 `results/REPORT.md`; 后续main_yaw定案以第一节链接的验收报告为准.

保留收到的原ZIP. 从本目录校验和回传(仅Python标准库, 不需要Git):

```text
python source/boards/sentinel_f407/tools/integration_handoff.py verify <收到的原ZIP>
python source/boards/sentinel_f407/tools/integration_handoff.py return --baseline <收到的原ZIP> --source source --results results --output RETURN-sentinel-integration-<日期>.zip
```

回传前整理 `results/REPORT.md`、日志、标定和新固件. 源码改动放在 `source/`, 仅建议也在报告中写明.
回传自动包含新增/修改/删除清单、完整修改文件、文本patch和全部results; 无需重传未改源码与环境.
