#ifndef EC_BOARD_REGISTRY_H
#define EC_BOARD_REGISTRY_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
// 正式 APP 角色的应用状态观测来源 (布局见 board_app_observation.h): 注册一次,
// 之后每次 ec_registry_publish 会把它展开为 source/axis/device 记录.
// 只有正式 APP 角色定义并调用它; 其它角色不链接本入口.
struct EcAppObservation;
void ec_registry_publish_app(const struct EcAppObservation* observation);
// SWD 只读定位入口: 固定描述符 (ec::monitor::Descriptor 布局, 40 字节, 无指针).
// 布局版本/记录尺寸/容量与记录表地址都写在描述符里, 读取方按 ELF 符号定位本符号后
// 先核对 magic/layout/record_size 再读记录.
void ec_registry_init(void);
// 每个控制周期调用一次; 编译期关闭监控注册表时为空实现 (不保留轮询与编码开销).
void ec_registry_publish(uint32_t now_ms);
// 编码一页记录供串口监控服务使用; 返回写入字节数, 0 表示注册表未启用或参数越界.
size_t ec_registry_encode_page(uint16_t first_index, uint8_t* out, size_t capacity);
#ifdef __cplusplus
}
#endif
#endif
