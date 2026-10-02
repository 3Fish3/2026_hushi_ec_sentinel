#include "ec/channel.hpp"
#include "ec/drivers/dji_device.hpp"
#include "ec/platform/stm32f407.hpp"

// 对 Cortex-M4 实际实例化模板, 不以只编译未使用的头文件冒充嵌入式兼容性验证.
// 本文件没有全局实例、任务或硬件副作用; 未被固件使用的实例由链接器裁剪.
namespace ec {
template class LatestValue<Sample<float>, stm32f407::IrqCriticalSection>;
template class LatestValue<drivers::DjiDeviceState, stm32f407::IrqCriticalSection>;
template class LatestValue<Command<float>, stm32f407::IrqCriticalSection>;
template class CommandMailbox<float, stm32f407::IrqCriticalSection>;
template class EventQueue<Command<float>, 8, stm32f407::IrqCriticalSection>;
} // 命名空间 ec
