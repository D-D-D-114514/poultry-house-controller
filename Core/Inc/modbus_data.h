#ifndef MODBUS_DATA_H
#define MODBUS_DATA_H

/* Modbus 寄存器 ↔ 系统状态同步 */

void MODBUS_DATA_SyncRegisters(void);   // 读传感器 → 更新寄存器
void MODBUS_DATA_ApplyWrites(void);     // 检查写入寄存器 → 应用到硬件

#endif
