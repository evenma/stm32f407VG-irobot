/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief 浮点数格式化辅助函数
 */

#ifndef PRINT_UTILS_H__
#define PRINT_UTILS_H__

#include <rtthread.h>

/**
 * @brief 将浮点数格式化为字符串并打印
 * @param value   要打印的浮点数
 * @param width   总宽度（包含符号、整数、小数点和精度），不足则右对齐空格填充
 * @param precision 小数位数（0-6）
 * @param sign    是否显示正号（1: 显示+号，0: 负数自动显示-号，正数不显示+）
 */
void print_float(float value, int width, int precision, int sign);

/**
 * @brief 将浮点数格式化为字符串（不打印）
 * @param value     要格式化的浮点数
 * @param buf       输出缓冲区（至少 32 字节）
 * @param width     总宽度（右对齐，0 表示无宽度控制）
 * @param precision 小数位数（0-6）
 * @param sign      是否显示正号（1: 显示+号，0: 负数自动显示-号，正数不显示+）
 */
void format_float(float value, char *buf, int width, int precision, int sign);

#endif /* PRINT_UTILS_H__ */
