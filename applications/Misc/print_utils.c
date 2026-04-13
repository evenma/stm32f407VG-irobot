/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief 浮点数格式化辅助函数实现
 */

#include "print_utils.h"
#include <math.h>

void print_float(float value, int width, int precision, int sign)
{
    char buf[32];
    format_float(value, buf, width, precision, sign);
    rt_kprintf("%s", buf);
}

void format_float(float value, char *buf, int width, int precision, int sign)
{
    int integer_part, decimal_part;
    int negative = (value < 0);
    if (negative) value = -value;

    integer_part = (int)value;
    // 处理小数部分，防止浮点误差
    decimal_part = (int)((value - integer_part) * powf(10, precision) + 0.5f);
    if (decimal_part >= (int)powf(10, precision)) {
        decimal_part -= (int)powf(10, precision);
        integer_part++;
    }

    // 格式化字符串
    int len = rt_snprintf(buf, 32, "%s%d.%0*d",
                          (negative ? "-" : (sign ? "+" : "")),
                          integer_part,
                          precision,
                          decimal_part);
    // 如果需要右对齐且宽度 > len，则前移指针
    if (width > len && width > 0) {
        int pad = width - len;
        for (int i = len; i >= 0; i--) {
            buf[i + pad] = buf[i];
        }
        for (int i = 0; i < pad; i++) {
            buf[i] = ' ';
        }
    }
}
