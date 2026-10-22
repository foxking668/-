#pragma once

/*
 * 赛道元素检测开关配置文件。
 *
 * 当前启用了：
 * - 斑马线元素检测模块；
 * - 斑马线元素的内部符号编号定义。
 *
 * 这些宏会在 `FixLine` 模块中控制相关功能是否参与编译。
 */
#define FIXLINE_INCLUDE_ELEMENTS_ZEBRA
#define FIXLINE_INCLUDE_ZEBRA_SYMBOL 3
