/**
 * Copyright (c) [2025] XinChengYang <vertax@foxmail.com> <yaphetys@gmail.com>
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef __UTILS_COLORPRINT_HPP__
#define __UTILS_COLORPRINT_HPP__

#include <stdio.h>
#include <string.h>

#include <iostream>
#include <sstream>

/*
 * Make screen print colorful :)
 * Author: XinCheng Yang
 * Email: yaphetys@gmail.com
 * Related link:
 * [1] https://en.wikipedia.org/wiki/ANSI_escape_code
 * [2]
 * https://github.com/caffedrine/CPP_Utils/blob/597fe5200be87fa1db2d2aa5d8a07c3bc32a66cd/Utils/include/AnsiColors.h
 *
 */
const std::string _tools_color_printf_version = "V1.2";
const std::string _tools_color_printf_info = "[Enh]: Add delete lines, ANSI_SCREEN_FLUSH";

// clang-format off
#ifdef EMPTY_ANSI_COLORS
    #define ANSI_COLOR_RED ""
    #define ANSI_COLOR_RED_BOLD ""
    #define ANSI_COLOR_GREEN ""
    #define ANSI_COLOR_GREEN_BOLD ""
    #define ANSI_COLOR_YELLOW ""
    #define ANSI_COLOR_YELLOW_BOLD ""
    #define ANSI_COLOR_BLUE ""
    #define ANSI_COLOR_BLUE_BOLD ""
    #define ANSI_COLOR_MAGENTA ""
    #define ANSI_COLOR_z::ILog4zManager::getRef().setLoggerDisplay(LOG4Z_MAIN_LOGGER_ID, false);MAGENTA_BOLD ""
#else
    #define ANSI_COLOR_RED "\x1b[0;31m"
    #define ANSI_COLOR_RED_BOLD "\x1b[1;31m"
    #define ANSI_COLOR_RED_BG "\x1b[0;41m"

    #define ANSI_COLOR_GREEN "\x1b[0;32m"
    #define ANSI_COLOR_GREEN_BOLD "\x1b[1;32m"
    #define ANSI_COLOR_GREEN_BG "\x1b[0;42m"

    #define ANSI_COLOR_YELLOW "\x1b[0;33m"
    #define ANSI_COLOR_YELLOW_BOLD "\x1b[1;33m"
    #define ANSI_COLOR_YELLOW_BG "\x1b[0;43m"

    #define ANSI_COLOR_BLUE "\x1b[0;34m"
    #define ANSI_COLOR_BLUE_BOLD "\x1b[1;34m"
    #define ANSI_COLOR_BLUE_BG "\x1b[0;44m"

    #define ANSI_COLOR_MAGENTA "\x1b[0;35m"
    #define ANSI_COLOR_MAGENTA_BOLD "\x1b[1;35m"
    #define ANSI_COLOR_MAGENTA_BG "\x1b[0;45m"

    #define ANSI_COLOR_CYAN "\x1b[0;36m"
    #define ANSI_COLOR_CYAN_BOLD "\x1b[1;36m"
    #define ANSI_COLOR_CYAN_BG "\x1b[0;46m"

    #define ANSI_COLOR_WHITE "\x1b[0;37m"
    #define ANSI_COLOR_WHITE_BOLD "\x1b[1;37m"
    #define ANSI_COLOR_WHITE_BG "\x1b[0;47m"

    #define ANSI_COLOR_BLACK "\x1b[0;30m"
    #define ANSI_COLOR_BLACK_BOLD "\x1b[1;30m"
    #define ANSI_COLOR_BLACK_BG "\x1b[0;40m"

    #define ANSI_COLOR_GRAY "\x1b[0;90m"  // 灰色
    #define ANSI_COLOR_GRAY_BOLD "\x1b[1;90m" // 加粗灰色

    // 新增颜色
    #define ANSI_COLOR_ORANGE "\x1b[38;5;214m" // 橙色
    #define ANSI_COLOR_ORANGE_BOLD "\x1b[1;38;5;214m" // 加粗橙色

    #define ANSI_COLOR_PURPLE "\x1b[38;5;129m" // 紫色
    #define ANSI_COLOR_PURPLE_BOLD "\x1b[1;38;5;129m" // 加粗紫色

    #define ANSI_COLOR_LIGHT_BLUE "\x1b[38;5;81m"  // 浅蓝色
    #define ANSI_COLOR_LIGHT_BLUE_BOLD "\x1b[1;38;5;81m" // 加粗浅蓝色

    #define ANSI_COLOR_TURQUOISE "\x1b[38;5;51m" // 青色
    #define ANSI_COLOR_TURQUOISE_BOLD "\x1b[1;38;5;51m" // 加粗青色

    #define ANSI_COLOR_RESET "\x1b[0m"
    #define ANSI_DELETE_LAST_LINE "\033[A\33[2K\r"
    #define ANSI_DELETE_CURRENT_LINE "\33[2K\r"
    #define ANSI_SCREEN_FLUSH std::fflush(stdout);

    #define SET_PRINT_COLOR( a ) std::cout << a ;

#endif
// clang-format on

struct _Scope_color {
    _Scope_color(const char *color) { std::cout << color; }

    ~_Scope_color() { std::cout << ANSI_COLOR_RESET; }
};

#define scope_color(a) _Scope_color _scope(a);

// inline int demo_test_color_printf()
// {
//     int i, j, n;

//     for ( i = 0; i < 11; i++ )
//     {
//         for ( j = 0; j < 10; j++ )
//         {
//             n = 10 * i + j;
//             if ( n > 108 )
//                 break;
//             printf( "\033[%dm %3d\033[m", n, n );
//         }
//         printf( "\n" );
//     }
//     return ( 0 );
// };

#endif // UTILS_COLORPRINT_HPP
