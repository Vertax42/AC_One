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

#ifndef __UTILS_OSCOMPATIBLE_HPP__
#define __UTILS_OSCOMPATIBLE_HPP__

#include <fstream>
#include <iostream>
#include <ostream>
#include <sstream>
#include <stdarg.h>
#include <stdio.h>
#include <string>

#if defined _MSC_VER
#include <direct.h>
#elif defined __GNUC__
#include <sys/stat.h>
#include <sys/types.h>
#endif

namespace common_tools {
inline void create_dir(std::string dir)
{
#if defined _MSC_VER
    _mkdir(dir.data());
#elif defined __GNUC__
    mkdir(dir.data(), 0777);
#endif
}

// Using asprintf() on windows
// https://stackoverflow.com/questions/40159892/using-asprintf-on-windows
#ifndef _vscprintf
/* For some reason, MSVC fails to honour this #ifndef. */
/* Hence function renamed to _vscprintf_so(). */
inline int _vscprintf_so(const char *format, va_list pargs)
{
    int retval;
    va_list argcopy;
    va_copy(argcopy, pargs);
    retval = vsnprintf(NULL, 0, format, argcopy);
    va_end(argcopy);
    return retval;
}
#endif //

#ifndef vasprintf
inline int vasprintf(char **strp, const char *fmt, va_list ap)
{
    int len = _vscprintf_so(fmt, ap);
    if(len == -1) return -1;
    char *str = (char *)malloc((size_t)len + 1);
    if(!str) return -1;
    int r = vsnprintf(str, len + 1, fmt, ap); /* "secure" version of vsprintf */
    if(r == -1) return free(str), -1;
    *strp = str;
    return r;
}
#endif // vasprintf
} // namespace common_tools

#endif // UTILS_OSCOMPATIBLE_HPP
