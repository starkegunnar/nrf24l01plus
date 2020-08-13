/*
 * Copyright (c) 2020 Martin George
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

#include <stdio.h>
#include <stdarg.h>

#include "nrf24l01plus_log.h"

static int _nrf_log_level = (NRF_LL_ERR | NRF_LL_WARN | NRF_LL_NOTICE);
static void (*_nrf_log_print)(int level, const char *line, int len) = NULL;

void nrf_log(int level, const char *format, ...)
{
    if (!format || !_nrf_log_print || !(_nrf_log_level & level))
    {
        return;
    }

    char line[256];
    va_list args;
    va_start(args, format);
    int n = nrf_vsnprintf(line, (sizeof(line) - 1), format, args);
    if ((int)(sizeof(line) - 1) < n)
    {
        n = sizeof(line) - 5;
        line[n++] = '.';
        line[n++] = '.';
        line[n++] = '.';
        line[n++] = '\n';
        line[n] = '\0';
    }
    else
    {
        line[n] = '\0';
    }

    /* Do print */
    _nrf_log_print(level, line, n);

    va_end(args);
}

void nrf_log_hexdump(int level, const void *data, size_t len)
{
    if (!data || !len || !_nrf_log_print || !(_nrf_log_level & level))
    {
        return;
    }

    unsigned char *buf = (unsigned char *)data;
    unsigned int n;

    for (n = 0; n < len;)
    {
        unsigned int start = n, m;
        char line[80];
        char *p = line;

        p += nrf_snprintf(line, 10, "%04X: ", start);

        for (m = 0; (m < 16) && (n < len); ++m)
        {
            p += nrf_snprintf(p, 5, "%02X ", buf[n++]);
        }
        while (m++ < 16)
        {
            p += nrf_snprintf(p, 5, "   ");
        }

        p += nrf_snprintf(p, 6, "   ");

        for (m = 0; (m < 16) && ((start + m < len)); ++m)
        {
            if ((' ' <= buf[start + m]) && ('~' >= buf[start + m]))
            {
                *p++ = buf[start + m];
            }
            else
            {
                *p++ = '.';
            }
        }
        while (m++ < 16)
        {
            *p++ = ' ';
        }

        *p++ = '\n';
        *p = '\0';
        _nrf_log_print(level, line, (p - line));
    }
}

int nrf_get_log_level(void)
{
    return _nrf_log_level;
}

void nrf_set_log_level(int level)
{
    _nrf_log_level = level;
}

void nrf_set_log_print(void (*nrf_log_print)(int level, const char *line, int len))
{
    _nrf_log_print = nrf_log_print;
}