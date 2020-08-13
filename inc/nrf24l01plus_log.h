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

#ifndef NRF24L01PLUS_LOG_H
#define NRF24L01PLUS_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef nrf_snprintf
#define nrf_snprintf snprintf
#endif

#ifndef nrf_vsnprintf
#define nrf_vsnprintf vsnprintf
#endif

#define NRF_LL_ERR      (1 << 0)
#define NRF_LL_WARN     (1 << 1)
#define NRF_LL_NOTICE   (1 << 2)
#define NRF_LL_INFO     (1 << 3)
#define NRF_LL_DEBUG    (1 << 4)
#define NRF_LL_COUNT    (5)

#define nrf_log_error(f, ...)           nrf_log(NRF_LL_ERR, f, ##__VA_ARGS__)
#define nrf_log_warn(f, ...)            nrf_log(NRF_LL_WARN, f, ##__VA_ARGS__)
#define nrf_log_notice(f, ...)          nrf_log(NRF_LL_NOTICE, f, ##__VA_ARGS__)
#define nrf_log_info(f, ...)            nrf_log(NRF_LL_INFO, f, ##__VA_ARGS__)
#define nrf_log_debug(f, ...)           nrf_log(NRF_LL_DEBUG, f, ##__VA_ARGS__)

#define nrf_log_hexdump_error(d, l)     nrf_log_hexdump(NRF_LL_ERR, d, l)
#define nrf_log_hexdump_warn(d, l)      nrf_log_hexdump(NRF_LL_WARN, d, l)
#define nrf_log_hexdump_notice(d, l)    nrf_log_hexdump(NRF_LL_NOTICE, d, l)
#define nrf_log_hexdump_info(d, l)      nrf_log_hexdump(NRF_LL_INFO, d, l)
#define nrf_log_hexdump_debug(d, l)     nrf_log_hexdump(NRF_LL_DEBUG, d, l)

void nrf_log(int level, const char *format, ...);
void nrf_log_hexdump(int level, const void *data, size_t len);

int nrf_get_log_level(void);
void nrf_set_log_level(int level);
void nrf_set_log_print(void (*nrf_log_print)(int level, const char *line, int len));

#ifdef __cplusplus
}
#endif

#endif /* NRF24L01PLUS_LOG_H */