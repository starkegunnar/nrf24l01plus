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

#include <string.h>
#include "nrf24l01plus.h"

/* Constants */
static const uint8_t pipe0_addr[] = {'n', 'o', 'd', 'e', '0'};

/* Static variables */
static nrf_handle_t handle = NULL;
static int nrf_interrupt = 0;

static nrf_pipe_config_t pipe_cfg[] =
{
  {pipe0_addr, 5, true, true, NRF_MAX_PAYLOAD_SIZE},
  {NULL, 0, false, false, 0}, /* NULL address at the end of pipe config */
};

static int nrf_callback(nrf_handle_t handle, nrf_callback_reason_t reason,
                        void *in, size_t len, void *user)
{
    (void)user;
    (void)handle;
    (void)in;
    switch (reason)
    {
    case NRF_CB_SPI_WRITE_READ:
        /*
            in: Pointer to nrf_spi_buffers_t
            nrf_spi_buffers_t->tx: pointer to write buffer
            nrf_spi_buffers_t->rx: pointer to read buffer
            len: bytes to be written/read
        */
        break;
    case NRF_CB_SPI_LOCK:
        /* Lock spi device if shared */
        break;
    case NRF_CB_SPI_UNLOCK:
        /* Unlock spi device if shared */
        break;
    case NRF_CB_WFI:
        /* Waiting for interrupt from radio before further processing */
        break;
    case NRF_CB_CE_SET:
        /* Set chip enable */
        break;
    case NRF_CB_CE_RESET:
        /* Reset chip enable */
        break;
    case NRF_CB_WRITE_ERROR:
        /* Error while transmitting */
        break;
    case NRF_CB_WRITE_COMPLETE:
        /* Transmit complete */
        break;
    case NRF_CB_READ_PAYLOAD:
        /* Received payload ready (in: pointer to data, len: number of bytes) */
        break;
    case NRF_CB_READ_ACK_PAYLOAD:
        /* Received ack payload ready (in: pointer to data, len: number of bytes) */
        break;
    case NRF_CB_ACK_PAYLOAD_SENT:
        /* Ack payload sent (len: pipe number where ack payload was sent) */
        break;
    default:
        break;
    }

    return 0;
}

int main_template(void)
{
    /* Init spi, gpios, interrupts etc. */

    handle = nrf_device_create(nrf_callback);
    if (!handle)
    {
        return -1;
    }

    nrf_init_t init;
    memset(&init, 0, sizeof(nrf_init_t));
    init.crc_len = NRF_CRC_16;
    init.address_width = NRF_ADDR_WIDTH_5;
    init.retry_delay = NRF_RETR_DELAY_4000;
    init.retry_cnt = NRF_RETR_COUNT_15;
    init.channel = NRF_CHANNEL_MAX;
    init.datarate = NRF_2MBPS;
    init.power = NRF_PWR_MIN;
    init.en_dynamic_ack = false;
    init.en_ack_payload = false;
    init.en_dynamic_payload = true;
    init.pipe_config = pipe_cfg;
    init.user = NULL;
    if (nrf_initialize(handle, &init))
    {
        nrf_device_free(handle);
        handle = NULL;
        return -1;
    }

    /* To enter RX mode
    if (nrf_set_state(nrf_handle_1, NRF_STATE_RX))
    {
        nrf_device_free(handle);
        handle = NULL;
        return -1;
    }
    Wait for interrupt on received payload
    */

    /* To transmit
    if (nrf_set_state(handle, NRF_STATE_STANDBY))
    {
        nrf_device_free(handle);
        handle = NULL;
        return -1;
    }
    if (nrf_write(handle, addr, addr_size, data, data_size)))
    {
        nrf_device_free(handle);
        handle = NULL;
        return -1;
    }
    Wait for interrupt on transmit error or completion
    */

    while (1)
    {
        if (nrf_interrupt)
        {
            nrf_interrupt = 0;
            if (nrf_service_interrupt(handle))
            {
                nrf_device_free(handle);
                handle = NULL;
                return -1;
            }
        }
    }

    nrf_device_free(handle);
    handle = NULL;

    return 0;
}


