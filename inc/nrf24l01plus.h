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

#ifndef NRF24L01PLUS_H
#define NRF24L01PLUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#include "nrf24l01plus_log.h"

#ifndef nrf_malloc
#define nrf_malloc malloc
#endif

#ifndef nrf_free
#define nrf_free free
#endif

#define NRF_MAX_PAYLOAD_SIZE 32
#define NRF_REG_TLV_SIZE ((22 * (1 + 1 + 1)) + (3 * (1 + 1 + 5))) /* 25 regs, [reg, len, value] */

typedef void* nrf_handle_t;

typedef enum
{
    NRF_CB_SPI_WRITE_READ,
    NRF_CB_SPI_LOCK,
    NRF_CB_SPI_UNLOCK,
    NRF_CB_WFI,
    NRF_CB_CE_SET,
    NRF_CB_CE_RESET,
    NRF_CB_WRITE_ERROR,
    NRF_CB_WRITE_COMPLETE,
    NRF_CB_READ_PAYLOAD,
    NRF_CB_READ_ACK_PAYLOAD,
    NRF_CB_ACK_PAYLOAD_SENT,
} nrf_callback_reason_t;

typedef struct
{
    uint8_t *tx;
    uint8_t *rx;
} nrf_spi_buffers_t;

typedef int (*nrf_callback_t)(nrf_handle_t handle, nrf_callback_reason_t reason,
                              void *in, size_t len, void *user);

typedef enum
{
    /* At boot the device will be in an unknown state */
    NRF_STATE_UNKNOWN,
    NRF_STATE_POWER_DOWN,
    NRF_STATE_STANDBY,
    NRF_STATE_RX,
} nrf_state_t;

typedef enum
{
    NRF_2MBPS,
    NRF_1MBPS,
    NRF_250KBPS,
} nrf_datarate_t;

typedef enum
{
    NRF_PWR_MIN,
    NRF_PWR_LOW,
    NRF_PWR_HIGH,
    NRF_PWR_MAX,
} nrf_power_dbm_t;

typedef enum
{
    NRF_ADDR_WIDTH_5,
    NRF_ADDR_WIDTH_4,
    NRF_ADDR_WIDTH_3,
} nrf_addr_width_t;

typedef enum
{
    NRF_RETR_DELAY_250,
    NRF_RETR_DELAY_500,
    NRF_RETR_DELAY_750,
    NRF_RETR_DELAY_1000,
    NRF_RETR_DELAY_1250,
    NRF_RETR_DELAY_1500,
    NRF_RETR_DELAY_1750,
    NRF_RETR_DELAY_2000,
    NRF_RETR_DELAY_2250,
    NRF_RETR_DELAY_2500,
    NRF_RETR_DELAY_2750,
    NRF_RETR_DELAY_3000,
    NRF_RETR_DELAY_3250,
    NRF_RETR_DELAY_3500,
    NRF_RETR_DELAY_3750,
    NRF_RETR_DELAY_4000,
} nrf_retransmit_delay_us_t;

typedef enum
{
    NRF_RETR_DISABLED,
    NRF_RETR_COUNT_1,
    NRF_RETR_COUNT_2,
    NRF_RETR_COUNT_3,
    NRF_RETR_COUNT_4,
    NRF_RETR_COUNT_5,
    NRF_RETR_COUNT_6,
    NRF_RETR_COUNT_7,
    NRF_RETR_COUNT_8,
    NRF_RETR_COUNT_9,
    NRF_RETR_COUNT_10,
    NRF_RETR_COUNT_11,
    NRF_RETR_COUNT_12,
    NRF_RETR_COUNT_13,
    NRF_RETR_COUNT_14,
    NRF_RETR_COUNT_15,
} nrf_retransmit_count_t;

typedef enum
{
    NRF_CRC_8,
    NRF_CRC_16,
    NRF_CRC_DISABLED,
} nrf_crc_length_t;

typedef enum
{
    NRF_RX_PIPE_0,
    NRF_RX_PIPE_1,
    NRF_RX_PIPE_2,
    NRF_RX_PIPE_3,
    NRF_RX_PIPE_4,
    NRF_RX_PIPE_5,
    NRF_NUM_RX_PIPES,
} nrf_pipe_t;

#define NRF_CHANNEL_MIN 0x00
#define NRF_CHANNEL_MAX 0x3F
typedef uint8_t nrf_channel_t;

/*
Note: Addresses where the level shifts only one time (example: 000FFFFFFF) can often be detected in
      noise and can give a false detection, which may give a raised Packet Error Rate. Addresses
      as a continuation of the preamble (hi-low toggling) also raises the Packet Error Rate.
*/
typedef struct
{
    const uint8_t *address;
    size_t address_width;
    bool auto_ack;
    bool dynamic_payload;
    size_t payload_width;
} nrf_pipe_config_t;

typedef struct
{
    nrf_crc_length_t            crc_len;
    nrf_addr_width_t            address_width;
    nrf_retransmit_delay_us_t   retry_delay;
    nrf_retransmit_count_t      retry_cnt;
    nrf_channel_t               channel;
    nrf_datarate_t              datarate;
    nrf_power_dbm_t             power;
    const nrf_pipe_config_t     *pipe_config;
    bool                        en_dynamic_ack;
    bool                        en_ack_payload;
    bool                        en_dynamic_payload;
    void                        *user;
} nrf_init_t;

/* Create radio device handle */
nrf_handle_t nrf_device_create(nrf_callback_t callback);
/* Free radio device handle */
void nrf_device_free(nrf_handle_t handle);
/* Get radio device handle context size */
size_t nrf_device_size(void);
/* Create radio device handle in pre-allocated buffer */
nrf_handle_t nrf_device_create_from_buffer(void *buffer, size_t *buffer_size,
                                           nrf_callback_t callback);
/* Initialize radio device registers */
int nrf_initialize(nrf_handle_t handle, const nrf_init_t *init);

/* Get radio device state */
nrf_state_t nrf_get_state(nrf_handle_t handle);
/* Set radio device state */
int nrf_set_state(nrf_handle_t handle, nrf_state_t state);

/* Transmit data */
int nrf_write(nrf_handle_t handle, const uint8_t *addr, size_t addr_size,
              const uint8_t *data, size_t data_size);
/* Transmit data without waiting for ack */
int nrf_write_no_ack(nrf_handle_t handle, const uint8_t *addr, size_t addr_size,
                     const uint8_t *data, size_t data_size);

/* Reset pending ack payload for pipe */
int nrf_reset_ack_payload(nrf_handle_t handle, nrf_pipe_t pipe);
/* Set pending ack payload for pipe */
int nrf_set_ack_payload(nrf_handle_t handle, nrf_pipe_t pipe,
                        const uint8_t *ack_payload, size_t ack_payload_size);

/* Service radio interrupt. Should not be called in interrupt context */
int nrf_service_interrupt(nrf_handle_t handle);

/* Dump radio device registers to log */
int nrf_register_dump(nrf_handle_t handle);
/* Dump radio device registers to tlv buffer [tag=reg, length, value] */
int nrf_register_dump_tlv(nrf_handle_t handle, uint8_t *buffer, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* NRF24L01PLUS_H */