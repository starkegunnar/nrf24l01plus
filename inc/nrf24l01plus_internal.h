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

#ifndef NRF24L01PLUS_INT_H
#define NRF24L01PLUS_INT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf24l01plus.h"

/* Variable commands */
#define NRF_CMD_MASK                0xE0
#define NRF_REGISTER_ADDR_MASK      0x1F
/* Read from register address 'r' */
#define NRF_CMD_R_REGISTER_MASK     0x00
#define NRF_CMD_R_REGISTER(a)       (NRF_CMD_R_REGISTER_MASK | (NRF_REGISTER_ADDR_MASK & a))
/* Write to register address 'r' */
#define NRF_CMD_W_REGISTER_MASK     0x20
#define NRF_CMD_W_REGISTER(a)       (NRF_CMD_W_REGISTER_MASK | (NRF_REGISTER_ADDR_MASK & a))
/* Write payload to be transmitted together with ACK packet on pipe 'p' */
#define NRF_CMD_W_ACK_PAYLOAD_MASK  0xA8
#define NRF_CMD_W_ACK_PIPE_MASK     0x07
#define NRF_CMD_W_ACK_PAYLOAD(p)    (NRF_CMD_W_ACK_PAYLOAD_MASK | (NRF_CMD_W_ACK_PIPE_MASK & p))

/* Fixed commands */
#define NRF_CMD_R_RX_PAYLOAD        0x61
#define NRF_CMD_W_TX_PAYLOAD        0xA0
#define NRF_CMD_FLUSH_TX            0xE1
#define NRF_CMD_FLUSH_RX            0xE0
#define NRF_CMD_REUSE_TX_PL         0xE3
#define NRF_CMD_R_RX_PL_WID         0x60
#define NRF_CMD_W_TX_PAYLOAD_NO_ACK 0xB0
#define NRF_CMD_NOP                 0xFF

#define NRF_REG_MAX_SIZE            5
#define NRF_FIFO_LEVELS             3

#define NRF_CONFIG_REG_ADDR         0x00
#define NRF_CONFIG_REG_OFFSET       NRF_CONFIG_REG_ADDR
typedef union
{
    struct
    {
        uint8_t prim_rx : 1;
        uint8_t pwr_up : 1;
        uint8_t crc_len : 1;
        uint8_t en_crc : 1;
        uint8_t mask_max_rt : 1;
        uint8_t mask_tx_ds : 1;
        uint8_t mask_rx_dr : 1;
        uint8_t reserved : 1;
    } bits;
    uint8_t data;
} nrf_config_reg_t;

#define NRF_EN_AA_REG_ADDR          0x01
#define NRF_EN_AA_REG_OFFSET        NRF_EN_AA_REG_ADDR
typedef union
{
    struct
    {
        uint8_t enaa_p0 : 1;
        uint8_t enaa_p1 : 1;
        uint8_t enaa_p2 : 1;
        uint8_t enaa_p3 : 1;
        uint8_t enaa_p4 : 1;
        uint8_t enaa_p5 : 1;
        uint8_t reserved : 2;
    } bits;
    uint8_t data;
} nrf_en_aa_reg_t;

#define NRF_EN_RXADDR_REG_ADDR      0x02
#define NRF_EN_RXADDR_REG_OFFSET    NRF_EN_RXADDR_REG_ADDR
typedef union
{
    struct
    {
        uint8_t erx_p0 : 1;
        uint8_t erx_p1 : 1;
        uint8_t erx_p2 : 1;
        uint8_t erx_p3 : 1;
        uint8_t erx_p4 : 1;
        uint8_t erx_p5 : 1;
        uint8_t reserved : 2;
    } bits;
    uint8_t data;
} nrf_en_rxaddr_reg_t;

#define NRF_SETUP_AW_REG_ADDR       0x03
#define NRF_SETUP_AW_REG_OFFSET     NRF_SETUP_AW_REG_ADDR
typedef union
{
    struct
    {
        uint8_t aw : 2;
        uint8_t reserved : 6;
    } bits;
    uint8_t data;
} nrf_setup_aw_reg_t;

#define NRF_SETUP_RETR_REG_ADDR     0x04
#define NRF_SETUP_RETR_REG_OFFSET   NRF_SETUP_RETR_REG_ADDR
typedef union
{
    struct
    {
        uint8_t arc : 4;
        uint8_t ard : 4;
    } bits;
    uint8_t data;
} nrf_setup_retr_reg_t;

#define NRF_RF_CH_REG_ADDR          0x05
#define NRF_RF_CH_REG_OFFSET        NRF_RF_CH_REG_ADDR
typedef union
{
    struct
    {
        uint8_t rf_ch : 7;
        uint8_t reserved : 1;
    } bits;
    uint8_t data;
} nrf_rf_ch_reg_t;

#define NRF_RF_SETUP_REG_ADDR       0x06
#define NRF_RF_SETUP_REG_OFFSET     NRF_RF_SETUP_REG_ADDR
typedef union
{
    struct
    {
        uint8_t obsolete : 1;
        uint8_t rf_pwr : 2;
        uint8_t rf_dr_high : 1;
        uint8_t pll_lock : 1; // only used in test
        uint8_t rf_dr_low : 1;
        uint8_t reserved : 1;
        uint8_t cont_wave : 1;
    } bits;
    uint8_t data;
} nrf_rf_setup_reg_t;

#define NRF_STATUS_REG_ADDR         0x07
#define NRF_STATUS_REG_OFFSET       NRF_STATUS_REG_ADDR
#define NRF_STATUS_RX_P_NO_PIPE_0   0x00
#define NRF_STATUS_RX_P_NO_PIPE_1   0x01
#define NRF_STATUS_RX_P_NO_PIPE_2   0x02
#define NRF_STATUS_RX_P_NO_PIPE_3   0x03
#define NRF_STATUS_RX_P_NO_PIPE_4   0x04
#define NRF_STATUS_RX_P_NO_PIPE_5   0x05
#define NRF_STATUS_RX_P_NO_EMPTY    0x07
#define NRF_STATUS_RESET_VAL        0x0E
#define NRF_STATUS_CLEAR_IRQ_MAX_RT 0x10
#define NRF_STATUS_CLEAR_IRQ_TX_DS  0x20
#define NRF_STATUS_CLEAR_IRQ_RX_DR  0x40
#define NRF_STATUS_CLEAR_IRQ_ALL    (NRF_STATUS_CLEAR_IRQ_MAX_RT | NRF_STATUS_CLEAR_IRQ_TX_DS | NRF_STATUS_CLEAR_IRQ_RX_DR)
typedef union
{
    struct
    {
        uint8_t tx_full : 1;
        uint8_t rx_p_no : 3;
        uint8_t max_rt : 1;
        uint8_t tx_ds : 1;
        uint8_t rx_dr : 1;
        uint8_t reserved : 1;
    } bits;
    uint8_t data;
} nrf_status_reg_t;

#define NRF_OBSERVE_TX_REG_ADDR     0x08
#define NRF_OBSERVE_TX_REG_OFFSET   NRF_OBSERVE_TX_REG_ADDR
typedef union
{
    struct
    {
        uint8_t arc_cnt : 4;
        uint8_t plos_cnt : 4;
    } bits;
    uint8_t data;
} nrf_observe_tx_reg_t;

#define NRF_RPD_REG_ADDR            0x09
#define NRF_RPD_REG_OFFSET          NRF_RPD_REG_ADDR
typedef union
{
    struct
    {
        uint8_t rpd : 1;
        uint8_t reserved : 7;
    } bits;
    uint8_t data;
} nrf_rpd_reg_t;

#define NRF_RX_ADDR_P0_REG_ADDR     0x0A
#define NRF_RX_ADDR_P0_REG_OFFSET   NRF_RX_ADDR_P0_REG_ADDR
typedef struct
{
    uint8_t addr[5];
} nrf_rx_addr_p0_reg_t;

#define NRF_RX_ADDR_P1_REG_ADDR     0x0B
#define NRF_RX_ADDR_P1_REG_OFFSET   (NRF_RX_ADDR_P0_REG_OFFSET+ sizeof(nrf_rx_addr_p0_reg_t))
typedef struct
{
    uint8_t addr[5];
} nrf_rx_addr_p1_reg_t;

#define NRF_RX_ADDR_P2_REG_ADDR     0x0C
#define NRF_RX_ADDR_P3_REG_ADDR     0x0D
#define NRF_RX_ADDR_P4_REG_ADDR     0x0E
#define NRF_RX_ADDR_P5_REG_ADDR     0x0F
#define NRF_RX_ADDR_P2_REG_OFFSET   (NRF_RX_ADDR_P1_REG_OFFSET+ sizeof(nrf_rx_addr_p1_reg_t))
#define NRF_RX_ADDR_P3_REG_OFFSET   (NRF_RX_ADDR_P2_REG_OFFSET+ 1)
#define NRF_RX_ADDR_P4_REG_OFFSET   (NRF_RX_ADDR_P3_REG_OFFSET+ 1)
#define NRF_RX_ADDR_P5_REG_OFFSET   (NRF_RX_ADDR_P4_REG_OFFSET+ 1)
typedef struct
{
    uint8_t addr;
} nrf_rx_addr_pX_reg_t;

#define NRF_TX_ADDR_REG_ADDR        0x10
#define NRF_TX_ADDR_REG_OFFSET      (NRF_RX_ADDR_P5_REG_OFFSET+ 1)
typedef struct
{
    uint8_t addr[5];
} nrf_tx_addr_reg_t;

#define NRF_RX_PW_P0_REG_ADDR       0x11
#define NRF_RX_PW_P1_REG_ADDR       0x12
#define NRF_RX_PW_P2_REG_ADDR       0x13
#define NRF_RX_PW_P3_REG_ADDR       0x14
#define NRF_RX_PW_P4_REG_ADDR       0x15
#define NRF_RX_PW_P5_REG_ADDR       0x16
#define NRF_RX_PW_P0_REG_OFFSET     (NRF_TX_ADDR_REG_OFFSET+ sizeof(nrf_tx_addr_reg_t))
#define NRF_RX_PW_P1_REG_OFFSET     (NRF_RX_PW_P0_REG_OFFSET+ 1)
#define NRF_RX_PW_P2_REG_OFFSET     (NRF_RX_PW_P1_REG_OFFSET+ 1)
#define NRF_RX_PW_P3_REG_OFFSET     (NRF_RX_PW_P2_REG_OFFSET+ 1)
#define NRF_RX_PW_P4_REG_OFFSET     (NRF_RX_PW_P3_REG_OFFSET+ 1)
#define NRF_RX_PW_P5_REG_OFFSET     (NRF_RX_PW_P4_REG_OFFSET+ 1)
typedef union
{
    struct
    {
        uint8_t rx_pw : 6;
        uint8_t reserved : 2;
    } bits;
    uint8_t data;
} nrf_rx_pw_reg_t;

#define NRF_FIFO_STATUS_REG_ADDR    0x17
#define NRF_FIFO_STATUS_REG_OFFSET  (NRF_RX_PW_P5_REG_OFFSET+ 1)
typedef union
{
    struct
    {
        uint8_t rx_empty : 1;
        uint8_t rx_full : 1;
        uint8_t reserved_0 : 2;
        uint8_t tx_empty : 1;
        uint8_t tx_full : 1;
        uint8_t tx_reuse : 1;
        uint8_t reserved_1 : 1;
    } bits;
    uint8_t data;
} nrf_fifo_status_reg_t;

#define NRF_DYNPD_REG_ADDR          0x1C
#define NRF_DYNPD_REG_OFFSET        (NRF_FIFO_STATUS_REG_OFFSET+ 1)
typedef union
{
    struct
    {
        uint8_t dpl_p0 : 1;
        uint8_t dpl_p1 : 1;
        uint8_t dpl_p2 : 1;
        uint8_t dpl_p3 : 1;
        uint8_t dpl_p4 : 1;
        uint8_t dpl_p5 : 1;
        uint8_t reserved : 2;
    } bits;
    uint8_t data;
} nrf_dynpd_reg_t;

#define NRF_FEATURE_REG_ADDR        0x1D
#define NRF_FEATURE_REG_OFFSET      (NRF_DYNPD_REG_OFFSET+ 1)
typedef union
{
    struct
    {
        uint8_t en_dyn_ack : 1;
        uint8_t en_ack_pay : 1;
        uint8_t en_dpl : 1;
        uint8_t reserved : 5;
    } bits;
    uint8_t data;
} nrf_feature_reg_t;

#define NRF_INVALID_REG_ADDR        0x1E

typedef struct
{
    uint8_t cmd;
    uint8_t payload[NRF_MAX_PAYLOAD_SIZE];
} __attribute__((packed)) nrf_spi_tx_payload_t;

typedef struct
{
    uint8_t status;
    uint8_t payload[NRF_MAX_PAYLOAD_SIZE];
} __attribute__((packed)) nrf_spi_rx_payload_t;

typedef struct
{
    const uint8_t *payload;
    size_t size;
} nrf_ack_payload_t;

typedef struct
{
    nrf_config_reg_t        config;         // 00, 1 byte
    nrf_en_aa_reg_t         en_aa;          // 01, 1 byte
    nrf_en_rxaddr_reg_t     en_rxaddr;      // 02, 1 byte
    nrf_setup_aw_reg_t      setup_aw;       // 03, 1 byte
    nrf_setup_retr_reg_t    setup_retr;     // 04, 1 byte
    nrf_rf_ch_reg_t         rf_ch;          // 05, 1 byte
    nrf_rf_setup_reg_t      rf_setup;       // 06, 1 byte
    nrf_rx_addr_p0_reg_t    rx_addr_p0;     // 0A, 5 bytes
    nrf_rx_addr_p1_reg_t    rx_addr_p1;     // 0B, 5 bytes
    nrf_rx_addr_pX_reg_t    rx_addr_p2;     // 0C, 1 byte
    nrf_rx_addr_pX_reg_t    rx_addr_p3;     // 0D, 1 byte
    nrf_rx_addr_pX_reg_t    rx_addr_p4;     // 0E, 1 byte
    nrf_rx_addr_pX_reg_t    rx_addr_p5;     // 0F, 1 byte
    nrf_tx_addr_reg_t       tx_addr;        // 10, 5 bytes
    nrf_rx_pw_reg_t         rx_pw_p0;       // 11, 1 byte
    nrf_rx_pw_reg_t         rx_pw_p1;       // 12, 1 byte
    nrf_rx_pw_reg_t         rx_pw_p2;       // 13, 1 byte
    nrf_rx_pw_reg_t         rx_pw_p3;       // 14, 1 byte
    nrf_rx_pw_reg_t         rx_pw_p4;       // 15, 1 byte
    nrf_rx_pw_reg_t         rx_pw_p5;       // 16, 1 byte
    nrf_dynpd_reg_t         dynpd;          // 1C, 1 byte
    nrf_feature_reg_t       feature;        // 1D, 1 byte
} __attribute__((packed)) nrf_writable_regs_t;

typedef struct
{
    nrf_state_t                 state;
    nrf_state_t                 prev_state;

    nrf_spi_tx_payload_t        tx;
    nrf_spi_rx_payload_t        rx;
    uint8_t                     len;

    nrf_writable_regs_t         regs;
    uint8_t                     ce_state;

    /* TX parameters */
    const uint8_t               *payload;
    size_t                      payload_size;
    uint8_t                     no_ack;
    uint8_t                     tx_pending;

    uint8_t                     last_rx_pipe;
    nrf_ack_payload_t           ack_payloads[NRF_NUM_RX_PIPES];

    nrf_callback_t              callback;

    void                        *user;
} nrf_device_t;

#ifdef __cplusplus
}
#endif

#endif /* NRF24L01PLUS_INT_H */