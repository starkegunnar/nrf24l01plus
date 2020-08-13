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
#include "nrf24l01plus_internal.h"
#include "nrf24l01plus_log.h"

#define NRF_MIN(a, b) ((a < b) ? a : b)
#define NRF_MAX(a, b) ((a < b) ? b : a)

#define NRF_BAD_ADDRESS_BYTE_0 0x55
#define NRF_BAD_ADDRESS_BYTE_1 0xAA

static inline void _nrf_ce_set(nrf_device_t *dev)
{
    (void)dev->callback(dev, NRF_CB_CE_SET, NULL, 0, dev->user);
    dev->ce_state = 1;
}

static inline void _nrf_ce_reset(nrf_device_t *dev)
{
    (void)dev->callback(dev, NRF_CB_CE_RESET, NULL, 0, dev->user);
    dev->ce_state = 0;
}

static int _nrf_spi_write_read(nrf_device_t *dev)
{
    int err = dev->callback(dev, NRF_CB_SPI_LOCK, NULL, 0, dev->user);
    if (!err)
    {
        nrf_spi_buffers_t buf = {(uint8_t *)&dev->tx, (uint8_t *)&dev->rx};
        err = dev->callback(dev, NRF_CB_SPI_WRITE_READ, &buf, dev->len, dev->user);
        if (err)
        {
            nrf_log_error("%s: User SPI write read returned error\n", __func__);
        }
        (void)dev->callback(dev, NRF_CB_SPI_UNLOCK, NULL, 0, dev->user);
    }
    return err;
}

static inline int _nrf_read_status(nrf_device_t *dev)
{
    dev->tx.cmd = NRF_CMD_NOP;
    dev->len = 1;
    return _nrf_spi_write_read(dev);
}

static inline int _nrf_flush_rx(nrf_device_t *dev)
{
    dev->tx.cmd = NRF_CMD_FLUSH_RX;
    dev->len = 1;
    return _nrf_spi_write_read(dev);
}

static inline int _nrf_flush_tx(nrf_device_t *dev)
{
    dev->tx.cmd = NRF_CMD_FLUSH_TX;
    dev->len = 1;
    return _nrf_spi_write_read(dev);
}

static inline int _nrf_reuse_tx_payload(nrf_device_t *dev)
{
    dev->tx.cmd = NRF_CMD_REUSE_TX_PL;
    dev->len = 1;
    return _nrf_spi_write_read(dev);
}

static int _nrf_read_rx_payload_width(nrf_device_t *dev, uint8_t *width)
{
    dev->tx.cmd = NRF_CMD_R_RX_PL_WID;
    dev->len = 2;
    int ret = _nrf_spi_write_read(dev);
    if (!ret)
    {
        *width = dev->rx.payload[0];
    }
    return ret;
}

static int _nrf_read_register(nrf_device_t *dev, uint8_t reg, uint8_t *data, size_t len)
{
    dev->tx.cmd = NRF_CMD_R_REGISTER(reg);
    dev->len = 1 + len;
    int ret = _nrf_spi_write_read(dev);
    if (!ret)
    {
        memcpy(data, dev->rx.payload, len);
    }
    return ret;
}

static int _nrf_write_register(nrf_device_t *dev, uint8_t reg, const uint8_t *data, size_t len)
{
    dev->tx.cmd = NRF_CMD_W_REGISTER(reg);
    memcpy(dev->tx.payload, data, len);
    dev->len = 1 + len;
    return _nrf_spi_write_read(dev);
}

static int _nrf_read_payload(nrf_device_t *dev, size_t len)
{
    dev->tx.cmd = NRF_CMD_R_RX_PAYLOAD;
    dev->len = 1 + len;
    int ret = _nrf_spi_write_read(dev);
    return ret;
}

static int _nrf_write_payload(nrf_device_t *dev, const uint8_t *payload, size_t len)
{
    dev->tx.cmd = NRF_CMD_W_TX_PAYLOAD;
    memcpy(dev->tx.payload, payload, len);
    dev->len = 1 + len;
    return _nrf_spi_write_read(dev);
}

static int _nrf_write_payload_no_ack(nrf_device_t *dev, const uint8_t *payload, size_t len)
{
    dev->tx.cmd = NRF_CMD_W_TX_PAYLOAD_NO_ACK;
    memcpy(dev->tx.payload, payload, len);
    dev->len = 1 + len;
    return _nrf_spi_write_read(dev);
}

static int _nrf_write_ack_payload(nrf_device_t *dev, uint8_t pipe, const uint8_t *ack, size_t len)
{
    dev->tx.cmd = NRF_CMD_W_ACK_PAYLOAD(pipe);
    memcpy(dev->tx.payload, ack, len);
    dev->len = 1 + len;
    return _nrf_spi_write_read(dev);
}

static int _nrf_pipe_address_valid(const uint8_t *address, size_t len)
{
    int bad_bytes = 0;
    int transitions = 0;
    for (int i = 0; i < len; ++i)
    {
        uint8_t tmp = address[i];
        if ((NRF_BAD_ADDRESS_BYTE_0 == tmp) ||
            (NRF_BAD_ADDRESS_BYTE_1 == tmp))
        {
            ++bad_bytes;
        }
        uint8_t msb = (0x80 & tmp);
        while ((1 >= transitions) && (0x00 != tmp) && (0xFF != tmp))
        {
            tmp <<= 1;
            if ((0x80 & tmp) != msb)
            {
                ++transitions;
                msb = (0x80 & tmp);
            }
        }
    }
    return ((2 >= bad_bytes) && (1 < transitions));
}

static int _nrf_reset(nrf_device_t *dev)
{
    if (_nrf_flush_rx(dev))
    {
        return -1;
    }

    if (_nrf_flush_tx(dev))
    {
        return -1;
    }

    uint8_t reset_irq = NRF_STATUS_CLEAR_IRQ_ALL;
    if (_nrf_write_register(dev, NRF_STATUS_REG_ADDR, &reset_irq, sizeof(reset_irq)))
    {
        return -1;
    }

    int err = 0, attempts = 100;
    do
    {
        err = _nrf_read_status(dev);
        --attempts;
    } while (attempts && !err && (NRF_STATUS_RESET_VAL != dev->rx.status));

    return err;
}

int nrf_initialize(nrf_handle_t handle, const nrf_init_t *init)
{
    if (!handle || !init)
    {
        return -1;
    }

    nrf_device_t *dev = (nrf_device_t *)handle;

    memset(&dev->regs, 0, sizeof(nrf_writable_regs_t));
    dev->user = init->user;

    if (_nrf_reset(dev))
    {
        return -1;
    }

    switch (init->crc_len)
    {
    case NRF_CRC_8:
        dev->regs.config.bits.en_crc = 1;
        dev->regs.config.bits.crc_len = 0;
        break;
    case NRF_CRC_16:
        dev->regs.config.bits.en_crc = 1;
        dev->regs.config.bits.crc_len = 1;
        break;
    default:
        dev->regs.config.bits.en_crc = 0;
        break;
    }
    if (_nrf_write_register(dev, NRF_CONFIG_REG_ADDR,
            &dev->regs.config.data, sizeof(nrf_config_reg_t)))
    {
        return -1;
    }

    switch (init->address_width)
    {
    case NRF_ADDR_WIDTH_4:
        dev->regs.setup_aw.bits.aw = 2;
        break;
    case NRF_ADDR_WIDTH_3:
        dev->regs.setup_aw.bits.aw = 1;
        break;
    default:
        dev->regs.setup_aw.bits.aw = 3;
        break;
    }
    if (_nrf_write_register(dev, NRF_SETUP_AW_REG_ADDR,
            &dev->regs.setup_aw.data, sizeof(nrf_setup_aw_reg_t)))
    {
        return -1;
    }

    dev->regs.setup_retr.bits.ard = init->retry_delay;
    dev->regs.setup_retr.bits.arc = init->retry_cnt;
    if (_nrf_write_register(dev, NRF_SETUP_RETR_REG_ADDR,
            &dev->regs.setup_retr.data, sizeof(nrf_setup_retr_reg_t)))
    {
        return -1;
    }

    if (NRF_CHANNEL_MAX < init->channel)
    {
        dev->regs.rf_ch.bits.rf_ch = NRF_CHANNEL_MAX;
    }
    else
    {
        dev->regs.rf_ch.bits.rf_ch = init->channel;
    }
    if (_nrf_write_register(dev, NRF_RF_CH_REG_ADDR,
            &dev->regs.rf_ch.data, sizeof(nrf_rf_ch_reg_t)))
    {
        return -1;
    }

    switch (init->datarate)
    {
    case NRF_250KBPS:
        dev->regs.rf_setup.bits.rf_dr_high = 0;
        dev->regs.rf_setup.bits.rf_dr_low = 1;
        break;
    case NRF_1MBPS:
        dev->regs.rf_setup.bits.rf_dr_high = 0;
        dev->regs.rf_setup.bits.rf_dr_low = 0;
        break;
    default:
        dev->regs.rf_setup.bits.rf_dr_high = 1;
        dev->regs.rf_setup.bits.rf_dr_low = 0;
        break;
    }
    dev->regs.rf_setup.bits.rf_pwr = init->power;
    if (_nrf_write_register(dev, NRF_RF_SETUP_REG_ADDR,
            &dev->regs.rf_setup.data, sizeof(nrf_rf_setup_reg_t)))
    {
        return -1;
    }

    dev->regs.feature.bits.en_dyn_ack = init->en_dynamic_ack;
    dev->regs.feature.bits.en_ack_pay = init->en_ack_payload;
    dev->regs.feature.bits.en_dpl = init->en_dynamic_payload;
    if (_nrf_write_register(dev, NRF_FEATURE_REG_ADDR,
            &dev->regs.feature.data, sizeof(nrf_feature_reg_t)))
    {
        return -1;
    }

    if (init->pipe_config)
    {
        for (int pipe = NRF_RX_PIPE_0; pipe < NRF_NUM_RX_PIPES; ++pipe)
        {
            const nrf_pipe_config_t *cfg = &init->pipe_config[pipe];
            if (cfg->address)
            {
                uint8_t *addr = NULL;
                uint8_t *pw = NULL;
                size_t addr_size = 0;
                const size_t aw = (dev->regs.setup_aw.bits.aw + 2);
                if ((NRF_RX_PIPE_0 == pipe) ||
                    (NRF_RX_PIPE_1 == pipe))
                {
                    if ((aw > cfg->address_width) ||
                        !_nrf_pipe_address_valid(cfg->address, cfg->address_width))
                    {
                        return -1;
                    }
                    if ((NRF_RX_PIPE_1 == pipe) &&
                        !memcmp(dev->regs.rx_addr_p0.addr, cfg->address, aw))
                    {
                        return -1;
                    }
                }
                else if (NRF_RX_PIPE_1 < pipe)
                {
                    for (uint8_t i = NRF_RX_PIPE_1; i < pipe; ++i)
                    {
                        addr = ((uint8_t *)&dev->regs.rx_addr_p2.addr) + (pipe - NRF_RX_PIPE_2);
                        if (*addr == cfg->address[0])
                        {
                            return -1;
                        }
                    }
                }
                switch (pipe)
                {
                case NRF_RX_PIPE_0:
                    memcpy(dev->regs.rx_addr_p0.addr, cfg->address, aw);
                    addr = dev->regs.rx_addr_p0.addr;
                    addr_size = aw;
                    pw = &dev->regs.rx_pw_p0.data;
                    break;
                case NRF_RX_PIPE_1:
                    memcpy(dev->regs.rx_addr_p1.addr, cfg->address, aw);
                    addr = dev->regs.rx_addr_p1.addr;
                    addr_size = aw;
                    pw = &dev->regs.rx_pw_p1.data;
                    break;
                case NRF_RX_PIPE_2:
                    dev->regs.rx_addr_p2.addr = cfg->address[0];
                    addr = &dev->regs.rx_addr_p2.addr;
                    addr_size = sizeof(nrf_rx_addr_pX_reg_t);
                    pw = &dev->regs.rx_pw_p2.data;
                    break;
                case NRF_RX_PIPE_3:
                    dev->regs.rx_addr_p3.addr = cfg->address[0];
                    addr = &dev->regs.rx_addr_p3.addr;
                    addr_size = sizeof(nrf_rx_addr_pX_reg_t);
                    pw = &dev->regs.rx_pw_p3.data;
                    break;
                case NRF_RX_PIPE_4:
                    dev->regs.rx_addr_p4.addr = cfg->address[0];
                    addr = &dev->regs.rx_addr_p4.addr;
                    addr_size = sizeof(nrf_rx_addr_pX_reg_t);
                    pw = &dev->regs.rx_pw_p4.data;
                    break;
                case NRF_RX_PIPE_5:
                    dev->regs.rx_addr_p5.addr = cfg->address[0];
                    addr = &dev->regs.rx_addr_p5.addr;
                    addr_size = sizeof(nrf_rx_addr_pX_reg_t);
                    pw = &dev->regs.rx_pw_p5.data;
                    break;
                default:
                    break;
                }

                if (cfg->auto_ack)
                {
                    dev->regs.en_aa.data |= (1 << pipe);
                    if (cfg->dynamic_payload &&
                        (NRF_MAX_PAYLOAD_SIZE >= cfg->payload_width))
                    {
                        dev->regs.dynpd.data |= (1 << pipe);
                        *(&dev->regs.rx_pw_p0.data + pipe) = cfg->payload_width;
                    }
                    else
                    {
                        dev->regs.dynpd.data &= ~(1 << pipe);
                        *(&dev->regs.rx_pw_p0.data + pipe) = NRF_MAX_PAYLOAD_SIZE;
                    }
                }
                else
                {
                    dev->regs.en_aa.data &= ~(1 << pipe);
                    *(&dev->regs.rx_pw_p0.data + pipe) = NRF_MAX_PAYLOAD_SIZE;
                }
                dev->regs.en_rxaddr.data |= (1 << pipe);

                if (_nrf_write_register(dev, (NRF_RX_ADDR_P0_REG_ADDR + pipe),
                        addr, addr_size))
                {
                    return -1;
                }
                if (_nrf_write_register(dev, NRF_EN_AA_REG_ADDR,
                        &dev->regs.en_aa.data, sizeof(nrf_rx_addr_pX_reg_t)))
                {
                    return -1;
                }
                if (_nrf_write_register(dev, NRF_EN_RXADDR_REG_ADDR,
                        &dev->regs.en_rxaddr.data, sizeof(nrf_rx_addr_pX_reg_t)))
                {
                    return -1;
                }
                if (_nrf_write_register(dev, (NRF_RX_PW_P0_REG_ADDR + pipe),
                        pw, sizeof(nrf_rx_pw_reg_t)))
                {
                    return -1;
                }
                if (_nrf_write_register(dev, NRF_DYNPD_REG_ADDR,
                        &dev->regs.dynpd.data, sizeof(nrf_rx_addr_pX_reg_t)))
                {
                    return -1;
                }
            }
            else
            {
                break;
            }
        }
    }

    dev->state = NRF_STATE_POWER_DOWN;

    return 0;
}

static int _nrf_read(nrf_device_t *dev)
{
    //_nrf_ce_reset(dev);

    if (_nrf_read_status(dev))
    {
        nrf_log_error("%s: Failed to read status\n", __func__);
        return -1;
    }

    nrf_status_reg_t status;
    status.data = dev->rx.status;
    if (status.bits.tx_ds &&
        dev->regs.feature.bits.en_ack_pay &&
        (dev->last_rx_pipe < NRF_NUM_RX_PIPES))
    {
        (void)dev->callback(dev, NRF_CB_ACK_PAYLOAD_SENT, NULL, dev->last_rx_pipe, dev->user);
        if (dev->ack_payloads[dev->last_rx_pipe].payload)
        {
            if (_nrf_write_ack_payload(dev, dev->last_rx_pipe,
                    dev->ack_payloads[dev->last_rx_pipe].payload,
                    dev->ack_payloads[dev->last_rx_pipe].size))
            {
                nrf_log_error("%s: Failed to write ack payload\n", __func__);
                return -1;
            }
        }

        uint8_t reset_irq = NRF_STATUS_CLEAR_IRQ_TX_DS;
        if (_nrf_write_register(dev, NRF_STATUS_REG_ADDR, &reset_irq, sizeof(nrf_status_reg_t)))
        {
            nrf_log_error("%s: Failed to read and clear status\n", __func__);
            return -1;
        }
        status.data = dev->rx.status;
    }
    if (status.bits.rx_dr)
    {
        uint8_t reset_irq = NRF_STATUS_CLEAR_IRQ_RX_DR;
        nrf_fifo_status_reg_t fifo_status;
        do
        {
            uint8_t width = 0;
            uint8_t pipe = status.bits.rx_p_no;
            if (NRF_NUM_RX_PIPES <= pipe)
            {
                nrf_log_error("%s: Invalid pipe or empty fifo\n", __func__);
                (void)_nrf_flush_rx(dev);
                return -1;
            }
            if (dev->regs.feature.bits.en_dpl && (dev->regs.dynpd.data & (1 << pipe)))
            {
                if (_nrf_read_rx_payload_width(dev, &width) || (NRF_MAX_PAYLOAD_SIZE < width))
                {
                    nrf_log_error("%s: Failed to read payload width\n", __func__);
                    (void)_nrf_flush_rx(dev);
                    return -1;
                }
            }
            else
            {
                uint8_t *pipe_wl = (uint8_t *)&dev->regs.rx_pw_p0;
                width = pipe_wl[pipe];
            }

            if (_nrf_read_payload(dev, width))
            {
                nrf_log_error("%s: Failed to read payload\n", __func__);
                (void)_nrf_flush_rx(dev);
                return -1;
            }
            dev->last_rx_pipe = pipe;
            (void)dev->callback(dev, NRF_CB_READ_PAYLOAD, dev->rx.payload, width, dev->user);

            if (status.bits.rx_dr &&
                _nrf_write_register(dev, NRF_STATUS_REG_ADDR, &reset_irq, sizeof(nrf_status_reg_t)))
            {
                nrf_log_error("%s: Failed to read and clear status\n", __func__);
                return -1;
            }

            if (_nrf_read_register(dev, NRF_FIFO_STATUS_REG_ADDR,
                    &fifo_status.data, sizeof(nrf_fifo_status_reg_t)))
            {
                nrf_log_error("%s: Failed to read fifo status\n", __func__);
                (void)_nrf_flush_rx(dev);
                return -1;
            }

            status.data = dev->rx.status;
        } while (!fifo_status.bits.rx_empty);
    }

    return 0;
}

static int _nrf_write_error(nrf_device_t *dev)
{
    _nrf_ce_reset(dev);
    (void)_nrf_flush_tx(dev);
    dev->payload = NULL;
    dev->payload_size = 0;
    dev->tx_pending = false;
    (void)dev->callback(dev, NRF_CB_WRITE_ERROR, NULL, 0, dev->user);
    (void)nrf_set_state(dev, dev->prev_state);
    return -1;
}

static int _nrf_write_continue(nrf_device_t *dev)
{
    if (_nrf_read_status(dev))
    {
        nrf_log_error("%s: Failed to read status\n", __func__);
        return -1;
    }

    nrf_status_reg_t status;
    status.data = dev->rx.status;
    if (status.bits.max_rt)
    {
        nrf_log_error("%s: Reached max retransmissions\n", __func__);
        return _nrf_write_error(dev);
    }
    else if (status.bits.tx_ds || status.bits.rx_dr)
    {
        if (status.bits.rx_dr)
        {
            nrf_fifo_status_reg_t fifo_status;
            do
            {
                uint8_t width = dev->regs.rx_pw_p0.bits.rx_pw;
                if (dev->regs.feature.bits.en_dpl && dev->regs.dynpd.bits.dpl_p0)
                {
                    if ((NRF_MAX_PAYLOAD_SIZE < width) || _nrf_read_rx_payload_width(dev, &width))
                    {
                        nrf_log_error("%s: Failed to read payload width\n", __func__);
                        (void)_nrf_flush_rx(dev);
                        return -1;
                    }
                }
                if (_nrf_read_payload(dev, width))
                {
                    nrf_log_error("%s: Failed to read ack payload\n", __func__);
                    return _nrf_write_error(dev);
                }
                (void)dev->callback(dev, NRF_CB_READ_ACK_PAYLOAD, dev->rx.payload, width, dev->user);

                if (_nrf_read_register(dev, NRF_FIFO_STATUS_REG_ADDR,
                        &fifo_status.data, sizeof(nrf_fifo_status_reg_t)))
                {
                    nrf_log_error("%s: Failed to read fifo status\n", __func__);
                    (void)_nrf_flush_rx(dev);
                    return -1;
                }
            } while (!fifo_status.bits.rx_empty);
            status.data = dev->rx.status;
        }
        if (status.bits.tx_ds)
        {
            if (dev->payload_size)
            {
                int err = 0;
                size_t len = NRF_MIN(dev->payload_size, NRF_MAX_PAYLOAD_SIZE);
                if (dev->no_ack)
                {
                    err = _nrf_write_payload_no_ack(dev, dev->payload, len);
                }
                else
                {
                    err = _nrf_write_payload(dev, dev->payload, len);
                }
                if (err)
                {
                    nrf_log_error("%s: Failed to write payload\n", __func__);
                    return _nrf_write_error(dev);
                }
                dev->payload += len;
                dev->payload_size -= len;
                (void)dev->callback(dev, NRF_CB_WFI, NULL, 0, dev->user);
            }
            else
            {
                nrf_fifo_status_reg_t fifo_status;
                if (_nrf_read_register(dev, NRF_FIFO_STATUS_REG_ADDR,
                        &fifo_status.data, sizeof(nrf_fifo_status_reg_t)))
                {
                    nrf_log_error("%s: Failed to read fifo status\n", __func__);
                    return _nrf_write_error(dev);
                }
                if (fifo_status.bits.tx_empty)
                {
                    _nrf_ce_reset(dev);
                    (void)dev->callback(dev, NRF_CB_WRITE_COMPLETE, NULL, 0, dev->user);
                    dev->payload = NULL;
                    dev->payload_size = 0;
                    dev->tx_pending = false;
                    (void)nrf_set_state(dev, dev->prev_state);
                }
                else
                {
                    (void)dev->callback(dev, NRF_CB_WFI, NULL, 0, dev->user);
                }
            }
        }
    }

    uint8_t reset_irq = NRF_STATUS_CLEAR_IRQ_ALL;
    if (_nrf_write_register(dev, NRF_STATUS_REG_ADDR, &reset_irq, sizeof(nrf_status_reg_t)))
    {
        nrf_log_error("%s: Failed to read and clear status\n", __func__);
        return -1;
    }

    return 0;
}

static int _nrf_write(nrf_device_t *dev, const uint8_t *addr, size_t addr_size,
                      const uint8_t *data, size_t data_size, bool no_ack)
{
    const size_t aw = (dev->regs.setup_aw.bits.aw + 2);
    if (!addr || (aw > addr_size) || !data || !data_size)
    {
        nrf_log_error("%s: Invalid args\n", __func__);
        return -1;
    }

    if (NRF_STATE_POWER_DOWN > dev->state)
    {
        nrf_log_error("%s: Device not initialized\n", __func__);
        return -1;
    }

    if (dev->tx_pending)
    {
        nrf_log_error("%s: Device busy, write in progress\n", __func__);
        return -1;
    }

    dev->prev_state = dev->state;

    (void)_nrf_flush_tx(dev);

    if (!dev->regs.config.bits.pwr_up || dev->regs.config.bits.prim_rx)
    {
        dev->regs.config.bits.pwr_up = 1;
        dev->regs.config.bits.prim_rx = 0;
        if (_nrf_write_register(dev, NRF_CONFIG_REG_ADDR,
                &dev->regs.config.data, sizeof(nrf_config_reg_t)))
        {
            return -1;
        }
        dev->state = NRF_STATE_STANDBY;
    }

    dev->payload = data;
    dev->payload_size = data_size;
    dev->no_ack = no_ack;
    if (memcmp(dev->regs.tx_addr.addr, addr, aw))
    {
        memcpy(dev->regs.tx_addr.addr, addr, aw);
        if (_nrf_write_register(dev, NRF_TX_ADDR_REG_ADDR,
                dev->regs.tx_addr.addr, aw))
        {
            return -1;
        }
    }
    if (!dev->no_ack)
    {
        if (_nrf_write_register(dev, NRF_RX_ADDR_P0_REG_ADDR,
                dev->regs.tx_addr.addr, aw))
        {
            return -1;
        }
    }

    int fifo_not_empty = dev->regs.feature.bits.en_ack_pay ? 1 : NRF_FIFO_LEVELS;
    do
    {
        int err = 0;
        size_t len = NRF_MIN(dev->payload_size, NRF_MAX_PAYLOAD_SIZE);
        if (dev->no_ack)
        {
            err = _nrf_write_payload_no_ack(dev, dev->payload, len);
        }
        else
        {
            err = _nrf_write_payload(dev, dev->payload, len);
        }
        if (err)
        {
            nrf_log_error("%s: Failed to write payload\n", __func__);
            return _nrf_write_error(dev);
        }
        dev->payload += len;
        dev->payload_size -= len;
        --fifo_not_empty;
    } while (dev->payload_size && fifo_not_empty);

    dev->tx_pending = true;

    _nrf_ce_set(dev);

    (void)dev->callback(dev, NRF_CB_WFI, NULL, 0, dev->user);

    return 0;
}

int nrf_service_interrupt(nrf_handle_t handle)
{
    if (!handle)
    {
        return -1;
    }

    nrf_device_t *dev = (nrf_device_t *)handle;
    if (dev->tx_pending)
    {
        return _nrf_write_continue(dev);
    }
    else if (NRF_STATE_RX == dev->state)
    {
        return _nrf_read(dev);
    }
    else
    {
        uint8_t reset_irq = NRF_STATUS_CLEAR_IRQ_ALL;
        if (_nrf_write_register(dev, NRF_STATUS_REG_ADDR, &reset_irq, sizeof(nrf_status_reg_t)))
        {
            nrf_log_error("%s: Failed to read and clear status\n", __func__);
            return -1;
        }
    }

    return 0;
}

nrf_state_t nrf_get_state(nrf_handle_t handle)
{
    nrf_state_t state = NRF_STATE_UNKNOWN;

    if (handle)
    {
        state = ((nrf_device_t *)handle)->state;
    }

    return state;
}

int nrf_set_state(nrf_handle_t handle, nrf_state_t state)
{
    if (!handle)
    {
        return -1;
    }

    nrf_device_t *dev = (nrf_device_t *)handle;
    if (dev->state == state)
    {
        return 0;
    }

    if (NRF_STATE_UNKNOWN == dev->state)
    {
        nrf_log_error("%s: Device not initialized\n", __func__);
        return -1;
    }

    if (dev->tx_pending)
    {
        nrf_log_error("%s: Device busy, write in progress\n", __func__);
        return -1;
    }

    if (_nrf_flush_rx(dev) || _nrf_flush_tx(dev))
    {
        return -1;
    }

    int err = 0;
    switch (state)
    {
    case NRF_STATE_UNKNOWN:
    case NRF_STATE_POWER_DOWN:
        dev->regs.config.bits.pwr_up = 0;
        err = _nrf_write_register(dev, NRF_CONFIG_REG_ADDR,
                &dev->regs.config.data, sizeof(nrf_config_reg_t));
        _nrf_ce_reset(dev);
        break;
    case NRF_STATE_STANDBY:
        dev->regs.config.bits.pwr_up = 1;
        dev->regs.config.bits.prim_rx = 0;
        err = _nrf_write_register(dev, NRF_CONFIG_REG_ADDR,
                &dev->regs.config.data, sizeof(nrf_config_reg_t));
        _nrf_ce_reset(dev);
        break;
    case NRF_STATE_RX:
        dev->regs.config.bits.pwr_up = 1;
        dev->regs.config.bits.prim_rx = 1;
        err = _nrf_write_register(dev, NRF_CONFIG_REG_ADDR,
                &dev->regs.config.data, sizeof(nrf_config_reg_t));
        if (!err && dev->regs.feature.bits.en_ack_pay)
        {
            for (int p = 0, w = 0; (p < NRF_NUM_RX_PIPES) && (w < NRF_FIFO_LEVELS); ++p)
            {
                if ((dev->regs.en_rxaddr.data & (1 << p)) &&
                    dev->ack_payloads[p].payload &&
                    dev->ack_payloads[p].size)
                {
                    err = _nrf_write_ack_payload(dev, p, dev->ack_payloads[p].payload,
                            dev->ack_payloads[p].size);
                    if (err)
                    {
                        nrf_log_error("%s: Failed to write ack payloads\n", __func__);
                        break;
                    }
                    ++w;
                }
            }
        }
        if (!err)
        {
            _nrf_ce_set(dev);
        }
        break;
    default:
        nrf_log_error("%s: Invalid state: %d\n", __func__, state);
        return -1;
    }

    if (!err)
    {
        dev->state = state;
    }

    return err;
}

int nrf_write(nrf_handle_t handle, const uint8_t *addr, size_t addr_size,
              const uint8_t *data, size_t data_size)
{
    if (!handle)
    {
        return -1;
    }

    nrf_device_t *dev = (nrf_device_t *)handle;
    return _nrf_write(dev, addr, addr_size, data, data_size, false);
}

int nrf_write_no_ack(nrf_handle_t handle, const uint8_t *addr, size_t addr_size,
                     const uint8_t *data, size_t data_size)
{
    if (!handle)
    {
        return -1;
    }

    nrf_device_t *dev = (nrf_device_t *)handle;
    if (!dev->regs.feature.bits.en_dyn_ack)
    {
        nrf_log_error("%s: Dynamic ack not enabled\n", __func__);
        return -1;
    }
    else
    {
        return _nrf_write(dev, addr, addr_size, data, data_size, true);
    }
}

int nrf_reset_ack_payload(nrf_handle_t handle, nrf_pipe_t pipe)
{
    if (!handle)
    {
        return -1;
    }

    nrf_device_t *dev = (nrf_device_t *)handle;
    if (NRF_NUM_RX_PIPES <= pipe)
    {
        nrf_log_error("%s: Invalid pipe number\n", __func__);
        return -1;
    }

    dev->ack_payloads[pipe].payload = NULL;
    dev->ack_payloads[pipe].size = 0;

    return 0;
}

int nrf_set_ack_payload(nrf_handle_t handle, nrf_pipe_t pipe,
                        const uint8_t *ack_payload, size_t ack_payload_size)
{
    if (!handle)
    {
        return -1;
    }

    nrf_device_t *dev = (nrf_device_t *)handle;
    if (NRF_NUM_RX_PIPES <= pipe)
    {
        nrf_log_error("%s: Invalid pipe number\n", __func__);
        return -1;
    }
    if (0 == (dev->regs.en_rxaddr.data & (1 << pipe)))
    {
        nrf_log_error("%s: Pipe not enabled\n", __func__);
        return -1;
    }
    if (!dev->regs.feature.bits.en_ack_pay)
    {
        nrf_log_error("%s: Ack payloads not enabled\n", __func__);
        return -1;
    }

    dev->ack_payloads[pipe].payload = ack_payload;
    dev->ack_payloads[pipe].size = NRF_MIN(ack_payload_size, NRF_MAX_PAYLOAD_SIZE);

    if ((NRF_STATE_RX == dev->state) && dev->ce_state)
    {
        /* Rewrite the updated ack payloads to device tx fifo */
        (void)_nrf_flush_tx(dev);
        for (int p = 0, w = 0; (p < NRF_NUM_RX_PIPES) && (w < NRF_FIFO_LEVELS); ++p)
        {
            if (dev->ack_payloads[p].payload && dev->ack_payloads[p].size)
            {
                if (_nrf_write_ack_payload(dev, p, dev->ack_payloads[p].payload,
                        dev->ack_payloads[p].size))
                {
                    nrf_log_error("%s: Failed to write ack payloads\n", __func__);
                    return -1;
                }
                ++w;
            }
        }
    }

    return 0;
}

void nrf_device_free(nrf_handle_t handle)
{
    if (handle)
    {
        nrf_free(handle);
    }
}

nrf_handle_t nrf_device_create(nrf_callback_t callback)
{
    if (!callback)
    {
        nrf_log_error("%s: No valid callback supplied\n", __func__);
        return NULL;
    }

    nrf_device_t *dev = nrf_malloc(sizeof(nrf_device_t));
    if (!dev)
    {
        nrf_log_error("%s: Failed to allocate context\n", __func__);
        return NULL;
    }

    memset(dev, 0, sizeof(nrf_device_t));

    dev->callback = callback;

    return dev;
}

size_t nrf_device_size(void)
{
    return sizeof(nrf_device_t);
}

nrf_handle_t nrf_device_create_from_buffer(void *buffer, size_t *buffer_size, nrf_callback_t callback)
{
    if (!buffer || !buffer_size)
    {
        nrf_log_error("%s: No valid buffer supplied\n", __func__);
        return NULL;
    }

    if (sizeof(nrf_device_t) > *buffer_size)
    {
        nrf_log_error("%s: Buffer size too small\n", __func__);
        *buffer_size = sizeof(nrf_device_t);
        return NULL;
    }

    if (!callback)
    {
        nrf_log_error("%s: No valid callback supplied\n", __func__);
        return NULL;
    }

    nrf_device_t *dev = (nrf_device_t *)buffer;
    memset(dev, 0, sizeof(nrf_device_t));

    dev->callback = callback;

    return (nrf_handle_t)dev;
}

uint8_t nrf_get_last_rx_pipe(nrf_handle_t handle)
{
    if (!handle)
    {
        return 0xFF;
    }

    return ((nrf_device_t *)handle)->last_rx_pipe;
}

int nrf_register_dump(nrf_handle_t handle)
{
    if (!handle)
    {
        return -1;
    }

    nrf_device_t *dev = (nrf_device_t *)handle;
    uint8_t data[NRF_REG_MAX_SIZE];

    nrf_log_debug("%s: register dump for device handle %p\n", __func__, handle);
    for (uint8_t reg = NRF_CONFIG_REG_ADDR; reg < NRF_INVALID_REG_ADDR;)
    {
        size_t reg_size = 1;
        switch (reg)
        {
        case NRF_RX_ADDR_P0_REG_ADDR:
        case NRF_RX_ADDR_P1_REG_ADDR:
        case NRF_TX_ADDR_REG_ADDR:
            reg_size = NRF_REG_MAX_SIZE;
            break;
        default:
            break;
        }

        if (_nrf_read_register(dev, reg, data, reg_size))
        {
            return -1;
        }

        if (NRF_REG_MAX_SIZE != reg_size)
        {
            nrf_log_debug("%02X: %02X\n", reg, data[0]);
        }
        else
        {
            nrf_log_debug("%02X: %02X %02X %02X %02X %02X\n", reg, data[0], data[1], data[2], data[3], data[4]);
        }

        if (NRF_FIFO_STATUS_REG_ADDR == reg)
        {
            reg = NRF_DYNPD_REG_ADDR;
        }
        else
        {
            ++reg;
        }
    }

    return 0;
}

int nrf_register_dump_tlv(nrf_handle_t handle, uint8_t *buffer, size_t size)
{
    if (!handle || !buffer)
    {
        return -1;
    }

    if (NRF_REG_TLV_SIZE > size)
    {
        nrf_log_error("%s: Buffer size too small\n", __func__);
        return -1;
    }

    nrf_device_t *dev = (nrf_device_t *)handle;
    uint8_t *p = buffer;

    nrf_log_debug("%s: register dump for device handle %p\n", __func__, handle);

    for (uint8_t reg = NRF_CONFIG_REG_ADDR; reg < NRF_INVALID_REG_ADDR;)
    {
        uint8_t reg_size = 1;
        switch (reg)
        {
        case NRF_RX_ADDR_P0_REG_ADDR:
        case NRF_RX_ADDR_P1_REG_ADDR:
        case NRF_TX_ADDR_REG_ADDR:
            reg_size = NRF_REG_MAX_SIZE;
            break;
        default:
            break;
        }

        *p++ = reg;
        *p++ = reg_size;
        if (_nrf_read_register(dev, reg, p, reg_size))
        {
            return -1;
        }
        p += reg_size;

        if (NRF_FIFO_STATUS_REG_ADDR == reg)
        {
            reg = NRF_DYNPD_REG_ADDR;
        }
        else
        {
            ++reg;
        }
    }

    return 0;
}
