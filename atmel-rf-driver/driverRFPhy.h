/*
 * Copyright (c) 2014-2015 ARM Limited. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef DRIVERRFPHY_H_
#define DRIVERRFPHY_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "nanostack/platform/arm_hal_phy.h"

/*AT86RF212 PHY Modes*/
#define BPSK_20                     0x00
#define BPSK_40                     0x04
#define BPSK_40_ALT                 0x14
#define OQPSK_SIN_RC_100            0x08
#define OQPSK_SIN_RC_200            0x09
#define OQPSK_RC_100                0x18
#define OQPSK_RC_200                0x19
#define OQPSK_SIN_250               0x0c
#define OQPSK_SIN_500               0x0d
#define OQPSK_SIN_500_ALT           0x0f
#define OQPSK_RC_250                0x1c
#define OQPSK_RC_500                0x1d
#define OQPSK_RC_500_ALT            0x1f
#define OQPSK_SIN_RC_400_SCR_ON     0x2A
#define OQPSK_SIN_RC_400_SCR_OFF    0x0A
#define OQPSK_RC_400_SCR_ON         0x3A
#define OQPSK_RC_400_SCR_OFF        0x1A
#define OQPSK_SIN_1000_SCR_ON       0x2E
#define OQPSK_SIN_1000_SCR_OFF      0x0E
#define OQPSK_RC_1000_SCR_ON        0x3E
#define OQPSK_RC_1000_SCR_OFF       0x1E

/*Worst case sensitivity*/
#define RF_DEFAULT_SENSITIVITY -88
/*Run calibration every 5 minutes*/
#define RF_CALIBRATION_INTERVAL 6000000
/*Wait ACK for 2.5ms*/
#define RF_ACK_WAIT_DEFAULT_TIMEOUT 50
/*CCA Timeout*/
#define RF_CCA_TIMEOUT  15

#define RF_BUFFER_SIZE 128

#define RF_PHY_MODE OQPSK_SIN_250

/*Radio RX and TX state definitions*/
#define RFF_ON 0x01
#define RFF_RX 0x02
#define RFF_TX 0x04
#define RFF_CCA 0x08

#define RF_MODE_NORMAL  0
#define RF_MODE_SNIFFER 1

/*Atmel RF Part Type*/
typedef enum
{
    ATMEL_UNKNOW_DEV = 0,
    ATMEL_AT86RF212,
    ATMEL_AT86RF231,
    ATMEL_AT86RF233
}rf_trx_part_e;

/*Atmel RF states*/
typedef enum
{
    NOP = 0x00,
    BUSY_RX = 0x01,
    RF_TX_START = 0x02,
    FORCE_TRX_OFF = 0x03,
    FORCE_PLL_ON = 0x04,
    RX_ON = 0x06,
    TRX_OFF = 0x08,
    PLL_ON = 0x09,
    BUSY_RX_AACK = 0x11,
    SLEEP = 0x0F,
    RX_AACK_ON = 0x16,
    TX_ARET_ON = 0x19
}rf_trx_states_t;

#define RF_TX_POWER_MAX 0

extern rf_trx_part_e rf_radio_type_read(void);

extern void rf_ack_wait_timer_start(uint16_t slots);
extern void rf_ack_wait_timer_stop(void);
extern void rf_handle_cca_ed_done(void);
extern void rf_handle_tx_end(void);
extern void rf_handle_rx_end(void);
extern void rf_on(void);
extern void rf_receive(void);
extern void rf_poll_trx_state_change(rf_trx_states_t trx_state);
extern void rf_init(void);
extern void rf_set_mac_address(const uint8_t *ptr);
extern int8_t rf_device_register(void);
extern int8_t rf_start_cca(uint8_t *data_ptr, uint16_t data_length, uint8_t tx_handle, data_protocol_e data_protocol );
extern void rf_cca_abort(void);
extern void rf_read_mac_address(uint8_t *ptr);
extern int8_t rf_read_random(void);
extern void rf_calibration_cb(void);
extern void rf_init_phy_mode(void);
extern void rf_ack_wait_timer_interrupt(void);
extern void rf_calibration_timer_interrupt(void);
extern void rf_calibration_timer_start(uint32_t slots);
extern void rf_cca_timer_interrupt(void);
extern void rf_cca_timer_start(uint32_t slots);
extern uint16_t rf_get_phy_mtu_size(void);
extern uint8_t rf_scale_lqi(int8_t rssi);

/**
 *  RF output power write
 *
 * \brief TX power has to be set before network start.
 *
 * \param power
 *              AT86RF233
 *              0 = 4 dBm
 *              1 = 3.7 dBm
 *              2 = 3.4 dBm
 *              3 = 3 dBm
 *              4 = 2.5 dBm
 *              5 = 2 dBm
 *              6 = 1 dBm
 *              7 = 0 dBm
 *              8 = -1 dBm
 *              9 = -2 dBm
 *              10 = -3 dBm
 *              11 = -4 dBm
 *              12 = -6 dBm
 *              13 = -8 dBm
 *              14 = -12 dBm
 *              15 = -17 dBm
 *
 *              AT86RF212B
 *              See datasheet for TX power settings
 *
 * \return 0, Supported Value
 * \return -1, Not Supported Value
 */
extern int8_t rf_tx_power_set(uint8_t power);
#ifdef __cplusplus
}
#endif
#endif /* DRIVERRFPHY_H_ */
