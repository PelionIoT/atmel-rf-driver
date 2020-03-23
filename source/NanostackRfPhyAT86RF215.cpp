/*
 * Copyright (c) 2020 ARM Limited. All rights reserved.
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

#include <string.h>

#if defined(MBED_CONF_NANOSTACK_CONFIGURATION) && DEVICE_SPI && DEVICE_INTERRUPTIN && defined(MBED_CONF_RTOS_PRESENT)

#include "ns_types.h"
#include "platform/arm_hal_interrupt.h"
#include "platform/mbed_wait_api.h"
#include "nanostack/platform/arm_hal_phy.h"
#include "NanostackRfPhyAtmel.h"
#include "AT86RF215Reg.h"
#include "mbed_trace.h"
#include "common_functions.h"
#include <Timer.h>
#include "Timeout.h"
#include "SPI.h"

#define TRACE_GROUP "AtRF"

#define RF_MTU_15_4_2011    127
#define RF_MTU_15_4G_2012   2047

namespace {

typedef enum {
    RF_NOP = 0x00,
    RF_SLEEP = 0x01,
    RF_TRX_OFF = 0x02,
    RF_TXPREP = 0x03,
    RF_TX = 0x04,
    RF_RX = 0x05,
    RF_TRANSITION = 0x06,
    RF_RESET = 0x07
} rf_command_e;

typedef enum {
    COMMON = 0x00,
    RF_09 = 0x01,
    RF_24 = 0x02,
    BBC0 = 0x03,
    BBC1 = 0x04
} rf_modules_e;

typedef enum {
    RF_IDLE,
    RF_CSMA_STARTED,
    RF_CSMA_WHILE_RX,
    RF_TX_STARTED,
    RF_RX_STARTED
} rf_states_e;

} // anonymous namespace

static void rf_init(void);
static int8_t rf_address_write(phy_address_type_e address_type, uint8_t *address_ptr);
static int8_t rf_extension(phy_extension_type_e extension_type, uint8_t *data_ptr);
static int8_t rf_interface_state_control(phy_interface_state_e new_state, uint8_t rf_channel);
static int8_t rf_start_csma_ca(uint8_t *data_ptr, uint16_t data_length, uint8_t tx_handle, data_protocol_e data_protocol);
static void rf_init_registers(rf_modules_e module);
static void rf_spi_exchange(const void *tx, size_t tx_len, void *rx, size_t rx_len);
static uint8_t rf_read_rf_register(uint8_t addr, rf_modules_e module);
static void rf_write_rf_register(uint8_t addr, rf_modules_e module, uint8_t data);
static void rf_write_rf_register_field(uint8_t addr, rf_modules_e module, uint8_t field, uint8_t value);
static void rf_write_tx_packet_length(uint16_t packet_length, rf_modules_e module);
static uint16_t rf_read_rx_frame_length(rf_modules_e module);
static void rf_write_tx_buffer(uint8_t *data, uint16_t len, rf_modules_e module);
static int rf_read_rx_buffer(uint16_t length, rf_modules_e module);
static void rf_irq_rf_enable(uint8_t irq, rf_modules_e module);
static void rf_irq_rf_disable(uint8_t irq, rf_modules_e module);
static void rf_irq_bbc_enable(uint8_t irq, rf_modules_e module);
static void rf_irq_bbc_disable(uint8_t irq, rf_modules_e module);
static rf_command_e rf_read_state(rf_modules_e module);
static void rf_poll_state_change(rf_command_e state, rf_modules_e module);
static void rf_change_state(rf_command_e state, rf_modules_e module);
static void rf_receive(uint16_t rx_channel, rf_modules_e module);
static void rf_interrupt_handler(void);
static void rf_irq_task_process_irq(void);
static void rf_handle_cca_ed_done(void);
static void rf_start_tx(void);
static void rf_backup_timer_interrupt(void);
static void rf_backup_timer_stop(void);
static void rf_backup_timer_start(uint32_t slots);
static int rf_set_channel(uint16_t channel, rf_modules_e module);
static int rf_set_ch0_frequency(uint32_t frequency, rf_modules_e module);
static int rf_set_channel_spacing(uint32_t channel_spacing, rf_modules_e module);
static int rf_set_fsk_symbol_rate_configuration(uint32_t symbol_rate, rf_modules_e module);
static void rf_calculate_symbol_rate(uint32_t baudrate, phy_modulation_e modulation);
static void rf_conf_set_cca_threshold(uint8_t percent);
// Defined register read/write functions
#define rf_read_bbc_register(x, y) rf_read_rf_register(x, (rf_modules_e)(y + 2))
#define rf_read_common_register(x) rf_read_rf_register(x, COMMON)
#define rf_write_bbc_register(x, y, z) rf_write_rf_register(x, (rf_modules_e)(y + 2), z)
#define rf_write_common_register(x, z) rf_write_rf_register(x, COMMON, z)
#define rf_write_bbc_register_field(v, x, y, z) rf_write_rf_register_field(v, (rf_modules_e)(x + 2), y, z)
#define rf_write_common_register_field(v, y, z) rf_write_rf_register_field(v, COMMON, y, z)

static int8_t rf_radio_driver_id = -1;
static phy_device_driver_s device_driver;
static uint8_t rf_version_num = 0;
static rf_modules_e rf_module = RF_24;
static uint8_t mac_tx_handle = 0;
static rf_states_e rf_state = RF_IDLE;
static uint8_t rx_buffer[RF_MTU_15_4G_2012];
static uint8_t rf_rx_channel;
static uint8_t rf_new_channel;
static uint16_t tx_sequence = 0xffff;
static uint32_t tx_time = 0;
static uint32_t rx_time = 0;
static uint8_t rf09_irq_mask = 0;
static uint8_t rf24_irq_mask = 0;
static uint8_t bbc0_irq_mask = 0;
static uint8_t bbc1_irq_mask = 0;

const uint32_t crc32_lookup_table[256] = {
        0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9, 0x130476dc, 0x17c56b6b, 0x1a864db2, 0x1e475005,
        0x2608edb8, 0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61, 0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd,
        0x4c11db70, 0x48d0c6c7, 0x4593e01e, 0x4152fda9, 0x5f15adac, 0x5bd4b01b, 0x569796c2, 0x52568b75,
        0x6a1936c8, 0x6ed82b7f, 0x639b0da6, 0x675a1011, 0x791d4014, 0x7ddc5da3, 0x709f7b7a, 0x745e66cd,
        0x9823b6e0, 0x9ce2ab57, 0x91a18d8e, 0x95609039, 0x8b27c03c, 0x8fe6dd8b, 0x82a5fb52, 0x8664e6e5,
        0xbe2b5b58, 0xbaea46ef, 0xb7a96036, 0xb3687d81, 0xad2f2d84, 0xa9ee3033, 0xa4ad16ea, 0xa06c0b5d,
        0xd4326d90, 0xd0f37027, 0xddb056fe, 0xd9714b49, 0xc7361b4c, 0xc3f706fb, 0xceb42022, 0xca753d95,
        0xf23a8028, 0xf6fb9d9f, 0xfbb8bb46, 0xff79a6f1, 0xe13ef6f4, 0xe5ffeb43, 0xe8bccd9a, 0xec7dd02d,
        0x34867077, 0x30476dc0, 0x3d044b19, 0x39c556ae, 0x278206ab, 0x23431b1c, 0x2e003dc5, 0x2ac12072,
        0x128e9dcf, 0x164f8078, 0x1b0ca6a1, 0x1fcdbb16, 0x018aeb13, 0x054bf6a4, 0x0808d07d, 0x0cc9cdca,
        0x7897ab07, 0x7c56b6b0, 0x71159069, 0x75d48dde, 0x6b93dddb, 0x6f52c06c, 0x6211e6b5, 0x66d0fb02,
        0x5e9f46bf, 0x5a5e5b08, 0x571d7dd1, 0x53dc6066, 0x4d9b3063, 0x495a2dd4, 0x44190b0d, 0x40d816ba,
        0xaca5c697, 0xa864db20, 0xa527fdf9, 0xa1e6e04e, 0xbfa1b04b, 0xbb60adfc, 0xb6238b25, 0xb2e29692,
        0x8aad2b2f, 0x8e6c3698, 0x832f1041, 0x87ee0df6, 0x99a95df3, 0x9d684044, 0x902b669d, 0x94ea7b2a,
        0xe0b41de7, 0xe4750050, 0xe9362689, 0xedf73b3e, 0xf3b06b3b, 0xf771768c, 0xfa325055, 0xfef34de2,
        0xc6bcf05f, 0xc27dede8, 0xcf3ecb31, 0xcbffd686, 0xd5b88683, 0xd1799b34, 0xdc3abded, 0xd8fba05a,
        0x690ce0ee, 0x6dcdfd59, 0x608edb80, 0x644fc637, 0x7a089632, 0x7ec98b85, 0x738aad5c, 0x774bb0eb,
        0x4f040d56, 0x4bc510e1, 0x46863638, 0x42472b8f, 0x5c007b8a, 0x58c1663d, 0x558240e4, 0x51435d53,
        0x251d3b9e, 0x21dc2629, 0x2c9f00f0, 0x285e1d47, 0x36194d42, 0x32d850f5, 0x3f9b762c, 0x3b5a6b9b,
        0x0315d626, 0x07d4cb91, 0x0a97ed48, 0x0e56f0ff, 0x1011a0fa, 0x14d0bd4d, 0x19939b94, 0x1d528623,
        0xf12f560e, 0xf5ee4bb9, 0xf8ad6d60, 0xfc6c70d7, 0xe22b20d2, 0xe6ea3d65, 0xeba91bbc, 0xef68060b,
        0xd727bbb6, 0xd3e6a601, 0xdea580d8, 0xda649d6f, 0xc423cd6a, 0xc0e2d0dd, 0xcda1f604, 0xc960ebb3,
        0xbd3e8d7e, 0xb9ff90c9, 0xb4bcb610, 0xb07daba7, 0xae3afba2, 0xaafbe615, 0xa7b8c0cc, 0xa379dd7b,
        0x9b3660c6, 0x9ff77d71, 0x92b45ba8, 0x9675461f, 0x8832161a, 0x8cf30bad, 0x81b02d74, 0x857130c3,
        0x5d8a9099, 0x594b8d2e, 0x5408abf7, 0x50c9b640, 0x4e8ee645, 0x4a4ffbf2, 0x470cdd2b, 0x43cdc09c,
        0x7b827d21, 0x7f436096, 0x7200464f, 0x76c15bf8, 0x68860bfd, 0x6c47164a, 0x61043093, 0x65c52d24,
        0x119b4be9, 0x155a565e, 0x18197087, 0x1cd86d30, 0x029f3d35, 0x065e2082, 0x0b1d065b, 0x0fdc1bec,
        0x3793a651, 0x3352bbe6, 0x3e119d3f, 0x3ad08088, 0x2497d08d, 0x2056cd3a, 0x2d15ebe3, 0x29d4f654,
        0xc5a92679, 0xc1683bce, 0xcc2b1d17, 0xc8ea00a0, 0xd6ad50a5, 0xd26c4d12, 0xdf2f6bcb, 0xdbee767c,
        0xe3a1cbc1, 0xe760d676, 0xea23f0af, 0xeee2ed18, 0xf0a5bd1d, 0xf464a0aa, 0xf9278673, 0xfde69bc4,
        0x89b8fd09, 0x8d79e0be, 0x803ac667, 0x84fbdbd0, 0x9abc8bd5, 0x9e7d9662, 0x933eb0bb, 0x97ffad0c,
        0xafb010b1, 0xab710d06, 0xa6322bdf, 0xa2f33668, 0xbcb4666d, 0xb8757bda, 0xb5365d03, 0xb1f740b4
};

static bool rf_update_config = false;
static int8_t cca_threshold = -80;
static bool cca_enabled = true;
static uint32_t rf_symbol_rate;

/* Channel configurations for 2.4 and sub-GHz */
static const phy_rf_channel_configuration_s phy_24ghz = {.channel_0_center_frequency = 2350000000U,
                                                         .channel_spacing = 5000000U,
                                                         .datarate = 250000U,
                                                         .number_of_channels = 16U,
                                                         .modulation = M_OQPSK};
static const phy_rf_channel_configuration_s phy_subghz = {.channel_0_center_frequency = 868300000U,
                                                          .channel_spacing = 2000000U,
                                                          .datarate = 250000U,
                                                          .number_of_channels = 11U,
                                                          .modulation = M_OQPSK};

static phy_rf_channel_configuration_s phy_current_config;

static const phy_device_channel_page_s phy_channel_pages[] = {
    { CHANNEL_PAGE_0, &phy_24ghz},
    { CHANNEL_PAGE_2, &phy_subghz},
    { CHANNEL_PAGE_0, NULL}
};

using namespace mbed;
using namespace rtos;

#include "rfbits.h"
static RFBits *rf;
static TestPins *test_pins;

#define MAC_FRAME_TYPE_MASK     0x07
#define MAC_TYPE_ACK            (2)
#define MAC_DATA_PENDING        0x10
#define FC_AR                   0x20
#define VERSION_FIELD_MASK      0x30
#define SHIFT_VERSION_FIELD     (4)

#define SIG_RADIO           1
#define SIG_TIMER_BACKUP    2
#define SIG_TIMER_CCA       4
#define SIG_TIMERS (SIG_TIMER_BACKUP|SIG_TIMER_CCA)
#define SIG_ALL (SIG_RADIO|SIG_TIMERS)

#define ACK_FRAME_LENGTH    3
// Give some additional time for processing, PHY headers, CRC etc.
#define PACKET_SENDING_EXTRA_TIME   5000
#define MAX_PACKET_SENDING_TIME (uint32_t)(8000000/phy_current_config.datarate)*device_driver.phy_MTU + PACKET_SENDING_EXTRA_TIME
#define ACK_SENDING_TIME (uint32_t)(8000000/phy_current_config.datarate)*ACK_FRAME_LENGTH + PACKET_SENDING_EXTRA_TIME

#define MAX_STATE_TRANSITION_TIME_US    1000

#define MIN_CCA_THRESHOLD  -117
#define MAX_CCA_THRESHOLD  -5

static uint32_t rf_get_timestamp(void)
{
    return (uint32_t)rf->tx_timer.read_us();
}

static void rf_lock(void)
{
    platform_enter_critical();
}

static void rf_unlock(void)
{
    platform_exit_critical();
}

static int8_t rf_device_register(const uint8_t *mac_addr)
{
    rf_init();
    device_driver.PHY_MAC = (uint8_t *)mac_addr;
    device_driver.driver_description = (char *)"ATMEL_MAC";
    device_driver.link_type = PHY_LINK_15_4_2_4GHZ_TYPE;
    device_driver.phy_channel_pages = phy_channel_pages;
    device_driver.phy_MTU = RF_MTU_15_4G_2012;
    device_driver.phy_header_length = 0;
    device_driver.phy_tail_length = 0;
    device_driver.address_write = &rf_address_write;
    device_driver.extension = &rf_extension;
    device_driver.state_control = &rf_interface_state_control;
    device_driver.tx = &rf_start_csma_ca;
    device_driver.phy_rx_cb = NULL;
    device_driver.phy_tx_done_cb = NULL;
    rf_radio_driver_id = arm_net_phy_register(&device_driver);
    rf_update_config = true;
    return rf_radio_driver_id;
}

static int8_t rf_address_write(phy_address_type_e address_type, uint8_t *address_ptr)
{
    uint8_t rf_address = 0, addr_size = 0;
    switch (address_type) {
        case PHY_MAC_48BIT:
            break;
        case PHY_MAC_64BIT:
            rf_address = BBC_MACEA0;
            addr_size = 8;
            break;
        case PHY_MAC_16BIT:
            rf_address = BBC_MACSHA0F0;
            addr_size = 2;
            break;
        case PHY_MAC_PANID:
            rf_address = BBC_MACPID0F0;
            addr_size = 2;
            break;
    }
    for (uint8_t i = 0; i < addr_size; i++) {
        rf_write_bbc_register(rf_address++, rf_module, address_ptr[(addr_size - 1) - i]);
    }
    return 0;
}

static int8_t rf_extension(phy_extension_type_e extension_type, uint8_t *data_ptr)
{
    phy_csma_params_t *csma_params;
    uint32_t *timer_value;
    switch (extension_type) {
        case PHY_EXTENSION_SET_CHANNEL:
            if (rf_state == RF_IDLE || rf_state == RF_CSMA_STARTED) {
                rf_receive(*data_ptr, rf_module);
            } else {
                // Store the new channel if couldn't change it yet.
                rf_new_channel = *data_ptr;
                return -1;
            }
            break;
        case PHY_EXTENSION_READ_RX_TIME:
            common_write_32_bit(rx_time, data_ptr);
            break;
        case PHY_EXTENSION_GET_TIMESTAMP:
            timer_value = (uint32_t *)data_ptr;
            *timer_value = rf_get_timestamp();
            break;
        case PHY_EXTENSION_SET_CSMA_PARAMETERS:
            csma_params = (phy_csma_params_t *)data_ptr;
            if (csma_params->backoff_time == 0) {
                rf->cca_timer.detach();
                if (rf_state == RF_TX_STARTED) {
                    rf_state = RF_IDLE;
                    rf_receive(rf_rx_channel, rf_module);
                }
                tx_time = 0;
            } else {
                tx_time = csma_params->backoff_time;
                cca_enabled = csma_params->cca_enabled;
            }
            break;
        case PHY_EXTENSION_GET_SYMBOLS_PER_SECOND:
            timer_value = (uint32_t *)data_ptr;
            *timer_value = rf_symbol_rate;
            break;
        case PHY_EXTENSION_DYNAMIC_RF_SUPPORTED:
            *data_ptr = true;
            break;
        case PHY_EXTENSION_SET_RF_CONFIGURATION:
            memcpy(&phy_current_config, data_ptr, sizeof(phy_rf_channel_configuration_s));
            rf_calculate_symbol_rate(phy_current_config.datarate, phy_current_config.modulation);
            if (phy_current_config.channel_0_center_frequency < 1000000000) {
                rf_module = RF_09;
            } else {
                rf_module = RF_24;
            }
            rf_update_config = true;
            if (rf_state == RF_IDLE) {
                rf_receive(rf_rx_channel, rf_module);
            }
            break;
        case PHY_EXTENSION_SET_CCA_THRESHOLD:
            rf_conf_set_cca_threshold(*data_ptr);
            break;
        default:
            break;
    }
    return 0;
}

static int8_t rf_interface_state_control(phy_interface_state_e new_state, uint8_t rf_channel)
{
    int8_t ret_val = 0;
    switch (new_state) {
        case PHY_INTERFACE_RESET:
            break;
        case PHY_INTERFACE_DOWN:
            break;
        case PHY_INTERFACE_UP:
            rf_receive(rf_channel, rf_module);
            break;
        case PHY_INTERFACE_RX_ENERGY_STATE:
            break;
        case PHY_INTERFACE_SNIFFER_STATE:
            break;
    }
    return ret_val;
}

#ifdef TEST_GPIOS_ENABLED
static void test1_toggle(void)
{
    if (test_pins->TEST4) {
        test_pins->TEST4 = 0;
    } else {
        test_pins->TEST4 = 1;
    }
}
static void test2_toggle(void)
{
    if (test_pins->TEST5) {
        test_pins->TEST5 = 0;
    } else {
        test_pins->TEST5 = 1;
    }
}
#endif //TEST_GPIOS_ENABLED

static void rf_init(void)
{
#ifdef TEST_GPIOS_ENABLED
    fhss_bc_switch = test1_toggle;
    fhss_uc_switch = test2_toggle;
#endif //TEST_GPIOS_ENABLED
    rf_lock();
    // Disable interrupts
    rf_write_rf_register(RF_IRQM, RF_09, 0);
    rf_write_rf_register(RF_IRQM, RF_24, 0);
    // Ensure basebands enabled, I/Q IF's disabled
    rf_write_rf_register_field(RF_IQIFC1, COMMON, CHPM, RF_MODE_BBRF);
    rf_change_state(RF_TRX_OFF, RF_09);
    rf_change_state(RF_TRX_OFF, RF_24);
    memcpy(&phy_current_config, &phy_24ghz, sizeof(phy_rf_channel_configuration_s));
    rf_calculate_symbol_rate(phy_current_config.datarate, phy_current_config.modulation);
    rf_init_registers(RF_24);
    rf->IRQ.rise(&rf_interrupt_handler);
    rf->IRQ.enable_irq();
    rf->tx_timer.start();
    rf_unlock();
}

static void rf_init_registers(rf_modules_e module)
{
    // O-QPSK configuration using IEEE Std 802.15.4-2011
    // FSK configuration using IEEE Std 802.15.4g-2012
    if (phy_current_config.modulation == M_OQPSK) {
        rf_module = RF_24;
        device_driver.phy_MTU = RF_MTU_15_4_2011;
        device_driver.link_type = PHY_LINK_15_4_2_4GHZ_TYPE;
        // 16-bit FCS
        rf_write_bbc_register_field(BBC_PC, module, FCST, FCST);
        // Enable O-QPSK
        rf_write_bbc_register_field(BBC_PC, module, PT, BB_MROQPSK);
        // Chip frequency 2000kchip/s
        rf_write_bbc_register_field(BBC_OQPSKC0, module, FCHIP, BB_FCHIP2000);
        // FCS type legacy O-QPSK is 16-bit
        rf_write_bbc_register_field(BBC_OQPSKC2, module, FCSTLEG, FCS_16);
        // Listen for both MR-O-QPSK and legacy O-QPSK PHY
        rf_write_bbc_register_field(BBC_OQPSKC2, module, RXM, RXM_2);
        // PHY type Legacy O-QPSK
        rf_write_bbc_register_field(BBC_OQPSKPHRTX, module, LEG, LEG);
        // Low pass filter cut-off frequency to 1000 kHz
        rf_write_rf_register_field(RF_TXCUTC, module, LPFCUT, RF_FLC1000KHZ);
        // Set TX filter to sample frequency / 2
        rf_write_rf_register_field(RF_TXDFE, module, RCUT, RCUT_4);
        // Enable auto ack
        rf_write_bbc_register_field(BBC_AMCS, module, AACK, AACK);
        // Enable address filter unit 0
        rf_write_bbc_register_field(BBC_AFC0, module, AFEN0, AFEN0);
        // Allow Ack frame type with address filter
        rf_write_bbc_register_field(BBC_AFFTM, module, TYPE_2, TYPE_2);
    } else if (phy_current_config.modulation == M_2FSK) {
        rf_module = RF_09;
        device_driver.phy_MTU = RF_MTU_15_4G_2012;
        device_driver.link_type = PHY_LINK_15_4_SUBGHZ_TYPE;
        // Enable FSK
        rf_write_bbc_register_field(BBC_PC, module, PT, BB_MRFSK);
        // Disable auto ack
        rf_write_bbc_register_field(BBC_AMCS, module, AACK, 0);
        // Disable address filter unit 0
        rf_write_bbc_register_field(BBC_AFC0, module, AFEN0, 0);

        rf_write_bbc_register_field(BBC_FSKC0, module, BT, BT_20);

        rf_write_bbc_register_field(BBC_FSKC2, module, FECIE, 0);

        rf_write_bbc_register_field(BBC_FSKC2, module, RXO, RXO_DIS);

        // Set modulation index
        if (phy_current_config.modulation_index == MODULATION_INDEX_0_5) {
            rf_write_bbc_register_field(BBC_FSKC0, module, MIDX, MIDX_05);
            rf_write_rf_register_field(RF_TXDFE, module, RCUT, RCUT_0);
        } else {
            rf_write_bbc_register_field(BBC_FSKC0, module, MIDX, MIDX_10);
            rf_write_rf_register_field(RF_TXDFE, module, RCUT, RCUT_4);
        }
        // Set Gain control settings
        rf_write_rf_register_field(RF_AGCC, module, AVGS, AVGS_8_SAMPLES);
        rf_write_rf_register_field(RF_AGCS, module, TGT, TGT_1);
        // Set symbol rate and related configurations
        rf_set_fsk_symbol_rate_configuration(phy_current_config.datarate, module);
        // Set preamble length
        uint8_t preamble_len = 24;
        if (phy_current_config.datarate < 150000) {
            preamble_len = 8;
        } else if (phy_current_config.datarate < 300000) {
            preamble_len = 12;
        }
        rf_write_bbc_register(BBC_FSKPLL, module, preamble_len);
    }
    // Disable filtering FCS
    rf_write_bbc_register_field(BBC_PC, module, FCSFE, 0);
    // Set channel spacing
    rf_set_channel_spacing(phy_current_config.channel_spacing, module);
    // Set channel 0 center frequency
    rf_set_ch0_frequency(phy_current_config.channel_0_center_frequency, module);
    // Set channel (must be called after frequency change)
    rf_set_channel(rf_rx_channel, module);
}

static void rf_csma_ca_timer_interrupt(void)
{
    rf_irq_rf_enable(EDC, RF_09);
    rf_irq_rf_enable(EDC, rf_module);
    rf_write_rf_register_field(RF_EDC, rf_module, EDM, RF_EDSINGLE);
    rf_backup_timer_start(MAX_PACKET_SENDING_TIME);
}

static void rf_csma_ca_timer_signal(void)
{
    rf->irq_thread_215.flags_set(SIG_TIMER_CCA);
}

static int8_t rf_start_csma_ca(uint8_t *data_ptr, uint16_t data_length, uint8_t tx_handle, data_protocol_e data_protocol)
{
    rf_lock();
    if (rf_state != RF_IDLE) {
        rf_unlock();
        return -1;
    }
    rf_state = RF_CSMA_STARTED;

    // If Ack is requested, store the MAC sequence. This will be compared with received Ack.
    uint8_t version = ((*(data_ptr + 1) & VERSION_FIELD_MASK) >> SHIFT_VERSION_FIELD);
    if ((version != MAC_FRAME_VERSION_2) && (*data_ptr & FC_AR)) {
        tx_sequence = *(data_ptr + 2);
    }
    rf_write_tx_buffer(data_ptr, data_length, rf_module);
    if (phy_current_config.modulation == M_OQPSK) {
        data_length += 2;
    } else if (phy_current_config.modulation == M_2FSK) {
        data_length += 4;
    }
    rf_write_tx_packet_length(data_length, rf_module);
    mac_tx_handle = tx_handle;

    if (tx_time) {
        uint32_t backoff_time = tx_time - rf_get_timestamp();
        // Max. time to TX can be 65ms, otherwise time has passed already -> send immediately
        if (backoff_time <= 65000) {
            rf->cca_timer.attach_us(rf_csma_ca_timer_signal, backoff_time);
            TEST_CSMA_STARTED
            rf_unlock();
            return 0;
        }
    }
    // Short timeout to start CCA immediately.
    rf->cca_timer.attach_us(rf_csma_ca_timer_signal, 1);
    TEST_CSMA_STARTED
    rf_unlock();
    return 0;
}

static void rf_handle_cca_ed_done(void)
{
    TEST_CSMA_DONE
    rf_irq_rf_disable(EDC, RF_09);
    rf_irq_rf_disable(EDC, rf_module);
    if (rf_state == RF_CSMA_WHILE_RX) {
        rf_state = RF_RX_STARTED;
    }

    int8_t status = device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_CCA_PREPARE, 0, 0);
    if (status == PHY_TX_NOT_ALLOWED) {
        if (rf_state == RF_CSMA_STARTED) {
            rf_state = RF_IDLE;
        }
        return;
    }
    if ((cca_enabled == true) && (rf_state == RF_RX_STARTED)) {
        device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_CCA_FAIL, 0, 0);
        return;
    }
    rf_backup_timer_stop();
    if (status == PHY_RESTART_CSMA) {
        device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_CCA_OK, 0, 0);
        if (tx_time) {
            uint32_t backoff_time = tx_time - rf_get_timestamp();
            // Max. time to TX can be 65ms, otherwise time has passed already -> send immediately
            if (backoff_time > 65000) {
                backoff_time = 1;
            }
            rf->cca_timer.attach_us(rf_csma_ca_timer_signal, backoff_time);
            TEST_CSMA_STARTED
        }
        return;
    }
    if ((cca_enabled == true) && (((int8_t) rf_read_rf_register(RF_EDV, rf_module) > cca_threshold))) {
        rf_state = RF_IDLE;
        device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_CCA_FAIL, 0, 0);
        return;
    }
    rf_irq_bbc_disable(RXFE, rf_module);
    rf_start_tx();
}

static void rf_handle_tx_done(void)
{
    rf_backup_timer_stop();
    TEST_TX_DONE
    rf_irq_bbc_disable(TXFE, rf_module);
    rf_state = RF_IDLE;
    rf_receive(rf_rx_channel, rf_module);
    device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_TX_SUCCESS, 0, 0);
}

static void rf_start_tx(void)
{
    rf_change_state(RF_TXPREP, rf_module);
    rf_irq_bbc_enable(TXFE, rf_module);
    rf_change_state(RF_TX, rf_module);
    rf_state = RF_TX_STARTED;
    TEST_TX_STARTED
    rf_backup_timer_start(MAX_PACKET_SENDING_TIME);
}

static void rf_handle_ack(uint8_t seq_number, uint8_t pending)
{
    phy_link_tx_status_e phy_status;
    if (tx_sequence == (uint16_t)seq_number) {
         if (pending) {
            phy_status = PHY_LINK_TX_DONE_PENDING;
        } else {
            phy_status = PHY_LINK_TX_DONE;
        }
        // No CCA attempts done, just waited Ack
        device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, phy_status, 0, 0);
        // Clear TX sequence when Ack is received to avoid duplicate Acks
        tx_sequence = 0xffff;
    }
}

static uint32_t rf_crc_reflect(uint32_t input, uint8_t size)
{
    uint32_t output = 0;
    for (int i = 0; i < size; i++) {
        if (input & (1 << i)) {
            output |= (uint32_t)(1 << ((size - 1) - i));
        }
    }
    return output;
}

static uint32_t rf_crc32(uint8_t *data, uint16_t length, bool ref_input, bool ref_output, bool xor_output)
{
    uint32_t crc = 0xffffffff;
    while(length--) {
        uint8_t byte = *data++;
        if (ref_input) {
            byte = rf_crc_reflect(byte, 8);
        }
        crc = (crc << 8) ^ crc32_lookup_table[(crc >> 24) ^ byte];
    }
    if (xor_output) {
        crc ^= 0xffffffff;
    }
    if (ref_output) {
        crc = rf_crc_reflect(crc, 32);
    }
    return crc;
}

static uint16_t rf_crc16(uint8_t *data, uint8_t length)
{
    uint16_t crc = 0;
    while (length--) {
        crc ^= *data++;
        for (uint8_t k = 0; k < 8; k++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0x8408;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

static bool rf_validate_fcs(uint16_t packet_length)
{
    if (phy_current_config.modulation == M_OQPSK) {
        uint16_t crc16_computed = rf_crc16(rx_buffer, packet_length - 2);
        uint16_t crc16_received = common_read_16_bit_inverse(&rx_buffer[packet_length-2]);
        if (crc16_computed == crc16_received) {
            return true;
        }
    } else if (phy_current_config.modulation == M_2FSK) {
        uint32_t crc32_computed = rf_crc32(rx_buffer, packet_length - 4, true , true, true);
        uint32_t crc32_received = common_read_32_bit_inverse(&rx_buffer[packet_length-4]);
        if (crc32_computed == crc32_received) {
            return true;
        }
    }
    return false;
}

static void rf_handle_rx_done(void)
{
    TEST_RX_DONE
    rf_backup_timer_stop();
    if (rf_state == RF_CSMA_WHILE_RX) {
        rf_state = RF_CSMA_STARTED;
        rf_backup_timer_start(MAX_PACKET_SENDING_TIME);
    } else if (rf_state == RF_RX_STARTED) {
        rf_state = RF_IDLE;
    }
    uint16_t rx_data_length = rf_read_rx_frame_length(rf_module);
    if (!rf_read_rx_buffer(rx_data_length, rf_module)) {
        if (rf_validate_fcs(rx_data_length)) {
            uint8_t version = ((rx_buffer[1] & VERSION_FIELD_MASK) >> SHIFT_VERSION_FIELD);
            if (((rx_buffer[0] & MAC_FRAME_TYPE_MASK) == MAC_TYPE_ACK) && (version < MAC_FRAME_VERSION_2)) {
                rf_handle_ack(rx_buffer[2], rx_buffer[0] & MAC_DATA_PENDING);
            } else {
                int8_t rssi = (int8_t) rf_read_rf_register(RF_EDV, rf_module);
                if (phy_current_config.modulation == M_OQPSK) {
                    rx_data_length -= 2;
                } else if (phy_current_config.modulation == M_2FSK) {
                    rx_data_length -= 4;
                }
                device_driver.phy_rx_cb(rx_buffer, rx_data_length, 0xf0, rssi, rf_radio_driver_id);
                // If auto ack used, must wait until RF returns to RF_TXPREP state
                if ((version != MAC_FRAME_VERSION_2) && (rx_buffer[0] & FC_AR)) {
                    wait_us(50);
                    rf_poll_state_change(RF_TXPREP, rf_module);
                }
            }
        } else {
            if (device_driver.phy_rf_statistics) {
                device_driver.phy_rf_statistics->crc_fails++;
            }
        }
    }
    rf_receive(rf_new_channel, rf_module);
}

static void rf_handle_rx_start(void)
{
    rx_time = rf_get_timestamp();
    if (rf_state == RF_CSMA_STARTED) {
        rf_backup_timer_stop();
        rf_state = RF_CSMA_WHILE_RX;
    } else {
        rf_state = RF_RX_STARTED;
    }
    TEST_RX_STARTED
    rf_backup_timer_start(MAX_PACKET_SENDING_TIME);
}

static void rf_receive(uint16_t rx_channel, rf_modules_e module)
{
    TEST_RX_DONE
    rf_lock();
    if (rf_update_config == true) {
        rf_update_config = false;
        rf_change_state(RF_TRX_OFF, module);
        rf_init_registers(module);
        rf_change_state(RF_TXPREP, module);
    }
    if (rx_channel != rf_rx_channel) {
        rf_change_state(RF_TXPREP, module);
        rf_set_channel(rx_channel, module);
        rf_rx_channel = rf_new_channel = rx_channel;
    }
    rf_change_state(RF_RX, module);
    rf_irq_bbc_enable(RXFS, module);
    rf_irq_bbc_enable(RXFE, module);
    rf_unlock();
}

static void rf_interrupt_handler(void)
{
    rf->irq_thread_215.flags_set(SIG_RADIO);
}

static void rf_irq_task_process_irq(void)
{
    uint8_t irq_rf09_status = 0, irq_bbc0_status = 0, irq_rf24_status = 0, irq_bbc1_status = 0;
    if (rf09_irq_mask) {
        irq_rf09_status = rf_read_common_register(RF09_IRQS);
        irq_rf09_status &= rf09_irq_mask;
    }
    if (bbc0_irq_mask) {
        irq_bbc0_status = rf_read_common_register(BBC0_IRQS);
        irq_bbc0_status &= bbc0_irq_mask;
    }
    if (rf24_irq_mask) {
        irq_rf24_status = rf_read_common_register(RF24_IRQS);
        irq_rf24_status &= rf24_irq_mask;
    }
    if (bbc1_irq_mask) {
        irq_bbc1_status = rf_read_common_register(BBC1_IRQS);
        irq_bbc1_status &= bbc1_irq_mask;
    }
    if ((rf_state == RF_CSMA_STARTED) || (rf_state == RF_CSMA_WHILE_RX)) {
        if ((irq_rf09_status & EDC) || (irq_rf24_status & EDC)) {
            rf_handle_cca_ed_done();
        }
    }
    if (rf_state == RF_TX_STARTED) {
        if ((irq_bbc0_status & TXFE) || (irq_bbc1_status & TXFE)) {
            rf_handle_tx_done();
        }
    }
    if ((rf_state == RF_IDLE) || (rf_state == RF_CSMA_STARTED)) {
        if ((irq_bbc0_status & RXFS) || (irq_bbc1_status & RXFS)) {
            rf_handle_rx_start();
        }
    }
    if ((rf_state == RF_RX_STARTED) || (rf_state == RF_CSMA_WHILE_RX)) {
        if ((irq_bbc0_status & RXFE) || (irq_bbc1_status & RXFE)) {
            rf_handle_rx_done();
        }
    }
}

static void rf_write_tx_packet_length(uint16_t packet_length, rf_modules_e module)
{
    if (packet_length > device_driver.phy_MTU) {
        return;
    }
    rf_write_bbc_register(BBC_TXFLH, module, (uint8_t) (packet_length >> 8));
    rf_write_bbc_register(BBC_TXFLL, module, (uint8_t) packet_length);
}

static uint16_t rf_read_rx_frame_length(rf_modules_e module)
{
    const uint8_t tx[2] = { static_cast<uint8_t>(module + 2), static_cast<uint8_t>(BBC_RXFLL) };
    uint8_t rx[2];
    rf->CS = 0;
    rf_spi_exchange(tx, 2, NULL, 0);
    rf_spi_exchange(NULL, 0, rx, 2);
    rf->CS = 1;
    return (uint16_t)((rx[1] << 8) | rx[0]);
}

static void rf_write_tx_buffer(uint8_t *data, uint16_t len, rf_modules_e module)
{
    uint16_t buffer_addr = BBC0_FBTXS + (0x1000 * (module - 1));
    const uint8_t tx[2] = { static_cast<uint8_t>(0x80 | (buffer_addr >> 8)), static_cast<uint8_t>(buffer_addr) };
    rf->CS = 0;
    rf_spi_exchange(tx, 2, NULL, 0);
    rf_spi_exchange(data, len, NULL, 0);
    rf->CS = 1;
}

static int rf_read_rx_buffer(uint16_t length, rf_modules_e module)
{
    if (length > device_driver.phy_MTU) {
        return -1;
    }
    uint8_t *ptr = rx_buffer;
    uint16_t buffer_addr = BBC0_FBRXS + (0x1000 * (module - 1));
    const uint8_t tx[2] = { static_cast<uint8_t>(buffer_addr >> 8), static_cast<uint8_t>(buffer_addr) };
    rf->CS = 0;
    rf_spi_exchange(tx, 2, NULL, 0);
    rf_spi_exchange(NULL, 0, ptr, length);
    rf->CS = 1;
    return 0;
}

static void rf_irq_rf_enable(uint8_t irq, rf_modules_e module)
{
    if ((module == RF_09) && !(rf09_irq_mask & irq)) {
        rf_write_rf_register_field(RF_IRQM, module, irq, irq);
        rf09_irq_mask |= irq;
    } else if ((module == RF_24) && !(rf24_irq_mask & irq)) {
        rf_write_rf_register_field(RF_IRQM, module, irq, irq);
        rf24_irq_mask |= irq;
    }
}

static void rf_irq_rf_disable(uint8_t irq, rf_modules_e module)
{
    if ((module == RF_09) && (rf09_irq_mask & irq)) {
        rf_write_rf_register_field(RF_IRQM, module, irq, 0);
        rf09_irq_mask &= ~irq;
    } else if ((module == RF_24) && (rf24_irq_mask & irq)) {
        rf_write_rf_register_field(RF_IRQM, module, irq, 0);
        rf24_irq_mask &= ~irq;
    }
}

static void rf_irq_bbc_enable(uint8_t irq, rf_modules_e module)
{
    if ((module == RF_09) && !(bbc0_irq_mask & irq)) {
        rf_write_bbc_register_field(BBC_IRQM, module, irq, irq);
        bbc0_irq_mask |= irq;
    } else if ((module == RF_24) && !(bbc1_irq_mask & irq)) {
        rf_write_bbc_register_field(BBC_IRQM, module, irq, irq);
        bbc1_irq_mask |= irq;
    }
}

static void rf_irq_bbc_disable(uint8_t irq, rf_modules_e module)
{
    if ((module == RF_09) && (bbc0_irq_mask & irq)) {
        rf_write_bbc_register_field(BBC_IRQM, module, irq, 0);
        bbc0_irq_mask &= ~irq;
    } else if ((module == RF_24) && (bbc1_irq_mask & irq)) {
        rf_write_bbc_register_field(BBC_IRQM, module, irq, 0);
        bbc1_irq_mask &= ~irq;
    }
}

static rf_command_e rf_read_state(rf_modules_e module)
{
    return (rf_command_e) rf_read_rf_register(RF_STATE, module);
}

static void rf_poll_state_change(rf_command_e state, rf_modules_e module)
{
    uint32_t transition_start_time = rf_get_timestamp();
    while (rf_read_state(module) != state) {
        if (rf_get_timestamp() > (transition_start_time + MAX_STATE_TRANSITION_TIME_US)) {
            tr_err("Failed to change module %u state from %x to: %x", module, rf_read_state(module), state);
            break;
        }
    }
}

static void rf_change_state(rf_command_e state, rf_modules_e module)
{
    rf_write_rf_register(RF_CMD, module, state);
    return rf_poll_state_change(state, module);
}

static void rf_spi_exchange(const void *tx, size_t tx_len, void *rx, size_t rx_len)
{
    rf->spi.write(static_cast<const char *>(tx), tx_len, static_cast<char *>(rx), rx_len);
}

static uint8_t rf_read_rf_register(uint8_t addr, rf_modules_e module)
{
    const uint8_t tx[2] = { static_cast<uint8_t>(module), static_cast<uint8_t>(addr) };
    uint8_t rx[3];
    rf->CS = 0;
    rf_spi_exchange(tx, 2, rx, 3);
    rf->CS = 1;
    return rx[2];
}

static void rf_write_rf_register(uint8_t addr, rf_modules_e module, uint8_t data)
{
    const uint8_t tx[3] = { static_cast<uint8_t>(0x80 | module), static_cast<uint8_t>(addr), static_cast<uint8_t>(data) };
    uint8_t rx[2];
    rf->CS = 0;
    rf_spi_exchange(tx, 3, rx, 2);
    rf->CS = 1;
}

static void rf_write_rf_register_field(uint8_t addr, rf_modules_e module, uint8_t field, uint8_t value)
{
    uint8_t reg_tmp = rf_read_rf_register(addr, module);
    reg_tmp &= ~field;
    reg_tmp |= value;
    rf_write_rf_register(addr, module, reg_tmp);
}

static void rf_backup_timer_interrupt(void)
{
    rf_read_common_register(RF09_IRQS);
    rf_read_common_register(RF24_IRQS);
    rf_read_common_register(BBC0_IRQS);
    rf_read_common_register(BBC1_IRQS);
    rf_irq_rf_disable(EDC, RF_09);
    rf_irq_rf_disable(EDC, RF_24);
    if (rf_state == RF_RX_STARTED) {
        if (device_driver.phy_rf_statistics) {
            device_driver.phy_rf_statistics->rx_timeouts++;
        }
    } else {
        if (device_driver.phy_rf_statistics) {
            device_driver.phy_rf_statistics->tx_timeouts++;
        }
    }
    if (rf_state == RF_TX_STARTED) {
        device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_TX_SUCCESS, 0, 0);
    }
    if ((rf_state == RF_CSMA_STARTED) || (rf_state == RF_CSMA_WHILE_RX)) {
        device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_CCA_FAIL, 0, 0);
    }
    TEST_TX_DONE
    TEST_RX_DONE
    rf_state = RF_IDLE;
    rf_receive(rf_rx_channel, rf_module);
}

static void rf_backup_timer_signal(void)
{
    rf->irq_thread_215.flags_set(SIG_TIMER_BACKUP);
}

static void rf_backup_timer_stop(void)
{
    rf->cal_timer.detach();
}

static void rf_backup_timer_start(uint32_t slots)
{
    // Using cal_timer as backup timer
    rf->cal_timer.attach_us(rf_backup_timer_signal, slots);
}

static int rf_set_channel(uint16_t channel, rf_modules_e module)
{
    rf_write_rf_register(RF_CNL, module, (uint8_t) channel);
    rf_write_rf_register_field(RF_CNM, module, CNH, (uint8_t) (channel >> 8));
    return 0;
}

static int rf_set_ch0_frequency(uint32_t frequency, rf_modules_e module)
{
    if (module == RF_24) {
        frequency -= 1500000000;
    }
    frequency /= 25000;
    rf_write_rf_register(RF_CCF0L, module, (uint8_t)frequency);
    rf_write_rf_register(RF_CCF0H, module, (uint8_t)(frequency >> 8));
    return 0;
}

static int rf_set_channel_spacing(uint32_t channel_spacing, rf_modules_e module)
{
    channel_spacing /= 25000;
    rf_write_rf_register(RF_CS, module, channel_spacing);
    return 0;
}

static int rf_set_fsk_symbol_rate_configuration(uint32_t symbol_rate, rf_modules_e module)
{
    if (symbol_rate == 50000) {
        rf_write_bbc_register_field(BBC_FSKC1, module, SRATE, SRATE_50KHZ);
        if (rf_version_num == 1) {
            rf_write_rf_register_field(RF_TXDFE, module, SR, SR_10);
        } else {
            rf_write_rf_register_field(RF_TXDFE, module, SR, SR_8);
        }
        rf_write_rf_register_field(RF_RXDFE, module, SR, SR_10);
        if (phy_current_config.modulation_index == MODULATION_INDEX_0_5) {
            rf_write_rf_register_field(RF_RXDFE, module, RCUT, RCUT_0);
        } else {
            rf_write_rf_register_field(RF_RXDFE, module, RCUT, RCUT_1);
        }
        rf_write_rf_register_field(RF_TXCUTC, module, PARAMP, RF_PARAMP32U);
        rf_write_rf_register_field(RF_TXCUTC, module, LPFCUT, RF_FLC80KHZ);
        rf_write_rf_register_field(RF_RXBWC, module, BW, RF_BW160KHZ_IF250KHZ);
        rf_write_rf_register_field(RF_RXBWC, module, IFS, 0);
    } else if (symbol_rate == 100000) {
        rf_write_bbc_register_field(BBC_FSKC1, module, SRATE, SRATE_100KHZ);
        if (rf_version_num == 1) {
            rf_write_rf_register_field(RF_TXDFE, module, SR, SR_5);
        } else {
            rf_write_rf_register_field(RF_TXDFE, module, SR, SR_4);
        }
        rf_write_rf_register_field(RF_RXDFE, module, SR, SR_5);
        rf_write_rf_register_field(RF_TXCUTC, module, PARAMP, RF_PARAMP16U);
        if (phy_current_config.modulation_index == MODULATION_INDEX_0_5) {
            rf_write_rf_register_field(RF_RXDFE, module, RCUT, RCUT_0);
            rf_write_rf_register_field(RF_TXCUTC, module, LPFCUT, RF_FLC100KHZ);
            rf_write_rf_register_field(RF_RXBWC, module, BW, RF_BW200KHZ_IF250KHZ);
        } else {
            rf_write_rf_register_field(RF_RXDFE, module, RCUT, RCUT_1);
            rf_write_rf_register_field(RF_TXCUTC, module, LPFCUT, RF_FLC160KHZ);
            rf_write_rf_register_field(RF_RXBWC, module, BW, RF_BW320KHZ_IF500KHZ);
        }
        rf_write_rf_register_field(RF_RXBWC, module, IFS, 0);
    } else if (symbol_rate == 150000) {
        rf_write_bbc_register_field(BBC_FSKC1, module, SRATE, SRATE_150KHZ);
        rf_write_rf_register_field(RF_TXDFE, module, SR, SR_2);
        rf_write_rf_register_field(RF_RXDFE, module, SR, SR_4);
        rf_write_rf_register_field(RF_TXCUTC, module, PARAMP, RF_PARAMP16U);
        if (phy_current_config.modulation_index == MODULATION_INDEX_0_5) {
            rf_write_rf_register_field(RF_RXDFE, module, RCUT, RCUT_0);
            rf_write_rf_register_field(RF_TXCUTC, module, LPFCUT, RF_FLC160KHZ);
            rf_write_rf_register_field(RF_RXBWC, module, BW, RF_BW320KHZ_IF500KHZ);
        } else {
            rf_write_rf_register_field(RF_RXDFE, module, RCUT, RCUT_1);
            rf_write_rf_register_field(RF_TXCUTC, module, LPFCUT, RF_FLC250KHZ);
            rf_write_rf_register_field(RF_RXBWC, module, BW, RF_BW400KHZ_IF500KHZ);
        }
        rf_write_rf_register_field(RF_RXBWC, module, IFS, 0);
    } else if (symbol_rate == 200000) {
        rf_write_bbc_register_field(BBC_FSKC1, module, SRATE, SRATE_200KHZ);
        rf_write_rf_register_field(RF_TXDFE, module, SR, SR_2);
        rf_write_rf_register_field(RF_RXDFE, module, SR, SR_4);
        rf_write_rf_register_field(RF_TXCUTC, module, PARAMP, RF_PARAMP16U);
        if (phy_current_config.modulation_index == MODULATION_INDEX_0_5) {
            rf_write_rf_register_field(RF_RXDFE, module, RCUT, RCUT_1);
            rf_write_rf_register_field(RF_TXCUTC, module, LPFCUT, RF_FLC200KHZ);
            rf_write_rf_register_field(RF_RXBWC, module, BW, RF_BW320KHZ_IF500KHZ);
            rf_write_rf_register_field(RF_RXBWC, module, IFS, 0);
        } else {
            rf_write_rf_register_field(RF_RXDFE, module, RCUT, RCUT_2);
            rf_write_rf_register_field(RF_TXCUTC, module, LPFCUT, RF_FLC315KHZ);
            rf_write_rf_register_field(RF_RXBWC, module, BW, RF_BW500KHZ_IF500KHZ);
            rf_write_rf_register_field(RF_RXBWC, module, IFS, IFS);
        }
    } else if (symbol_rate == 300000) {
        rf_write_bbc_register_field(BBC_FSKC1, module, SRATE, SRATE_300KHZ);
        rf_write_rf_register_field(RF_TXDFE, module, SR, SR_1);
        rf_write_rf_register_field(RF_RXDFE, module, SR, SR_2);
        rf_write_rf_register_field(RF_TXCUTC, module, PARAMP, RF_PARAMP8U);
        if (phy_current_config.modulation_index == MODULATION_INDEX_0_5) {
            rf_write_rf_register_field(RF_RXDFE, module, RCUT, RCUT_0);
            rf_write_rf_register_field(RF_TXCUTC, module, LPFCUT, RF_FLC315KHZ);
            rf_write_rf_register_field(RF_RXBWC, module, BW, RF_BW500KHZ_IF500KHZ);
            rf_write_rf_register_field(RF_RXBWC, module, IFS, IFS);
        } else {
            rf_write_rf_register_field(RF_RXDFE, module, RCUT, RCUT_1);
            rf_write_rf_register_field(RF_TXCUTC, module, LPFCUT, RF_FLC500KHZ);
            rf_write_rf_register_field(RF_RXBWC, module, BW, RF_BW630KHZ_IF1000KHZ);
            rf_write_rf_register_field(RF_RXBWC, module, IFS, 0);
        }
    } else if (symbol_rate == 400000) {
        rf_write_bbc_register_field(BBC_FSKC1, module, SRATE, SRATE_400KHZ);
        rf_write_rf_register_field(RF_TXDFE, module, SR, SR_1);
        rf_write_rf_register_field(RF_RXDFE, module, SR, SR_2);
        rf_write_rf_register_field(RF_TXCUTC, module, PARAMP, RF_PARAMP8U);
        if (phy_current_config.modulation_index == MODULATION_INDEX_0_5) {
            rf_write_rf_register_field(RF_RXDFE, module, RCUT, RCUT_0);
            rf_write_rf_register_field(RF_TXCUTC, module, LPFCUT, RF_FLC400KHZ);
            rf_write_rf_register_field(RF_RXBWC, module, BW, RF_BW630KHZ_IF1000KHZ);
            rf_write_rf_register_field(RF_RXBWC, module, IFS, 0);
        } else {
            rf_write_rf_register_field(RF_RXDFE, module, RCUT, RCUT_1);
            rf_write_rf_register_field(RF_TXCUTC, module, LPFCUT, RF_FLC625KHZ);
            rf_write_rf_register_field(RF_RXBWC, module, BW, RF_BW1000KHZ_IF1000KHZ);
            rf_write_rf_register_field(RF_RXBWC, module, IFS, IFS);
        }
    }
    return 0;
}

static void rf_conf_set_cca_threshold(uint8_t percent)
{
    uint8_t step = (MAX_CCA_THRESHOLD-MIN_CCA_THRESHOLD);
    cca_threshold = MIN_CCA_THRESHOLD + (step * percent) / 100;
}

static void rf_calculate_symbol_rate(uint32_t baudrate, phy_modulation_e modulation)
{
    uint8_t bits_in_symbols = 4;
    if (modulation == M_2FSK) {
        bits_in_symbols = 1;
    }
    rf_symbol_rate = baudrate / bits_in_symbols;
}

void RFBits::rf_irq_task(void)
{
    for (;;) {
        uint32_t flags = ThisThread::flags_wait_any(SIG_ALL);
        rf_lock();
        if (flags & SIG_RADIO) {
            rf_irq_task_process_irq();
        }
        if (flags & SIG_TIMER_CCA) {
            rf_csma_ca_timer_interrupt();
        }
        if (flags & SIG_TIMER_BACKUP) {
            rf_backup_timer_interrupt();
        }
        rf_unlock();
    }
}

int RFBits::init_215_driver(RFBits *_rf, TestPins *_test_pins, const uint8_t mac[8], uint8_t *rf_part_num)
{
    rf = _rf;
    test_pins = _test_pins;
    irq_thread_215.start(mbed::callback(this, &RFBits::rf_irq_task));
    *rf_part_num = rf_read_common_register(RF_PN);
    rf_version_num = rf_read_common_register(RF_VN);
    tr_info("RF version number: %x", rf_version_num);
    return rf_device_register(mac);
}

#endif // MBED_CONF_NANOSTACK_CONFIGURATION && DEVICE_SPI && DEVICE_INTERRUPTIN && defined(MBED_CONF_RTOS_PRESENT)
