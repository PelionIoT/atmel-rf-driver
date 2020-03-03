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
#include "NanostackRfPhyAT86RF215.h"
#include "NanostackRfPhyAtmel.h"
#include "mbed_trace.h"
#include <Timer.h>
#include "Timeout.h"
#include "SPI.h"

#define TRACE_GROUP "AtRF"

#define RF_MTU              127

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
static void rf_init_registers(void);
static void rf_spi_exchange(const void *tx, size_t tx_len, void *rx, size_t rx_len);
static uint8_t rf_read_rf_register(uint8_t addr, rf_modules_e module);
static void rf_write_rf_register(uint8_t addr, rf_modules_e module, uint8_t data);
static void rf_write_rf_register_field(uint8_t addr, rf_modules_e module, uint8_t field, uint8_t value);
static void rf_write_tx_packet_length(uint16_t packet_length);
static uint16_t rf_read_rx_frame_length(rf_modules_e module);
static uint16_t rf_read_frame_buffer_level(rf_modules_e module);
static void rf_write_tx_buffer(uint8_t *data, uint16_t len, rf_modules_e module);
static int rf_read_rx_buffer(uint16_t length, rf_modules_e module);
static void rf_irq_rf_enable(uint8_t irq, rf_modules_e module);
static void rf_irq_rf_disable(uint8_t irq, rf_modules_e module);
static void rf_irq_bbc_enable(uint8_t irq, rf_modules_e module);
static void rf_irq_bbc_disable(uint8_t irq, rf_modules_e module);
static rf_command_e rf_read_state(rf_modules_e module);
static void rf_poll_state_change(rf_command_e state, rf_modules_e module);
static void rf_change_state(rf_command_e state);
static void rf_receive(uint16_t rx_channel);
static void rf_interrupt_handler(void);
static void rf_irq_task_process_irq(void);
static void rf_handle_cca_ed_done(void);
static void rf_start_tx(void);
static void rf_backup_timer_interrupt(void);
static void rf_backup_timer_stop(void);
static void rf_backup_timer_start(uint32_t slots);
// Defined register read functions
#define rf_read_bbc_register(x, y) rf_read_rf_register(x, (rf_modules_e)(y + 2))
#define rf_read_common_register(x) rf_read_rf_register(x, COMMON)
#define rf_write_bbc_register(x, y, z) rf_write_rf_register(x, (rf_modules_e)(y + 2), z)
#define rf_write_common_register(x, z) rf_write_rf_register(x, COMMON, z)
#define rf_write_bbc_register_field(v, x, y, z) rf_write_rf_register_field(v, (rf_modules_e)(x + 2), y, z)
#define rf_write_common_register_field(v, y, z) rf_write_rf_register_field(v, COMMON, y, z)

static int8_t rf_radio_driver_id = -1;
static phy_device_driver_s device_driver;
static rf_modules_e rf_module = RF_24;
static uint8_t mac_tx_handle = 0;
static rf_states_e rf_state = RF_IDLE;
static uint8_t rx_buffer[RF_MTU];
static uint8_t rf_rx_channel;
static uint16_t tx_sequence = 0xffff;
static uint8_t rf09_irq_mask = 0;
static uint8_t rf24_irq_mask = 0;
static uint8_t bbc0_irq_mask = 0;
static uint8_t bbc1_irq_mask = 0;

static int8_t cca_threshold = -80;

/* Channel configurations for 2.4 and sub-GHz */
static const phy_rf_channel_configuration_s phy_24ghz = {.channel_0_center_frequency = 2405000000U,
                                                         .channel_spacing = 5000000U,
                                                         .datarate = 250000U,
                                                         .number_of_channels = 16U,
                                                         .modulation = M_OQPSK};
static const phy_rf_channel_configuration_s phy_subghz = {.channel_0_center_frequency = 868300000U,
                                                          .channel_spacing = 2000000U,
                                                          .datarate = 250000U,
                                                          .number_of_channels = 11U,
                                                          .modulation = M_OQPSK};

static const phy_device_channel_page_s phy_channel_pages[] = {
    { CHANNEL_PAGE_0, &phy_24ghz},
    { CHANNEL_PAGE_2, &phy_subghz},
    { CHANNEL_PAGE_0, NULL}
};

using namespace mbed;
using namespace rtos;

#include "rfbits.h"
static RFBits *rf;

#define MAC_FRAME_TYPE_MASK     0x07
#define MAC_FRAME_BEACON        (0)
#define MAC_TYPE_DATA           (1)
#define MAC_TYPE_ACK            (2)
#define MAC_TYPE_COMMAND        (3)
#define MAC_DATA_PENDING        0x10
#define FC_DST_MODE             0x0C
#define FC_SRC_MODE             0xC0
#define FC_DST_ADDR_NONE        0x00
#define FC_DST_16_BITS          0x08
#define FC_DST_64_BITS          0x0C
#define FC_SRC_64_BITS          0xC0
#define FC_SEQUENCE_COMPRESSION 0x01
#define FC_AR                   0x20
#define FC_PAN_ID_COMPRESSION   0x40
#define VERSION_FIELD_MASK      0x30
#define SHIFT_SEQ_COMP_FIELD    (0)
#define SHIFT_VERSION_FIELD     (4)
#define SHIFT_PANID_COMP_FIELD  (6)
#define OFFSET_DST_PAN_ID       (3)
#define OFFSET_DST_ADDR         (5)

#define SIG_RADIO           1
#define SIG_TIMER_ACK       2
#define SIG_TIMER_BACKUP    4
#define SIG_TIMER_CCA       8
#define SIG_TIMERS (SIG_TIMER_ACK|SIG_TIMER_BACKUP|SIG_TIMER_CCA)
#define SIG_ALL (SIG_RADIO|SIG_TIMERS)

#define ACK_FRAME_LENGTH    3
// Give some additional time for processing, PHY headers, CRC etc.
#define PACKET_SENDING_EXTRA_TIME   5000
#define MAX_PACKET_SENDING_TIME (uint32_t)(8000000/phy_subghz.datarate)*RF_MTU + PACKET_SENDING_EXTRA_TIME
#define ACK_SENDING_TIME (uint32_t)(8000000/phy_subghz.datarate)*ACK_FRAME_LENGTH + PACKET_SENDING_EXTRA_TIME

#define MAX_STATE_TRANSITION_TIME_US    1000

// t1 = 180ns, SEL falling edge to MISO active [SPI setup assumed slow enough to not need manual delay]
#define CS_SELECT()  {rf->CS = 0; /* delay_ns(180); */}
// t9 = 250ns, last clock to SEL rising edge, t8 = 250ns, SPI idle time between consecutive access
#define CS_RELEASE() {wait_ns(250); rf->CS = 1; wait_ns(250);}


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
    if (rf_module == RF_09) {
        device_driver.link_type = PHY_LINK_15_4_SUBGHZ_TYPE;
    } else {
        device_driver.link_type = PHY_LINK_15_4_2_4GHZ_TYPE;
    }
    device_driver.phy_channel_pages = phy_channel_pages;
    device_driver.phy_MTU = RF_MTU;
    device_driver.phy_header_length = 0;
    device_driver.phy_tail_length = 0;
    device_driver.address_write = &rf_address_write;
    device_driver.extension = &rf_extension;
    device_driver.state_control = &rf_interface_state_control;
    device_driver.tx = &rf_start_csma_ca;
    device_driver.phy_rx_cb = NULL;
    device_driver.phy_tx_done_cb = NULL;
    rf_radio_driver_id = arm_net_phy_register(&device_driver);
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
    switch (extension_type) {
        case PHY_EXTENSION_CTRL_PENDING_BIT:
            break;
        case PHY_EXTENSION_READ_LAST_ACK_PENDING_STATUS:
            break;
        case PHY_EXTENSION_SET_CHANNEL:
            break;
        case PHY_EXTENSION_READ_CHANNEL_ENERGY:
            break;
        case PHY_EXTENSION_READ_LINK_STATUS:
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
            rf_receive(rf_channel);
            break;
        case PHY_INTERFACE_RX_ENERGY_STATE:
            break;
        case PHY_INTERFACE_SNIFFER_STATE:
            break;
    }
    return ret_val;
}

#ifdef TEST_GPIOS_ENABLED
void test1_toggle(void)
{
    if (rf->TEST4) {
        rf->TEST4 = 0;
    } else {
        rf->TEST4 = 1;
    }
}
void test2_toggle(void)
{
    if (rf->TEST5) {
        rf->TEST5 = 0;
    } else {
        rf->TEST5 = 1;
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
    rf_change_state(RF_TRX_OFF);
    rf_init_registers();
    rf->IRQ.rise(&rf_interrupt_handler);
    rf->IRQ.enable_irq();
    rf_unlock();
}

static void rf_init_registers(void)
{
    rf->tx_timer.start();
    tr_debug("RF_IRQM: %x", rf_read_rf_register(RF_IRQM, rf_module));
    rf_write_rf_register(RF_IRQM, RF_09, 0);
    rf_write_rf_register(RF_IRQM, RF_24, 0);

    rf_write_bbc_register(BBC_AMEDT, rf_module, -60);
    tr_debug("BBC_AMEDT: %x", rf_read_bbc_register(BBC_AMEDT, rf_module));
    rf_write_bbc_register_field(BBC_PC, rf_module, PT, BB_MROQPSK);
    rf_write_bbc_register_field(BBC_PC, rf_module, FCST, FCST);
    rf_write_bbc_register_field(BBC_PC, rf_module, FCSFE, 0);
    tr_debug("BBC_PC: %x", rf_read_bbc_register(BBC_PC, rf_module));
    // Ensure basebands enabled, I/Q IF's disabled
    rf_write_rf_register_field(RF_IQIFC1, COMMON, CHPM, RF_MODE_BBRF);

    tr_debug("RF_IQIFC1: %x", rf_read_common_register(RF_IQIFC1));

    rf_write_bbc_register_field(BBC_OQPSKC0, rf_module, FCHIP, BB_FCHIP2000);
    rf_write_bbc_register_field(BBC_OQPSKC2, rf_module, FCSTLEG, FCS_16);
    rf_write_bbc_register_field(BBC_OQPSKC2, rf_module, RXM, RXM_2);
    rf_write_bbc_register_field(BBC_OQPSKPHRTX, rf_module, LEG, LEG);

    tr_debug("BBC_OQPSKC0: %x", rf_read_bbc_register(BBC_OQPSKC0, rf_module));
    tr_debug("BBC_OQPSKC1: %x", rf_read_bbc_register(BBC_OQPSKC1, rf_module));
    tr_debug("BBC_OQPSKC2: %x", rf_read_bbc_register(BBC_OQPSKC2, rf_module));
    tr_debug("BBC_OQPSKC3: %x", rf_read_bbc_register(BBC_OQPSKC3, rf_module));
    tr_debug("BBC_OQPSKPHRTX: %x", rf_read_bbc_register(BBC_OQPSKPHRTX, rf_module));
    tr_debug("BBC_OQPSKPHRRX: %x", rf_read_bbc_register(BBC_OQPSKPHRRX, rf_module));


    rf_write_rf_register_field(RF_TXCUTC, rf_module, LPFCUT, RF_FLC1000KHZ);
    rf_write_rf_register_field(RF_TXDFE, rf_module, RCUT, RCUT_4);

    tr_debug("RF_TXCUTC: %x", rf_read_rf_register(RF_TXCUTC, rf_module));
    tr_debug("RF_TXDFE: %x", rf_read_rf_register(RF_TXDFE, rf_module));

    rf_write_rf_register(RF_CS, rf_module, 200);

    tr_debug("RF_CS: %x", rf_read_rf_register(RF_CS, rf_module));


    rf_write_rf_register(RF_CCF0L, rf_module, 0xd0);
    rf_write_rf_register(RF_CCF0H, rf_module, 0x84);
    tr_debug("RF_CCF0L: %x", rf_read_rf_register(RF_CCF0L, rf_module));
    tr_debug("RF_CCF0H: %x", rf_read_rf_register(RF_CCF0H, rf_module));

    tr_debug("RF_RXBWC: %x", rf_read_rf_register(RF_RXBWC, rf_module));

    rf_write_bbc_register_field(BBC_AMCS, rf_module, AACK, AACK);
    tr_debug("BBC_AMCS: %x", rf_read_bbc_register(BBC_AMCS, rf_module));

    rf_write_bbc_register_field(BBC_AFC0, rf_module, AFEN0, AFEN0);

    tr_debug("BBC_AFC0: %x", rf_read_bbc_register(BBC_AFC0, rf_module));

//    rf_write_bbc_register(BBC_AMAACKTL, rf_module, 0);
//    rf_write_bbc_register(BBC_AMAACKTH, rf_module, 3);
    tr_debug("BBC_AMAACKT: %x %x", rf_read_bbc_register(BBC_AMAACKTH, rf_module), rf_read_bbc_register(BBC_AMAACKTL, rf_module));

    rf_write_bbc_register_field(BBC_AFFTM, rf_module, TYPE_2, TYPE_2);

    tr_debug("BBC_AFFTM: %x", rf_read_bbc_register(BBC_AFFTM, rf_module));

    rf_write_rf_register_field(RF_AGCC, rf_module, AGCI, AGCI);
    tr_debug("RF_AGCC: %x", rf_read_rf_register(RF_AGCC, rf_module));

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
    TEST_ACK_TX_STARTED
    rf_lock();
    if ((rf_state != RF_IDLE) && (rf_state != RF_RX_STARTED)) {
        rf_unlock();
        TEST_ACK_TX_DONE
        return -1;
    }
    if (rf_state == RF_RX_STARTED) {
        rf_state = RF_CSMA_WHILE_RX;
    } else {
        rf_state = RF_CSMA_STARTED;
    }
    // If Ack is requested, store the MAC sequence. This will be compared with received Ack.
    uint8_t version = ((*(data_ptr + 1) & VERSION_FIELD_MASK) >> SHIFT_VERSION_FIELD);
    if ((version != MAC_FRAME_VERSION_2) && (*data_ptr & FC_AR)) {
        tx_sequence = *(data_ptr + 2);
    }
    rf_write_tx_buffer(data_ptr, data_length, rf_module);
    rf_write_tx_packet_length(data_length + 2);
    mac_tx_handle = tx_handle;
    rf->cca_timer.attach_us(rf_csma_ca_timer_signal, 100);
    rf_unlock();
    return 0;
}

static void rf_handle_cca_ed_done(void)
{
    rf_irq_rf_disable(EDC, RF_09);
    rf_irq_rf_disable(EDC, rf_module);
    TEST_ACK_TX_DONE
    if ((rf_state == RF_RX_STARTED) || (rf_state == RF_CSMA_WHILE_RX)) {
        rf_state = RF_RX_STARTED;
        device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_CCA_FAIL, 0, 0);
        return;
    }
    rf_backup_timer_stop();
    if (((int8_t) rf_read_rf_register(RF_EDV, rf_module) > cca_threshold)) {
        //TEST2_ON
        rf_state = RF_IDLE;
        device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_CCA_FAIL, 0, 0);
        //TEST2_OFF
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
    rf_receive(rf_rx_channel);
    device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_TX_SUCCESS, 0, 0);
}

static void rf_start_tx(void)
{
    rf_change_state(RF_TXPREP);
    rf_irq_bbc_enable(TXFE, rf_module);
    rf_change_state(RF_TX);
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

static void rf_handle_rx_done(void)
{
    TEST_RX_DONE
    rf_backup_timer_stop();
    if (rf_state == RF_CSMA_WHILE_RX) {
        rf_state = RF_CSMA_STARTED;
        rf_backup_timer_start(MAX_PACKET_SENDING_TIME);
    }

    if (rf_read_bbc_register(BBC_PC, rf_module) & FCSOK) {
        uint16_t rx_data_length = rf_read_rx_frame_length(rf_module);
        if (!rf_read_rx_buffer(rx_data_length, rf_module)) {
            uint8_t version = ((rx_buffer[1] & VERSION_FIELD_MASK) >> SHIFT_VERSION_FIELD);
            if (((rx_buffer[0] & MAC_FRAME_TYPE_MASK) == MAC_TYPE_ACK) && (version < MAC_FRAME_VERSION_2)) {
                rf_handle_ack(rx_buffer[2], rx_buffer[0] & MAC_DATA_PENDING);
            } else {
                int8_t rssi = (int8_t) rf_read_rf_register(RF_EDV, rf_module);
                device_driver.phy_rx_cb(rx_buffer, rx_data_length - 2, 0xf0, rssi, rf_radio_driver_id);
                // If auto ack used, must wait until RF returns to RF_TXPREP state
                if ((version != MAC_FRAME_VERSION_2) && (rx_buffer[0] & FC_AR)) {
                    wait_us(50);
                    rf_poll_state_change(RF_TXPREP, rf_module);
                }
            }
        }
    }
    if (rf_state == RF_RX_STARTED) {
        rf_state = RF_IDLE;
    }
    rf_receive(rf_rx_channel);
}

static void rf_handle_rx_start(void)
{
    if (rf_state == RF_CSMA_STARTED) {
        rf_backup_timer_stop();
        rf_state = RF_CSMA_WHILE_RX;
    } else {
        rf_state = RF_RX_STARTED;
    }
    TEST_RX_STARTED
    rf_backup_timer_start(MAX_PACKET_SENDING_TIME);
}

static void rf_receive(uint16_t rx_channel)
{
    TEST_RX_DONE
    TEST1_ON
    rf_lock();
    if (rx_channel != rf_rx_channel) {
        rf_change_state(RF_TXPREP);
        rf_write_rf_register(RF_CNL, rf_module, (uint8_t) rx_channel);
        rf_write_rf_register_field(RF_CNM, rf_module, CNH, (uint8_t) (rx_channel >> 8));
        rf_rx_channel = rx_channel;
    }
    rf_change_state(RF_RX);
    rf_irq_bbc_enable(RXFS, rf_module);
    rf_irq_bbc_enable(RXFE, rf_module);
    rf_unlock();
    TEST1_OFF
}

static void rf_interrupt_handler(void)
{
    rf->irq_thread_215.flags_set(SIG_RADIO);
}

static void rf_irq_task_process_irq(void)
{
    TEST2_ON
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
    TEST2_OFF
}

static void rf_write_tx_packet_length(uint16_t packet_length)
{
    if (packet_length > 2047) {
        return;
    }
    rf_write_bbc_register(BBC_TXFLH, rf_module, (packet_length >> 8) & 0x03);
    rf_write_bbc_register(BBC_TXFLL, rf_module, (uint8_t) packet_length);
}

static uint16_t rf_read_rx_frame_length(rf_modules_e module)
{
    const uint8_t tx[2] = { static_cast<uint8_t>(module + 2), static_cast<uint8_t>(BBC_RXFLL) };
    uint8_t rx[2];
    CS_SELECT();
    rf_spi_exchange(tx, 2, NULL, 0);
    rf_spi_exchange(NULL, 0, rx, 2);
    CS_RELEASE();
    return (uint16_t)((rx[1] << 8) | rx[0]);
}

static uint16_t rf_read_frame_buffer_level(rf_modules_e module)
{
    const uint8_t tx[2] = { static_cast<uint8_t>(module + 2), static_cast<uint8_t>(BBC_FBLL) };
    uint8_t rx[2];
    CS_SELECT();
    rf_spi_exchange(tx, 2, NULL, 0);
    rf_spi_exchange(NULL, 0, rx, 2);
    CS_RELEASE();
    return (uint16_t)((rx[1] << 8) | rx[0]);
}

static void rf_write_tx_buffer(uint8_t *data, uint16_t len, rf_modules_e module)
{
    uint16_t buffer_addr = BBC0_FBTXS + (0x1000 * (module - 1));
    const uint8_t tx[2] = { static_cast<uint8_t>(0x80 | (buffer_addr >> 8)), static_cast<uint8_t>(buffer_addr) };
    CS_SELECT();
    rf_spi_exchange(tx, 2, NULL, 0);
    rf_spi_exchange(data, len, NULL, 0);
    CS_RELEASE();
}

static int rf_read_rx_buffer(uint16_t length, rf_modules_e module)
{
    if (length > RF_MTU) {
        return -1;
    }
    uint8_t *ptr = rx_buffer;
    uint16_t buffer_addr = BBC0_FBRXS + (0x1000 * (module - 1));
    const uint8_t tx[2] = { static_cast<uint8_t>(buffer_addr >> 8), static_cast<uint8_t>(buffer_addr) };
    CS_SELECT();
    rf_spi_exchange(tx, 2, NULL, 0);
    rf_spi_exchange(NULL, 0, ptr, length);
    CS_RELEASE();
    return 0;
}

static void rf_irq_rf_enable(uint8_t irq, rf_modules_e module)
{
    if ((module == RF_09) && !(rf09_irq_mask & irq)) {
        rf_write_rf_register_field(RF_IRQM, rf_module, irq, irq);
        rf09_irq_mask |= irq;
    } else if ((module == RF_24) && !(rf24_irq_mask & irq)) {
        rf_write_rf_register_field(RF_IRQM, rf_module, irq, irq);
        rf24_irq_mask |= irq;
    }
}

static void rf_irq_rf_disable(uint8_t irq, rf_modules_e module)
{
    if ((module == RF_09) && (rf09_irq_mask & irq)) {
        rf_write_rf_register_field(RF_IRQM, rf_module, irq, 0);
        rf09_irq_mask &= ~irq;
    } else if ((module == RF_24) && (rf24_irq_mask & irq)) {
        rf_write_rf_register_field(RF_IRQM, rf_module, irq, 0);
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
            tr_err("Failed to change state from %x to: %x", rf_read_state(module), state);
            break;
        }
    }
}

static void rf_change_state(rf_command_e state)
{
    rf_write_rf_register(RF_CMD, rf_module, state);
    return rf_poll_state_change(state, rf_module);
}

static void rf_spi_exchange(const void *tx, size_t tx_len, void *rx, size_t rx_len)
{
    rf->spi.write(static_cast<const char *>(tx), tx_len, static_cast<char *>(rx), rx_len);
}

static uint8_t rf_read_rf_register(uint8_t addr, rf_modules_e module)
{
    const uint8_t tx[2] = { static_cast<uint8_t>(module), static_cast<uint8_t>(addr) };
    uint8_t rx[3];
    CS_SELECT();
    rf_spi_exchange(tx, 2, rx, 3);
    CS_RELEASE();
    return rx[2];
}

static void rf_write_rf_register(uint8_t addr, rf_modules_e module, uint8_t data)
{
    const uint8_t tx[3] = { static_cast<uint8_t>(0x80 | module), static_cast<uint8_t>(addr), static_cast<uint8_t>(data) };
    uint8_t rx[2];
    CS_SELECT();
    rf_spi_exchange(tx, 3, rx, 2);
    CS_RELEASE();
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
    rf_irq_rf_disable(EDC, rf_module);
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
        TEST_ACK_TX_DONE
        device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_CCA_FAIL, 0, 0);
    }
    TEST_TX_DONE
    TEST_RX_DONE
    rf_state = RF_IDLE;
    rf_receive(rf_rx_channel);
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

static void (*irq_callbacks[3])(void);

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

int RFBits::init_215_driver(RFBits *_rf, const uint8_t mac[8], uint8_t *rf_part_num)
{
    rf = _rf;
    irq_thread_215.start(mbed::callback(this, &RFBits::rf_irq_task));
    *rf_part_num = rf_read_common_register(RF_PN);
    return rf_device_register(mac);
}

#endif // MBED_CONF_NANOSTACK_CONFIGURATION && DEVICE_SPI && DEVICE_INTERRUPTIN && defined(MBED_CONF_RTOS_PRESENT)
