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
#include "mbed_trace.h"
#include <Timer.h>
#include "Timeout.h"
#include "SPI.h"

#define TRACE_GROUP "AtRF"

typedef enum {
    RF_NOP = 0x00,
    RF_SLEEP = 0x01,
    RF_TRX_OFF = 0x02,
    RF_TXPREP = 0x03,
    RF_TX = 0x04,
    RF_RX = 0x05,
    RF_RESET = 0x07
} rf_states_t;

typedef enum {
    COMMON = 0x00,
    RF_09 = 0x01,
    RF_24 = 0x02,
    BBC0 = 0x03,
    BBC1 = 0x04
} rf_modules_t;

static void rf_init(void);
static int8_t rf_address_write(phy_address_type_e address_type, uint8_t *address_ptr);
static int8_t rf_extension(phy_extension_type_e extension_type, uint8_t *data_ptr);
static int8_t rf_interface_state_control(phy_interface_state_e new_state, uint8_t rf_channel);
static int8_t rf_start_csma_ca(uint8_t *data_ptr, uint16_t data_length, uint8_t tx_handle, data_protocol_e data_protocol);
static void rf_init_registers(void);
static uint8_t rf_read_register(uint8_t addr, rf_modules_t module);
static void rf_write_register(uint8_t addr, rf_modules_t module, uint8_t data);
static void rf_write_register_field(uint8_t addr, rf_modules_t module, uint8_t field, uint8_t value);
static rf_states_t rf_read_state(rf_modules_t module);
static void rf_poll_state_change(rf_states_t state, rf_modules_t module);
static void rf_change_trx_state(rf_states_t state);
static void rf_receive(void);
static void rf_interrupt_handler(void);
static void rf_irq_task_process_irq(void);

static int8_t rf_radio_driver_id = -1;
static phy_device_driver_s device_driver;
static rf_modules_t rf_module = RF_24;

/* Channel configurations for 2.4 and sub-GHz */
static const phy_rf_channel_configuration_s phy_24ghz = {.channel_0_center_frequency = 2405000000U, .channel_spacing = 5000000U, .datarate = 250000U, .number_of_channels = 16U, .modulation = M_OQPSK};
static const phy_rf_channel_configuration_s phy_subghz = {.channel_0_center_frequency = 868300000U, .channel_spacing = 2000000U, .datarate = 250000U, .number_of_channels = 11U, .modulation = M_OQPSK};

static const phy_device_channel_page_s phy_channel_pages[] = {
    { CHANNEL_PAGE_0, &phy_24ghz},
    { CHANNEL_PAGE_2, &phy_subghz},
    { CHANNEL_PAGE_0, NULL}
};

using namespace mbed;
using namespace rtos;

#include "rfbits.h"

static RFBits *rf;

#define SIG_RADIO       1
#define SIG_TIMER_ACK   2
#define SIG_TIMER_CAL   4
#define SIG_TIMER_CCA   8

#define SIG_TIMERS (SIG_TIMER_ACK|SIG_TIMER_CAL|SIG_TIMER_CCA)
#define SIG_ALL (SIG_RADIO|SIG_TIMERS)

// t1 = 180ns, SEL falling edge to MISO active [SPI setup assumed slow enough to not need manual delay]
#define CS_SELECT()  {rf->CS = 0; /* delay_ns(180); */}
// t9 = 250ns, last clock to SEL rising edge, t8 = 250ns, SPI idle time between consecutive access
#define CS_RELEASE() {wait_ns(250); rf->CS = 1; wait_ns(250);}

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
    device_driver.link_type = PHY_LINK_15_4_SUBGHZ_TYPE;
    device_driver.phy_channel_pages = phy_channel_pages;
    device_driver.phy_MTU = 127;
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
    int8_t ret_val = 0;
    switch (address_type) {
        /*Set 48-bit address*/
        case PHY_MAC_48BIT:
            break;
        /*Set 64-bit address*/
        case PHY_MAC_64BIT:
            //rf_set_address(address_ptr);
            break;
        /*Set 16-bit address*/
        case PHY_MAC_16BIT:
            //rf_set_short_adr(address_ptr);
            break;
        /*Set PAN Id*/
        case PHY_MAC_PANID:
            //rf_set_pan_id(address_ptr);
            break;
    }
    return ret_val;
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
            tr_debug("enable irq");
            rf->IRQ.enable_irq();
            break;
        case PHY_INTERFACE_RX_ENERGY_STATE:
            break;
        case PHY_INTERFACE_SNIFFER_STATE:
            break;
    }
    return ret_val;
}

void rf_csma_ca_interrupt(void)
{
    tr_debug("cca");
    rf_write_register_field(RF_EDC, rf_module, EDM, RF_EDSINGLE);
    tr_debug("RF_EDC: %x", rf_read_register(RF_EDC, rf_module));
}

static void rf_csma_ca_timer_signal(void)
{
    rf->irq_thread_215.flags_set(SIG_TIMER_CCA);
}

static int8_t rf_start_csma_ca(uint8_t *data_ptr, uint16_t data_length, uint8_t tx_handle, data_protocol_e data_protocol)
{
    rf->cca_timer.attach_us(rf_csma_ca_timer_signal, 1000);
    return 0;
}

static void rf_interrupt_handler(void)
{
    rf->irq_thread_215.flags_set(SIG_RADIO);
}

static void rf_irq_task_process_irq(void)
{
    uint8_t irq_status = rf_read_register(RF24_IRQS, COMMON);
    if (irq_status & EDC) {
        tr_debug("i");
    }
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
            rf_csma_ca_interrupt();
        }

        rf_unlock();
    }
}

static void rf_init(void)
{
    rf_lock();

    rf_init_registers();
    tr_debug("RF state: %u", rf_read_state(rf_module));
    rf_change_trx_state(RF_TRX_OFF);
    tr_debug("RF state: %u", rf_read_state(rf_module));
    rf_change_trx_state(RF_TXPREP);
    tr_debug("RF state: %u", rf_read_state(rf_module));
    rf_receive();
    tr_debug("RF state: %u", rf_read_state(rf_module));

    rf->IRQ.rise(&rf_interrupt_handler);

    rf_unlock();
}

static void rf_init_registers(void)
{
    //rf_if_write_register(RF09_IRQM, EDC, true);
    rf_write_register(RF_IRQM, RF_09, EDC);
    rf_write_register(RF_IRQM, rf_module, EDC);
    tr_debug("RF24_IRQM: %x", rf_read_register(RF_IRQM, rf_module));

    tr_debug("RF_CFG: %x", rf_read_register(RF_CFG, COMMON));

    // Ensure basebands enabled, I/Q IF's disabled
    rf_write_register_field(RF_IQIFC1, COMMON, CHPM, RF_MODE_BBRF);
    tr_debug("RF_IQIFC1: %x", rf_read_register(RF_IQIFC1, COMMON));
}

static void rf_receive(void)
{
    rf_change_trx_state(RF_RX);
}


static rf_states_t rf_read_state(rf_modules_t module)
{
    return (rf_states_t) rf_read_register(RF_STATE, rf_module);
}

static void rf_poll_state_change(rf_states_t state, rf_modules_t module)
{
    uint16_t while_counter = 0;
    while (rf_read_state(module) != state) {
        while_counter++;
        if (while_counter == 0x1ff) {
            break;
        }
    }
}

static void rf_change_trx_state(rf_states_t state)
{
    rf_write_register(RF_CMD, rf_module, state);
    return rf_poll_state_change(state, rf_module);
}

static void rf_spi_exhange(const void *out, void *in, uint8_t out_len, uint8_t in_len)
{
    CS_SELECT();
    rf->spi.write(static_cast<const char *>(out), out_len, static_cast<char *>(in), in_len);
    CS_RELEASE();
}

static uint8_t rf_read_register(uint8_t addr, rf_modules_t module)
{
    const uint8_t tx[2] = { static_cast<uint8_t>(module), static_cast<uint8_t>(addr) };
    uint8_t rx[3];
    rf_spi_exhange(tx, rx, 2, 3);
    return rx[2];
}

static void rf_write_register(uint8_t addr, rf_modules_t module, uint8_t data)
{
    const uint8_t tx[3] = { static_cast<uint8_t>(0x80 | module), static_cast<uint8_t>(addr), static_cast<uint8_t>(data) };
    uint8_t rx[2];
    rf_spi_exhange(tx, rx, 3, 2);
}

static void rf_write_register_field(uint8_t addr, rf_modules_t module, uint8_t field, uint8_t value)
{
    uint8_t reg_tmp = rf_read_register(addr, module);
    reg_tmp &= ~field;
    reg_tmp |= value;
    rf_write_register(addr, module, reg_tmp);
}

int RFBits::init_215_driver(RFBits *_rf, const uint8_t mac[8], uint8_t *rf_part_num)
{
    rf = _rf;
    tr_debug("init 215");
    tr_debug("MAC: %s", trace_array(mac, 8));
    irq_thread_215.start(mbed::callback(this, &RFBits::rf_irq_task));
    *rf_part_num = rf_read_register(RF_PN, COMMON);
    return rf_device_register(mac);
}

#endif // MBED_CONF_NANOSTACK_CONFIGURATION && DEVICE_SPI && DEVICE_INTERRUPTIN && defined(MBED_CONF_RTOS_PRESENT)

