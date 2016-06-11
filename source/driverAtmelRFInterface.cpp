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
#include "platform/arm_hal_interrupt.h"
#include "nanostack/platform/arm_hal_phy.h"
#include "atmel-rf-driver/driverRFPhy.h"
#include "driverAtmelRFInterface.h"
#include "mbed-drivers/mbed.h"

#ifdef MBED_CONF_RTOS_PRESENT
#include "Mutex.h"
#include "Thread.h"
using namespace rtos;

static void rf_if_irq_task_process_irq();

#define SIG_RADIO       1
#define SIG_TIMER_ACK   2
#define SIG_TIMER_CAL   4
#define SIG_TIMER_CCA   8

#define SIG_TIMERS (SIG_TIMER_ACK|SIG_TIMER_CAL|SIG_TIMER_CCA)
#define SIG_ALL (SIG_RADIO|SIG_TIMERS)
#endif

// HW pins to RF chip
#define SPI_SPEED 7500000

class UnlockedSPI : public SPI {
public:
    UnlockedSPI(PinName mosi, PinName miso, PinName sclk, PinName ssel=NC) :
        SPI(mosi, miso, sclk, ssel) { }
    virtual void lock() { }
    virtual void unlock() { }
};

struct RFBits {
    RFBits();
    UnlockedSPI spi;
    DigitalOut CS;
    DigitalOut RST;
    DigitalOut SLP_TR;
    InterruptIn IRQ;
    Timeout ack_timer;
    Timeout cal_timer;
    Timeout cca_timer;
#ifdef MBED_CONF_RTOS_PRESENT
    Thread irq_thread;
    Mutex mutex;
    void rf_if_irq_task();
#endif
};

RFBits::RFBits() :
spi(PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCLK),
CS(PIN_SPI_CS),
RST(PIN_SPI_RST),
SLP_TR(PIN_SPI_SLP),
IRQ(PIN_SPI_IRQ)
#ifdef MBED_CONF_RTOS_PRESENT
,irq_thread(osPriorityRealtime, 1024)
#endif
{
    irq_thread.start(this, &RFBits::rf_if_irq_task);
}

void (*app_rf_settings_cb)(void) = 0;
static RFBits *rf;
static uint8_t rf_part_num = 0;
static uint8_t rf_rx_lqi;
static int8_t rf_rx_rssi;
static uint8_t rf_rx_status;
/*TODO: RSSI Base value setting*/
static int8_t rf_rssi_base_val = -91;

static uint8_t rf_if_spi_exchange(uint8_t out);

void rf_if_lock(void)
{
    platform_enter_critical();
}

void rf_if_unlock(void)
{
    platform_exit_critical();
}

#ifdef MBED_CONF_RTOS_PRESENT
static void rf_if_cca_timer_signal(void)
{
    rf->irq_thread.signal_set(SIG_TIMER_CCA);
}

static void rf_if_cal_timer_signal(void)
{
    rf->irq_thread.signal_set(SIG_TIMER_CAL);
}

static void rf_if_ack_timer_signal(void)
{
    rf->irq_thread.signal_set(SIG_TIMER_ACK);
}
#endif


/* Delay functions for RF Chip SPI access */
#ifdef __CC_ARM
__asm static void delay_loop(uint32_t count)
{
1
  SUBS a1, a1, #1
  BCS  %BT1
  BX   lr
}
#elif defined (__ICCARM__)
static void delay_loop(uint32_t count)
{
  __asm volatile(
    "loop: \n"
    " SUBS %0, %0, #1 \n"
    " BCS.n  loop\n"
    : "+r" (count)
    :
    : "cc"
  );
}
#else // GCC
static void delay_loop(uint32_t count)
{
  __asm__ volatile (
    "%=:\n\t"
    "SUBS %0, %0, #1\n\t"
    "BCS  %=b\n\t"
    : "+r" (count)
    :
    : "cc"
  );
}
#endif

static void delay_ns(uint32_t ns)
{
  uint32_t cycles_per_us = SystemCoreClock / 1000000;
  // Cortex-M0 takes 4 cycles per loop (SUB=1, BCS=3)
  // Cortex-M3 and M4 takes 3 cycles per loop (SUB=1, BCS=2)
  // Cortex-M7 - who knows?
  // Cortex M3-M7 have "CYCCNT" - would be better than a software loop, but M0 doesn't
  // Assume 3 cycles per loop for now - will be 33% slow on M0. No biggie,
  // as original version of code was 300% slow on M4.
  // [Note that this very calculation, plus call overhead, will take multiple
  // cycles. Could well be 100ns on its own... So round down here, startup is
  // worth at least one loop iteration.]
  uint32_t count = (cycles_per_us * ns) / 3000;

  delay_loop(count);
}

// t1 = 180ns, SEL falling edge to MISO active [SPI setup assumed slow enough to not need manual delay]
#define CS_SELECT()  {rf->CS = 0; /* delay_ns(180); */}
 // t9 = 250ns, last clock to SEL rising edge, t8 = 250ns, SPI idle time between consecutive access
#define CS_RELEASE() {delay_ns(250); rf->CS = 1; delay_ns(250);}

/*
 * \brief Read connected radio part.
 *
 * This function only return valid information when rf_init() is called
 *
 * \return
 */
rf_trx_part_e rf_radio_type_read(void)
{
  rf_trx_part_e ret_val = ATMEL_UNKNOW_DEV;

  switch (rf_part_num)
  {
    case PART_AT86RF212:
      ret_val = ATMEL_AT86RF212;
      break;

    case PART_AT86RF231:
      ret_val = ATMEL_AT86RF231;
      break;
    case PART_AT86RF233:
      ret_val = ATMEL_AT86RF231;
      break;
    default:
      break;
  }

  return ret_val;
}


/*
 * \brief Function starts the ACK wait timeout.
 *
 * \param slots Given slots, resolution 50us
 *
 * \return none
 */
void rf_if_ack_wait_timer_start(uint16_t slots)
{
#ifdef MBED_CONF_RTOS_PRESENT
  rf->ack_timer.attach(rf_if_ack_timer_signal, slots*50e-6);
#else
  rf->ack_timer.attach(rf_ack_wait_timer_interrupt, slots*50e-6);
#endif
}

/*
 * \brief Function starts the calibration interval.
 *
 * \param slots Given slots, resolution 50us
 *
 * \return none
 */
void rf_if_calibration_timer_start(uint32_t slots)
{
#ifdef MBED_CONF_RTOS_PRESENT
  rf->cal_timer.attach(rf_if_cal_timer_signal, slots*50e-6);
#else
  rf->cal_timer.attach(rf_calibration_timer_interrupt, slots*50e-6);
#endif
}

/*
 * \brief Function starts the CCA interval.
 *
 * \param slots Given slots, resolution 50us
 *
 * \return none
 */
void rf_if_cca_timer_start(uint32_t slots)
{
#ifdef MBED_CONF_RTOS_PRESENT
  rf->cca_timer.attach(rf_if_cca_timer_signal, slots*50e-6);
#else
  rf->cca_timer.attach(rf_cca_timer_interrupt, slots*50e-6);
#endif
}

/*
 * \brief Function stops the ACK wait timeout.
 *
 * \param none
 *
 * \return none
 */
void rf_if_ack_wait_timer_stop(void)
{
  rf->ack_timer.detach();
}


/*
 * \brief Function sets SLP_TR pin high in RF interface.
 *
 * \param none
 *
 * \return none
 */
void rf_if_slp_tr_pin_high(void)
{
  rf->SLP_TR = 1;
}

/*
 * \brief Function sets SLP_TR pin low in RF interface.
 *
 * \param none
 *
 * \return none
 */
void rf_if_slp_tr_pin_low(void)
{
  rf->SLP_TR = 0;
}


/*
 * \brief Function reads data from the given RF SRAM address.
 *
 * \param ptr Read pointer
 * \param sram_address Read address in SRAM
 * \param len Length of the read
 *
 * \return none
 */
void rf_if_read_payload(uint8_t *ptr, uint8_t sram_address, uint8_t len)
{
  uint8_t i;

  CS_SELECT();
  rf_if_spi_exchange(0x20);
  rf_if_spi_exchange(sram_address);
  for(i=0; i<len; i++)
    *ptr++ = rf_if_spi_exchange(0);

  /*Read LQI and RSSI in variable*/
  rf_rx_lqi = rf_if_spi_exchange(0);
  rf_rx_rssi = rf_if_spi_exchange(0);
  rf_rx_status = rf_if_spi_exchange(0);
  CS_RELEASE();
}

/*
 * \brief Function sets bit(s) in given RF register.
 *
 * \param addr Address of the register to set
 * \param bit Bit(s) to set
 * \param bit_mask Masks the field inside the register
 *
 * \return none
 */
void rf_if_set_bit(uint8_t addr, uint8_t bit, uint8_t bit_mask)
{
  uint8_t reg = rf_if_read_register(addr);
  reg &= ~bit_mask;
  reg |= bit;
  rf_if_write_register(addr, reg);
}

/*
 * \brief Function clears bit(s) in given RF register.
 *
 * \param addr Address of the register to clear
 * \param bit Bit(s) to clear
 *
 * \return none
 */
void rf_if_clear_bit(uint8_t addr, uint8_t bit)
{
  uint8_t reg = rf_if_read_register(addr);
  reg &= ~bit;
  rf_if_write_register(addr, reg);
}

/*
 * \brief Function writes register in RF.
 *
 * \param addr Address on the RF
 * \param data Written data
 *
 * \return none
 */
void rf_if_write_register(uint8_t addr, uint8_t data)
{
  uint8_t cmd = 0xC0;
  CS_SELECT();
  rf_if_spi_exchange(cmd | addr);
  rf_if_spi_exchange(data);
  CS_RELEASE();
}

/*
 * \brief Function reads RF register.
 *
 * \param addr Address on the RF
 *
 * \return Read data
 */
uint8_t rf_if_read_register(uint8_t addr)
{
  uint8_t cmd = 0x80;
  uint8_t data;
  CS_SELECT();
  rf_if_spi_exchange(cmd | addr);
  data = rf_if_spi_exchange(0);
  CS_RELEASE();
  return data;
}

/*
 * \brief Function resets the RF.
 *
 * \param none
 *
 * \return none
 */
void rf_if_reset_radio(void)
{
  if (!rf) {
      rf = new RFBits;
  }
  rf->spi.frequency(SPI_SPEED);
  rf->IRQ.rise(0);
  rf->RST = 1;
  wait(10e-4);
  rf->RST = 0;
  wait(10e-3);
  CS_RELEASE();
  rf->SLP_TR = 0;
  wait(10e-3);
  rf->RST = 1;
  wait(10e-3);

  rf->IRQ.rise(&rf_if_interrupt_handler);
}

/*
 * \brief Function enables the promiscuous mode.
 *
 * \param none
 *
 * \return none
 */
void rf_if_enable_promiscuous_mode(void)
{
  /*Set AACK_PROM_MODE to enable the promiscuous mode*/
  rf_if_set_bit(XAH_CTRL_1, AACK_PROM_MODE, AACK_PROM_MODE);
}

/*
 * \brief Function enables the promiscuous mode.
 *
 * \param none
 *
 * \return none
 */
void rf_if_disable_promiscuous_mode(void)
{
  /*Set AACK_PROM_MODE to enable the promiscuous mode*/
  rf_if_clear_bit(XAH_CTRL_1, AACK_PROM_MODE);
}

/*
 * \brief Function enables the Antenna diversity usage.
 *
 * \param none
 *
 * \return none
 */
void rf_if_enable_ant_div(void)
{
  /*Set ANT_EXT_SW_EN to enable controlling of antenna diversity*/
  rf_if_set_bit(ANT_DIV, ANT_EXT_SW_EN, ANT_EXT_SW_EN);
}

/*
 * \brief Function disables the Antenna diversity usage.
 *
 * \param none
 *
 * \return none
 */
void rf_if_disable_ant_div(void)
{
  rf_if_clear_bit(ANT_DIV, ANT_EXT_SW_EN);
}

/*
 * \brief Function sets the SLP TR pin.
 *
 * \param none
 *
 * \return none
 */
void rf_if_enable_slptr(void)
{
  rf->SLP_TR = 1;
}

/*
 * \brief Function clears the SLP TR pin.
 *
 * \param none
 *
 * \return none
 */
void rf_if_disable_slptr(void)
{
  rf->SLP_TR = 0;
}

/*
 * \brief Function writes the antenna diversity settings.
 *
 * \param none
 *
 * \return none
 */
void rf_if_write_antenna_diversity_settings(void)
{
  /*Recommended setting of PDT_THRES is 3 when antenna diversity is used*/
  rf_if_set_bit(RX_CTRL, 0x03, 0x0f);
  rf_if_write_register(ANT_DIV, ANT_DIV_EN | ANT_EXT_SW_EN | ANT_CTRL_DEFAULT);
}

/*
 * \brief Function writes the TX output power register.
 *
 * \param value Given register value
 *
 * \return none
 */
void rf_if_write_set_tx_power_register(uint8_t value)
{
  rf_if_write_register(PHY_TX_PWR, value);
}

/*
 * \brief Function returns the RF part number.
 *
 * \param none
 *
 * \return part number
 */
uint8_t rf_if_read_part_num(void)
{
  return rf_if_read_register(PART_NUM);
}

/*
 * \brief Function writes the RF settings and initialises SPI interface.
 *
 * \param none
 *
 * \return none
 */
void rf_if_write_rf_settings(void)
{
  /*Reset RF module*/
  rf_if_reset_radio();

  rf_part_num = rf_if_read_part_num();

  rf_if_write_register(XAH_CTRL_0,0);
  rf_if_write_register(TRX_CTRL_1, 0x20);

  /*CCA Mode - Carrier sense OR energy above threshold. Channel list is set separately*/
  rf_if_write_register(PHY_CC_CCA, 0x05);

  /*Read transceiver PART_NUM*/
  rf_part_num = rf_if_read_register(PART_NUM);

  /*Sub-GHz RF settings*/
  if(rf_part_num == PART_AT86RF212)
  {
    /*GC_TX_OFFS mode-dependent setting - OQPSK*/
    rf_if_write_register(RF_CTRL_0, 0x32);

    if(rf_if_read_register(VERSION_NUM) == VERSION_AT86RF212B)
    {
      /*TX Output Power setting - 0 dBm North American Band*/
      rf_if_write_register(PHY_TX_PWR, 0x03);
    }
    else
    {
      /*TX Output Power setting - 0 dBm North American Band*/
      rf_if_write_register(PHY_TX_PWR, 0x24);
    }

    /*PHY Mode: IEEE 802.15.4-2006/2011 - OQPSK-SIN-250*/
    rf_if_write_register(TRX_CTRL_2, RF_PHY_MODE);
    /*Based on receiver Characteristics. See AT86RF212B Datasheet where RSSI BASE VALUE in range -97 - -100 dBm*/
    rf_rssi_base_val = -98;
  }
  /*2.4GHz RF settings*/
  else
  {
    /*Set RPC register*/
    rf_if_write_register(TRX_RPC, 0xef);
    /*PHY Mode: IEEE 802.15.4 - Data Rate 250 kb/s*/
    rf_if_write_register(TRX_CTRL_2, 0);
    rf_rssi_base_val = -91;
  }
}


/*
 * \brief Function checks the channel availability
 *
 * \param none
 *
 * \return 1 Channel clear
 * \return 0 Channel not clear
 */
uint8_t rf_if_check_cca(void)
{
  uint8_t retval = 0;
  if(rf_if_read_register(TRX_STATUS) & CCA_STATUS)
  {
    retval = 1;
  }
  return retval;
}

/*
 * \brief Function checks if the CRC is valid in received frame
 *
 * \param none
 *
 * \return 1 CRC ok
 * \return 0 CRC failed
 */
uint8_t rf_if_check_crc(void)
{
  uint8_t retval = 0;
  if(rf_if_read_register(PHY_RSSI) & CRC_VALID)
  {
    retval = 1;
  }
  return retval;
}

/*
 * \brief Function returns the RF state
 *
 * \param none
 *
 * \return RF state
 */
uint8_t rf_if_read_trx_state(void)
{
  return rf_if_read_register(TRX_STATUS) & 0x1F;
}

/*
 * \brief Function reads data from RF SRAM.
 *
 * \param ptr Read pointer
 * \param len Length of the read
 *
 * \return none
 */
void rf_if_read_packet(uint8_t *ptr, uint8_t len)
{
  if(rf_part_num == PART_AT86RF231 || rf_part_num == PART_AT86RF212)
    rf_if_read_payload(ptr, 0, len);
  else if(rf_part_num == PART_AT86RF233)
    rf_if_read_payload(ptr, 1, len);
}

/*
 * \brief Function writes RF short address registers
 *
 * \param short_address Given short address
 *
 * \return none
 */
void rf_if_write_short_addr_registers(uint8_t *short_address)
{
  rf_if_write_register(SHORT_ADDR_1, *short_address++);
  rf_if_write_register(SHORT_ADDR_0, *short_address);
}

/*
 * \brief Function sets the frame pending in ACK message
 *
 * \param state Given frame pending state
 *
 * \return none
 */
void rf_if_ack_pending_ctrl(uint8_t state)
{
  rf_if_lock();
  if(state)
  {
    rf_if_set_bit(CSMA_SEED_1, (1 << AACK_SET_PD), (1 << AACK_SET_PD));
  }
  else
  {
    rf_if_clear_bit(CSMA_SEED_1, (1 << AACK_SET_PD));
  }
  rf_if_unlock();
}

/*
 * \brief Function returns the state of frame pending control
 *
 * \param none
 *
 * \return Frame pending state
 */
uint8_t rf_if_last_acked_pending(void)
{
  uint8_t last_acked_data_pending;

  rf_if_lock();
  if(rf_if_read_register(CSMA_SEED_1) & 0x20)
    last_acked_data_pending = 1;
  else
    last_acked_data_pending = 0;
  rf_if_unlock();

  return last_acked_data_pending;
}

/*
 * \brief Function calibrates the RF part.
 *
 * \param none
 *
 * \return none
 */
void rf_if_calibration(void)
{
  rf_if_set_bit(FTN_CTRL, FTN_START, FTN_START);
  /*Wait while calibration is running*/
  while(rf_if_read_register(FTN_CTRL) & FTN_START);
}

/*
 * \brief Function writes RF PAN Id registers
 *
 * \param pan_id Given PAN Id
 *
 * \return none
 */
void rf_if_write_pan_id_registers(uint8_t *pan_id)
{
  rf_if_write_register(PAN_ID_1, *pan_id++);
  rf_if_write_register(PAN_ID_0, *pan_id);
}

/*
 * \brief Function writes RF IEEE Address registers
 *
 * \param address Given IEEE Address
 *
 * \return none
 */
void rf_if_write_ieee_addr_registers(uint8_t *address)
{
  uint8_t i;
  uint8_t temp = IEEE_ADDR_0;

  for(i=0; i<8; i++)
    rf_if_write_register(temp++, address[7-i]);
}

/*
 * \brief Function writes data in RF frame buffer.
 *
 * \param ptr Pointer to data
 * \param length Pointer to length
 *
 * \return none
 */
void rf_if_write_frame_buffer(uint8_t *ptr, uint8_t length)
{
  uint8_t i;
  uint8_t cmd = 0x60;

  CS_SELECT();
  rf_if_spi_exchange(cmd);
  rf_if_spi_exchange(length + 2);
  for(i=0; i<length; i++)
    rf_if_spi_exchange(*ptr++);

  CS_RELEASE();
}

/*
 * \brief Function returns 8-bit random value.
 *
 * \param none
 *
 * \return random value
 */
uint8_t rf_if_read_rnd(void)
{
  uint8_t temp;
  uint8_t tmp_rpc_val = 0;
  /*RPC must be disabled while reading the random number*/
  if(rf_part_num == PART_AT86RF233)
  {
    tmp_rpc_val = rf_if_read_register(TRX_RPC);
    rf_if_write_register(TRX_RPC, 0xc1);
  }

  wait(1e-3);
  temp = ((rf_if_read_register(PHY_RSSI)>>5) << 6);
  wait(1e-3);
  temp |= ((rf_if_read_register(PHY_RSSI)>>5) << 4);
  wait(1e-3);
  temp |= ((rf_if_read_register(PHY_RSSI)>>5) << 2);
  wait(1e-3);
  temp |= ((rf_if_read_register(PHY_RSSI)>>5));
  wait(1e-3);
  if(rf_part_num == PART_AT86RF233)
    rf_if_write_register(TRX_RPC, tmp_rpc_val);
  return temp;
}

/*
 * \brief Function changes the state of the RF.
 *
 * \param trx_state Given RF state
 *
 * \return none
 */
void rf_if_change_trx_state(rf_trx_states_t trx_state)
{
  rf_if_lock();
  rf_if_write_register(TRX_STATE, trx_state);
  /*Wait while not in desired state*/
  rf_poll_trx_state_change(trx_state);
  rf_if_unlock();
}

/*
 * \brief Function enables the TX END interrupt
 *
 * \param none
 *
 * \return none
 */
void rf_if_enable_tx_end_interrupt(void)
{
  rf_if_set_bit(IRQ_MASK, TRX_END, 0x08);
}

/*
 * \brief Function enables the RX END interrupt
 *
 * \param none
 *
 * \return none
 */
void rf_if_enable_rx_end_interrupt(void)
{
  rf_if_set_bit(IRQ_MASK, TRX_END, 0x08);
}

/*
 * \brief Function enables the RX START interrupt
 *
 * \param none
 *
 * \return none
 */
void rf_if_enable_rx_start_interrupt(void)
{
  rf_if_set_bit(IRQ_MASK, RX_START, 0x04);
}

/*
 * \brief Function enables the CCA ED interrupt
 *
 * \param none
 *
 * \return none
 */
void rf_if_enable_cca_ed_done_interrupt(void)
{
  rf_if_set_bit(IRQ_MASK, CCA_ED_DONE, 0x10);
}

/*
 * \brief Function starts the CCA process
 *
 * \param none
 *
 * \return none
 */
void rf_if_start_cca_process(void)
{
  rf_if_set_bit(PHY_CC_CCA, CCA_REQUEST, 0x80);
}

/*
 * \brief Function returns the length of the received packet
 *
 * \param none
 *
 * \return packet length
 */
uint8_t rf_if_read_received_frame_length(void)
{
  uint8_t length;

  CS_SELECT();
  rf_if_spi_exchange(0x20);
  length = rf_if_spi_exchange(0);
  CS_RELEASE();

  return length;
}

/*
 * \brief Function returns the LQI of the received packet
 *
 * \param none
 *
 * \return packet LQI
 */
uint8_t rf_if_read_lqi(void)
{
  return rf_rx_lqi;
}

/*
 * \brief Function returns the RSSI of the received packet
 *
 * \param none
 *
 * \return packet RSSI
 */
int8_t rf_if_read_rssi(void)
{
  int16_t rssi_calc_tmp;
  if(rf_part_num == PART_AT86RF212)
    rssi_calc_tmp = rf_rx_rssi * 103;
  else
    rssi_calc_tmp = rf_rx_rssi * 100;
  rssi_calc_tmp /= 100;
  rssi_calc_tmp = (rf_rssi_base_val + rssi_calc_tmp);
  return (int8_t)rssi_calc_tmp;
}

/*
 * \brief Function returns the RSSI of the received packet
 *
 * \param none
 *
 * \return packet RSSI
 */
uint8_t rf_if_read_rx_status(void)
{
  return rf_rx_status;
}

/*
 * \brief Function sets the RF channel field
 *
 * \param Given channel
 *
 * \return none
 */
void rf_if_set_channel_register(uint8_t channel)
{
  rf_if_set_bit(PHY_CC_CCA, channel, 0x1f);
}

#ifdef MBED_CONF_RTOS_PRESENT
void rf_if_interrupt_handler(void)
{
    rf->irq_thread.signal_set(SIG_RADIO);
}

// Started during construction of rf, so variable
// rf isn't set at the start. Uses 'this' instead.
void RFBits::rf_if_irq_task(void)
{
    for (;;) {
        osEvent event = irq_thread.signal_wait(0);
        if (event.status != osEventSignal) {
            continue;
        }
        rf_if_lock();
        if (event.value.signals & SIG_RADIO) {
            rf_if_irq_task_process_irq();
        }
        if (event.value.signals & SIG_TIMER_ACK) {
            rf_ack_wait_timer_interrupt();
        }
        if (event.value.signals & SIG_TIMER_CCA) {
            rf_cca_timer_interrupt();
        }
        if (event.value.signals & SIG_TIMER_CAL) {
            rf_calibration_timer_interrupt();
        }
        rf_if_unlock();
    }
}

static void rf_if_irq_task_process_irq(void)
#else
/*
 * \brief Function is a RF interrupt vector. End of frame in RX and TX are handled here as well as CCA process interrupt.
 *
 * \param none
 *
 * \return none
 */
void rf_if_interrupt_handler(void)
#endif
{
  uint8_t irq_status;

  /*Read interrupt flag*/
  irq_status = rf_if_read_register(IRQ_STATUS);

  /*Disable interrupt on RF*/
  rf_if_clear_bit(IRQ_MASK, irq_status);
  /*RX start interrupt*/
  if(irq_status & RX_START)
  {
  }
  /*Address matching interrupt*/
  if(irq_status & AMI)
  {
  }
  if(irq_status & TRX_UR)
  {
  }
  /*Frame end interrupt (RX and TX)*/
  if(irq_status & TRX_END)
  {
    /*TX done interrupt*/
    if(rf_if_read_trx_state() == PLL_ON || rf_if_read_trx_state() == TX_ARET_ON)
    {
      rf_handle_tx_end();
    }
    /*Frame received interrupt*/
    else
    {
      rf_handle_rx_end();
    }
  }
  if(irq_status & CCA_ED_DONE)
  {
    rf_handle_cca_ed_done();
  }
}

/*
 * \brief Function writes/read data in SPI interface
 *
 * \param out Output data
 *
 * \return Input data
 */
uint8_t rf_if_spi_exchange(uint8_t out)
{
  uint8_t v;
  v = rf->spi.write(out);
  // t9 = t5 = 250ns, delay between LSB of last byte to next MSB or delay between LSB & SEL rising
  // [SPI setup assumed slow enough to not need manual delay]
  // delay_ns(250);
  return v;
}
