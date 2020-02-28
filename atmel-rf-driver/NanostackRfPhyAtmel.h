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

#ifndef NANOSTACK_RF_PHY_ATMEL_H_
#define NANOSTACK_RF_PHY_ATMEL_H_

#include "at24mac.h"
#include "PinNames.h"
#include "NanostackRfPhyAT86RF215.h"

#if defined(MBED_CONF_NANOSTACK_CONFIGURATION) && DEVICE_SPI && DEVICE_I2C && defined(MBED_CONF_RTOS_PRESENT)

#include "NanostackRfPhy.h"

// Uncomment to use testing gpios attached to TX/RX processes
 #define TEST_GPIOS_ENABLED

// Arduino pin defaults for convenience
#if !defined(ATMEL_SPI_MOSI)
#define ATMEL_SPI_MOSI   D11
#endif
#if !defined(ATMEL_SPI_MISO)
#define ATMEL_SPI_MISO   D12
#endif
#if !defined(ATMEL_SPI_SCLK)
#define ATMEL_SPI_SCLK   D13
#endif
#if !defined(ATMEL_SPI_CS)
#define ATMEL_SPI_CS     D10
#endif
#if !defined(ATMEL_SPI_RST)
#define ATMEL_SPI_RST    D5
#endif
#if !defined(ATMEL_SPI_SLP)
#define ATMEL_SPI_SLP    D7
#endif
#if !defined(ATMEL_SPI_IRQ)
#define ATMEL_SPI_IRQ    D9
#endif
#if !defined(ATMEL_I2C_SDA)
#define ATMEL_I2C_SDA    D14
#endif
#if !defined(ATMEL_I2C_SCL)
#define ATMEL_I2C_SCL    D15
#endif
#if !defined(ATMEL_SPI_TEST1)
#define ATMEL_SPI_TEST1    D6
#endif
#if !defined(ATMEL_SPI_TEST2)
#define ATMEL_SPI_TEST2    D3
#endif
#if !defined(ATMEL_SPI_TEST3)
#define ATMEL_SPI_TEST3    D4
#endif
#if !defined(ATMEL_SPI_TEST4)
#define ATMEL_SPI_TEST4    D2
#endif
#if !defined(ATMEL_SPI_TEST5)
#define ATMEL_SPI_TEST5    D8
#endif

class RFBits;

class NanostackRfPhyAtmel : public NanostackRfPhy {
public:
    NanostackRfPhyAtmel(PinName spi_mosi, PinName spi_miso,
                        PinName spi_sclk, PinName spi_cs,  PinName spi_rst, PinName spi_slp, PinName spi_irq,
                        PinName i2c_sda, PinName i2c_scl
#ifdef TEST_GPIOS_ENABLED
                       ,PinName spi_test1, PinName spi_test2, PinName spi_test3, PinName spi_test4, PinName spi_test5
#endif //TEST_GPIOS_ENABLED
                       );
    virtual ~NanostackRfPhyAtmel();
    virtual int8_t rf_register();
    virtual void rf_unregister();
    virtual void get_mac_address(uint8_t *mac);
    virtual void set_mac_address(uint8_t *mac);

private:
    AT24Mac _mac;
    uint8_t _mac_addr[8];
    RFBits *_rf;
    bool _mac_set;

    const PinName _spi_mosi;
    const PinName _spi_miso;
    const PinName _spi_sclk;
    const PinName _spi_cs;
    const PinName _spi_rst;
    const PinName _spi_slp;
    const PinName _spi_irq;
#ifdef TEST_GPIOS_ENABLED
    const PinName _spi_test1;
    const PinName _spi_test2;
    const PinName _spi_test3;
    const PinName _spi_test4;
    const PinName _spi_test5;
#endif //TEST_GPIOS_ENABLED
};

#ifdef TEST_GPIOS_ENABLED
#define TEST_TX_STARTED     rf->TEST1 = 1;
#define TEST_TX_DONE        rf->TEST1 = 0;
#define TEST_RX_STARTED     rf->TEST2 = 1;
#define TEST_RX_DONE        rf->TEST2 = 0;
#define TEST_ACK_TX_STARTED rf->TEST3 = 1;
#define TEST_ACK_TX_DONE    rf->TEST3 = 0;
#define TEST1_ON            rf->TEST4 = 1;
#define TEST1_OFF           rf->TEST4 = 0;
#define TEST2_ON            rf->TEST5 = 1;
#define TEST2_OFF           rf->TEST5 = 0;
extern void (*fhss_uc_switch)(void);
extern void (*fhss_bc_switch)(void);
#else //TEST_GPIOS_ENABLED
#define TEST_TX_STARTED
#define TEST_TX_DONE
#define TEST_RX_STARTED
#define TEST_RX_DONE
#define TEST_ACK_TX_STARTED
#define TEST_ACK_TX_DONE
#define TEST1_ON
#define TEST1_OFF
#define TEST2_ON
#define TEST2_OFF
#endif //TEST_GPIOS_ENABLED


#endif /* MBED_CONF_NANOSTACK_CONFIGURATION */
#endif /* NANOSTACK_RF_PHY_ATMEL_H_ */
