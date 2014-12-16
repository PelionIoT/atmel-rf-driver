/*
 * Copyright (c) 2014 ARM. All rights reserved.
 */
/*
 * driverAtmelRFInterface.h
 *
 *  Created on: 14 July 2014
 *      Author: mBed Team
 */

#ifndef DRIVERRFINTERFACE_H_
#define DRIVERRFINTERFACE_H_
#ifdef __cplusplus
extern "C" {
#endif

/*Delay between transfers(bytes) (32*DLYBCT)/MCK -> (32*6/120MHz=1.6us)*/
#define SPI_DLYBCT	6
/*Delay before SPCK DLYBS/MCK -> 140/120MHz=1.16us)*/
#define SPI_DLYBS	140
/*Serial clock baud rate MCK/SCBR -> 120MHz/18=6.7MHz)*/
#define SPI_SCBR	18

#define PHY_ACK_WAIT_TICK_VAL 185
#define PHY_CALIBRATION_TICK_VAL 185
#define PHY_ACK_WAIT_TIMER	1
#define PHY_CALIBRATION_TIMER	2
#define PHY_CCA_TIMER	3

/*Supported transceivers*/
#define PART_AT86RF231 		0x03
#define PART_AT86RF212 		0x07
#define PART_AT86RF233 		0x0B
#define VERSION_AT86RF212 	0x01
#define VERSION_AT86RF212B 	0x03

/*RF Configuration Registers*/
#define TRX_STATUS		0x01
#define TRX_STATE		0x02
#define TRX_CTRL_0		0x03
#define TRX_CTRL_1		0x04
#define PHY_TX_PWR		0x05
#define PHY_RSSI		0x06
#define PHY_ED_LEVEL	0x07
#define PHY_CC_CCA		0x08
#define RX_CTRL			0x0A
#define SFD_VALUE		0x0B
#define TRX_CTRL_2		0x0C
#define ANT_DIV			0x0D
#define IRQ_MASK		0x0E
#define IRQ_STATUS		0x0F
#define VREG_CTRL		0x10
#define BATMON			0x11
#define	XOSC_CTRL		0x12
#define CC_CTRL_0		0x13
#define CC_CTRL_1		0x14
#define RX_SYN			0x15
#define TRX_RPC			0x16
#define RF_CTRL_0		0x16
#define XAH_CTRL_1		0x17
#define FTN_CTRL		0x18
#define PLL_CF			0x1A
#define PLL_DCU			0x1B
#define PART_NUM		0x1C
#define VERSION_NUM		0x1D
#define MAN_ID_0		0x1E
#define MAN_ID_1		0x1F
#define SHORT_ADDR_0	0x20
#define SHORT_ADDR_1	0x21
#define PAN_ID_0		0x22
#define PAN_ID_1		0x23
#define IEEE_ADDR_0		0x24
#define IEEE_ADDR_1		0x25
#define IEEE_ADDR_2		0x26
#define IEEE_ADDR_3		0x27
#define IEEE_ADDR_4		0x28
#define IEEE_ADDR_5		0x29
#define IEEE_ADDR_6		0x2A
#define IEEE_ADDR_7		0x2B
#define XAH_CTRL_0		0x2C
#define CSMA_SEED_0 	0x2D
#define CSMA_SEED_1 	0x2E
#define CSMA_BE		 	0x2F

/* CSMA_SEED_1*/
#define AACK_FVN_MODE1 			7
#define AACK_FVN_MODE0			6
#define AACK_SET_PD 			5
#define AACK_DIS_ACK 			4
#define AACK_I_AM_COORD 		3
#define CSMA_SEED_12 			2
#define CSMA_SEED_11 			1
#define CSMA_SEED_10			0

/*TRX_STATUS bits*/
#define CCA_STATUS	0x40
#define CCA_DONE	0x80

/*PHY_CC_CCA bits*/
#define CCA_REQUEST	0x80
#define CCA_MODE_1 0x20
#define CCA_MODE_3 0x60

/*IRQ_MASK bits*/
#define RX_START 0x04
#define TRX_END 0x08
#define CCA_ED_DONE 0x10
#define AMI 0x20
#define TRX_UR 0x40

/*ANT_DIV bits*/
#define ANT_DIV_EN 		0x08
#define ANT_EXT_SW_EN	0x04
#define ANT_CTRL_DEFAULT	0x03

/*TRX_CTRL_1 bits*/
#define PA_EXT_EN 		0x80

/*FTN_CTRL bits*/
#define FTN_START 		0x80

/*PHY_RSSI bits*/
#define CRC_VALID	0x80

/*XAH_CTRL_1 bits*/
#define AACK_PROM_MODE	0x02

void rf_if_cca_timer_start(uint32_t slots);
void rf_if_enable_promiscuous_mode(void);


extern uint8_t rf_if_read_rnd(void);
extern void rf_if_calibration_timer_start(uint32_t slots);
extern void rf_if_interrupt_handler(void);
extern void (*rf_if_get_rf_interrupt_function())(void);
extern void rf_if_calibration_timer_interrupt(void);
extern void rf_if_timer_init(void);
extern void rf_if_ack_wait_timer_start(uint16_t slots);
extern void rf_if_ack_wait_timer_stop(void);
extern void rf_if_ack_wait_timer_interrupt(void);
extern int8_t rf_if_set_rf_irq_pin(uint8_t port, uint8_t pin);
extern int8_t rf_if_set_slp_tr_pin(uint8_t port, uint8_t pin);
extern int8_t rf_if_set_reset_pin(uint8_t port, uint8_t pin);
extern int8_t rf_if_set_spi_interface(uint8_t spi_interface, uint8_t cs_device);
extern void rf_if_ack_pending_ctrl(uint8_t state);
extern void rf_if_calibration(void);
extern uint8_t rf_if_read_register(uint8_t addr);
extern void rf_if_set_bit(uint8_t addr, uint8_t bit, uint8_t bit_mask);
extern void rf_if_clear_bit(uint8_t addr, uint8_t bit);
extern void rf_if_write_register(uint8_t addr, uint8_t data);
extern void rf_if_reset_radio(void);
extern void rf_if_enable_pa_ext(void);
extern void rf_if_disable_pa_ext(void);
extern void rf_if_enable_ant_div(void);
extern void rf_if_disable_ant_div(void);
extern void rf_if_enable_slptr(void);
extern void rf_if_disable_slptr(void);
extern void rf_if_write_antenna_diversity_settings(void);
extern void rf_if_write_set_tx_power_register(uint8_t value);
extern void rf_if_write_set_trx_rpc_register(uint8_t value);
extern void rf_if_write_rf_settings(void);
extern uint8_t rf_if_check_cca(void);
extern uint8_t rf_if_check_crc(void);
extern uint8_t rf_if_read_trx_state(void);
extern void rf_if_read_packet(uint8_t *ptr, uint8_t len);
extern void rf_if_write_short_addr_registers(uint8_t *short_address);
extern uint8_t rf_if_last_acked_pending(void);
extern void rf_if_write_pan_id_registers(uint8_t *pan_id);
extern void rf_if_write_ieee_addr_registers(uint8_t *address);
extern void rf_if_write_frame_buffer(uint8_t *ptr, uint8_t length);
extern void rf_if_change_trx_state(rf_trx_states_t trx_state);
extern void rf_if_enable_tx_end_interrupt(void);
extern void rf_if_enable_rx_end_interrupt(void);
extern void rf_if_enable_rx_start_interrupt(void);
extern void rf_if_enable_cca_ed_done_interrupt(void);
extern void rf_if_start_cca_process(void);
extern uint8_t rf_if_read_received_frame_length(void);
extern uint8_t rf_if_read_lqi(void);
extern int8_t rf_if_read_rssi(void);
extern void rf_if_set_channel_register(uint8_t channel);
void rf_if_enable_promiscuous_mode(void);
void rf_if_disable_promiscuous_mode(void);
uint8_t rf_if_read_part_num(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERRFINTERFACE_H_ */
