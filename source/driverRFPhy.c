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
#include "ns_types.h"
#include "platform/arm_hal_interrupt.h"
#include "nanostack/platform/arm_hal_phy.h"
#include "atmel-rf-driver/driverRFPhy.h"
#include "driverAtmelRFInterface.h"
#include <string.h>
#include "randLIB.h"
#include "at24mac.h"

/*RF receive buffer*/
static uint8_t rf_buffer[RF_BUFFER_SIZE];
/*ACK wait duration changes depending on data rate*/
static uint16_t rf_ack_wait_duration = RF_ACK_WAIT_DEFAULT_TIMEOUT;

static int8_t rf_sensitivity = RF_DEFAULT_SENSITIVITY;
static uint8_t rf_mode = RF_MODE_NORMAL;
static uint8_t radio_tx_power = 0x00;   // Default to +4dBm
static uint8_t rf_phy_channel = 12;
static uint8_t rf_tuned = 1;
static uint8_t rf_use_antenna_diversity = 0;
static uint8_t tx_sequence = 0xff;
static uint8_t need_ack = 0;
static uint8_t rf_rx_mode = 0;
static uint8_t rf_flags = 0;
static uint8_t rf_rnd_rssi = 0;
static int8_t rf_radio_driver_id = -1;
static phy_device_driver_s device_driver;
static uint8_t atmel_MAC[8] = {1, 2, 3, 4, 5, 6, 7, 8};
static uint8_t mac_tx_handle = 0;
static int8_t rf_interface_state_control(phy_interface_state_e new_state, uint8_t rf_channel);
static int8_t rf_extension(phy_extension_type_e extension_type,uint8_t *data_ptr);
static int8_t rf_address_write(phy_address_type_e address_type,uint8_t *address_ptr);

/* Channel configurations for 2.4 and sub-GHz */
static const phy_rf_channel_configuration_s phy_24ghz = {2405000000, 5000000, 250000, 16, M_OQPSK};
static const phy_rf_channel_configuration_s phy_subghz = {868300000, 2000000, 250000, 11, M_OQPSK};

static const phy_device_channel_page_s phy_channel_pages[] = {
        { CHANNEL_PAGE_0, &phy_24ghz},
        { CHANNEL_PAGE_2, &phy_subghz},
        { CHANNEL_PAGE_0, NULL}
};

/*
 * \brief Function sets given RF flag on.
 *
 * \param x Given RF flag
 *
 * \return none
 */
void rf_flags_set(uint8_t x)
{
    rf_flags |= x;
}

/*
 * \brief Function clears given RF flag on.
 *
 * \param x Given RF flag
 *
 * \return none
 */
void rf_flags_clear(uint8_t x)
{
    rf_flags &= ~x;
}

/*
 * \brief Function checks if given RF flag is on.
 *
 * \param x Given RF flag
 *
 * \return states of the given flags
 */
uint8_t rf_flags_check(uint8_t x)
{
    return (rf_flags & x);
}

/*
 * \brief Function clears all RF flags.
 *
 * \param none
 *
 * \return none
 */
void rf_flags_reset(void)
{
    rf_flags = 0;
}

/*
 * \brief Function initialises and registers the RF driver.
 *
 * \param none
 *
 * \return rf_radio_driver_id Driver ID given by NET library
 */
int8_t rf_device_register(void)
{
    rf_trx_part_e radio_type;

    if (0 != at24mac_read_eui64(atmel_MAC))
        return -1; //No MAC

    rf_init();

    radio_type = rf_radio_type_read();
    if(radio_type != ATMEL_UNKNOW_DEV)
    {
        /*Set pointer to MAC address*/
        device_driver.PHY_MAC = atmel_MAC;
        device_driver.driver_description = "ATMEL_MAC";
        //Create setup Used Radio chips
        if(radio_type == ATMEL_AT86RF212)
        {
            device_driver.link_type = PHY_LINK_15_4_SUBGHZ_TYPE;
        }
        else
        {
            device_driver.link_type = PHY_LINK_15_4_2_4GHZ_TYPE;
        }
        device_driver.phy_channel_pages = phy_channel_pages;
        /*Maximum size of payload is 127*/
        device_driver.phy_MTU = 127;
        /*No header in PHY*/
        device_driver.phy_header_length = 0;
        /*No tail in PHY*/
        device_driver.phy_tail_length = 0;
        /*Set address write function*/
        device_driver.address_write = &rf_address_write;
        /*Set RF extension function*/
        device_driver.extension = &rf_extension;
        /*Set RF state control function*/
        device_driver.state_control = &rf_interface_state_control;
        /*Set transmit function*/
        device_driver.tx = &rf_start_cca;
        /*NULLIFY rx and tx_done callbacks*/
        device_driver.phy_rx_cb = NULL;
        device_driver.phy_tx_done_cb = NULL;
        /*Register device driver*/
        rf_radio_driver_id = arm_net_phy_register(&device_driver);
    }
    return rf_radio_driver_id;
}

/*
 * \brief Function returns the generated 8-bit random value for seeding Pseudo-random generator. This value was generated by reading noise from RF channel in RF initialisation.
 *
 * \param none
 *
 * \return random RSSI value
 */
int8_t rf_read_random(void)
{
    return rf_rnd_rssi;
}

/*
 * \brief Function is a call back for ACK wait timeout.
 *
 * \param none
 *
 * \return none
 */
void rf_ack_wait_timer_interrupt(void)
{
    platform_enter_critical();
    /*Force PLL state*/
    rf_if_change_trx_state(FORCE_PLL_ON);
    rf_poll_trx_state_change(PLL_ON);
    /*Start receiver in RX_AACK_ON state*/
    rf_rx_mode = 0;
    rf_flags_clear(RFF_RX);
    rf_receive();
    platform_exit_critical();
}

/*
 * \brief Function is a call back for calibration interval timer.
 *
 * \param none
 *
 * \return none
 */
void rf_calibration_timer_interrupt(void)
{
    /*Calibrate RF*/
    rf_calibration_cb();
    /*Start new calibration timeout*/
    rf_calibration_timer_start(RF_CALIBRATION_INTERVAL);
}

/*
 * \brief Function is a call back for cca interval timer.
 *
 * \param none
 *
 * \return none
 */
void rf_cca_timer_interrupt(void)
{
    /*Start CCA process*/
    if(rf_if_read_trx_state() == BUSY_RX_AACK)
    {
        if(device_driver.phy_tx_done_cb){
            device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_CCA_FAIL, 1, 1);
        }
    }
    else
    {
        /*Set radio in RX state to read channel*/
        rf_receive();
        rf_if_enable_cca_ed_done_interrupt();
        rf_if_start_cca_process();
    }
}


/*
 * \brief Function starts the ACK wait timeout.
 *
 * \param slots Given slots, resolution 50us
 *
 * \return none
 */
void rf_ack_wait_timer_start(uint16_t slots)
{
    rf_if_ack_wait_timer_start(slots);
}

/*
 * \brief Function starts the calibration interval.
 *
 * \param slots Given slots, resolution 50us
 *
 * \return none
 */
void rf_calibration_timer_start(uint32_t slots)
{
    rf_if_calibration_timer_start(slots);
}

/*
 * \brief Function starts the CCA timout.
 *
 * \param slots Given slots, resolution 50us
 *
 * \return none
 */
void rf_cca_timer_start(uint32_t slots)
{
    rf_if_cca_timer_start(slots);
}

/*
 * \brief Function stops the ACK wait timeout.
 *
 * \param none
 *
 * \return none
 */
void rf_ack_wait_timer_stop(void)
{
    rf_if_ack_wait_timer_stop();
}

/*
 * \brief Function reads the MAC address array.
 *
 * \param ptr Pointer to read array
 *
 * \return none
 */
void rf_read_mac_address(uint8_t *ptr)
{
    memcpy(ptr, atmel_MAC, 8);
}

/*
 * \brief Function sets the MAC address array.
 *
 * \param ptr Pointer to given MAC address array
 *
 * \return none
 */
void rf_set_mac_address(const uint8_t *ptr)
{
    memcpy(atmel_MAC,ptr,8);
}

uint16_t rf_get_phy_mtu_size(void)
{
    return device_driver.phy_MTU;
}

/*
 * \brief Function writes various RF settings in startup.
 *
 * \param none
 *
 * \return none
 */
void rf_write_settings(void)
{
    platform_enter_critical();
    rf_if_write_rf_settings();
    /*Set output power*/
    rf_if_write_set_tx_power_register(radio_tx_power);
    /*Initialise Antenna Diversity*/
    if(rf_use_antenna_diversity)
        rf_if_write_antenna_diversity_settings();
    platform_exit_critical();
}

/*
 * \brief Function writes 16-bit address in RF address filter.
 *
 * \param short_address Given short address
 *
 * \return none
 */
void rf_set_short_adr(uint8_t * short_address)
{
    uint8_t rf_off_flag = 0;
    platform_enter_critical();
    /*Wake up RF if sleeping*/
    uint8_t state = rf_if_read_trx_state();
    if(state == 0x00 || state == 0x1F)
    {
        rf_if_disable_slptr();
        rf_off_flag = 1;
        rf_poll_trx_state_change(TRX_OFF);
    }
    /*Write address filter registers*/
    rf_if_write_short_addr_registers(short_address);
    /*RF back to sleep*/
    if(rf_off_flag)
        rf_if_enable_slptr();
    platform_exit_critical();
}

/*
 * \brief Function writes PAN Id in RF PAN Id filter.
 *
 * \param pan_id Given PAN Id
 *
 * \return none
 */
void rf_set_pan_id(uint8_t *pan_id)
{
    uint8_t rf_off_flag = 0;

    platform_enter_critical();
    uint8_t state = rf_if_read_trx_state();
    /*Wake up RF if sleeping*/
    if(state == 0x00 || state == 0x1F)
    {
        rf_if_disable_slptr();
        rf_off_flag = 1;
        rf_poll_trx_state_change(TRX_OFF);
    }
    /*Write address filter registers*/
    rf_if_write_pan_id_registers(pan_id);
    /*RF back to sleep*/
    if(rf_off_flag)
        rf_if_enable_slptr();
    platform_exit_critical();
}

/*
 * \brief Function writes 64-bit address in RF address filter.
 *
 * \param address Given 64-bit address
 *
 * \return none
 */
void rf_set_address(uint8_t *address)
{
    uint8_t rf_off_flag = 0;

    platform_enter_critical();
    /*Wake up RF if sleeping*/
    uint8_t state = rf_if_read_trx_state();
    if(state == 0x00 || state == 0x1F)
    {
        rf_if_disable_slptr();
        rf_off_flag = 1;
        rf_poll_trx_state_change(TRX_OFF);
    }
    /*Write address filter registers*/
    rf_if_write_ieee_addr_registers(address);
    /*RF back to sleep*/
    if(rf_off_flag)
        rf_if_enable_slptr();

    platform_exit_critical();
}

/*
 * \brief Function sets the RF channel.
 *
 * \param ch New channel
 *
 * \return none
 */
void rf_channel_set(uint8_t ch)
{
    platform_enter_critical();
    rf_phy_channel = ch;
    if(ch < 0x1f)
        rf_if_set_channel_register(ch);
    platform_exit_critical();
}


/*
 * \brief Function initialises the radio driver and resets the radio.
 *
 * \param none
 *
 * \return none
 */
void rf_init(void)
{
    /*Reset RF module*/
    rf_if_reset_radio();
    /*Write RF settings*/
    rf_write_settings();
    /*Initialise PHY mode*/
    rf_init_phy_mode();
    /*Clear RF flags*/
    rf_flags_reset();
    /*Set RF in TRX OFF state*/
    rf_if_change_trx_state(TRX_OFF);
    /*Set RF in PLL_ON state*/
    rf_if_change_trx_state(PLL_ON);
    /*Start receiver*/
    rf_receive();
    /*Read random variable. This will be used when seeding pseudo-random generator*/
    rf_rnd_rssi = rf_if_read_rnd();
    /*Start RF calibration timer*/
    rf_calibration_timer_start(RF_CALIBRATION_INTERVAL);
}

/**
 * \brief Function gets called when MAC is setting radio off.
 *
 * \param none
 *
 * \return none
 */
void rf_off(void)
{
    if(rf_flags_check(RFF_ON))
    {
        rf_cca_abort();
        uint16_t while_counter = 0;
        /*Wait while receiving*/
        while(rf_if_read_trx_state() == BUSY_RX_AACK)
        {
            while_counter++;
            if(while_counter == 0xffff)
                break;
        }
        /*RF state change: RX_AACK_ON->PLL_ON->TRX_OFF->SLEEP*/
        if(rf_if_read_trx_state() == RX_AACK_ON)
        {
            rf_if_change_trx_state(PLL_ON);
        }
        rf_if_change_trx_state(TRX_OFF);
        rf_if_enable_slptr();
        rf_flags_clear(~RFF_ON);
        /*Disable Antenna Diversity*/
        if(rf_use_antenna_diversity)
            rf_if_disable_ant_div();
    }
}

/*
 * \brief Function polls the RF state until it has changed to desired state.
 *
 * \param trx_state RF state
 *
 * \return none
 */
void rf_poll_trx_state_change(rf_trx_states_t trx_state)
{
    uint16_t while_counter = 0;
    platform_enter_critical();

    if(trx_state != RF_TX_START)
    {
        if(trx_state == FORCE_PLL_ON)
            trx_state = PLL_ON;
        else if(trx_state == FORCE_TRX_OFF)
            trx_state = TRX_OFF;

        while(rf_if_read_trx_state() != trx_state)
        {
            while_counter++;
            if(while_counter == 0x1ff)
                break;
        }
    }
    platform_exit_critical();
}

/*
 * \brief Function starts the CCA process before starting data transmission and copies the data to RF TX FIFO.
 *
 * \param data_ptr Pointer to TX data
 * \param data_length Length of the TX data
 * \param tx_handle Handle to transmission
 * \return 0 Success
 * \return -1 Busy
 */
int8_t rf_start_cca(uint8_t *data_ptr, uint16_t data_length, uint8_t tx_handle, data_protocol_e data_protocol )
{
    (void)data_protocol;
    /*Check if transmitter is busy*/
    if(rf_if_read_trx_state() == BUSY_RX_AACK)
    {
        /*Return busy*/
        return -1;
    }
    else
    {
        platform_enter_critical();
        /*Check if transmitted data needs to be acked*/
        if(*data_ptr & 0x20)
            need_ack = 1;
        else
            need_ack = 0;
        /*Store the sequence number for ACK handling*/
        tx_sequence = *(data_ptr + 2);

        /*Write TX FIFO*/
        rf_if_write_frame_buffer(data_ptr, (uint8_t)data_length);
        rf_flags_set(RFF_CCA);
        /*Start CCA timeout*/
        rf_cca_timer_start(RF_CCA_BASE_BACKOFF + randLIB_get_random_in_range(0, RF_CCA_RANDOM_BACKOFF));
        /*Store TX handle*/
        mac_tx_handle = tx_handle;
        platform_exit_critical();
    }

    /*Return success*/
    return 0;
}

/*
 * \brief Function aborts CCA process.
 *
 * \param none
 *
 * \return none
 */
void rf_cca_abort(void)
{
    /*Clear RFF_CCA RF flag*/
    rf_flags_clear(RFF_CCA);
}



/*
 * \brief Function starts the transmission of the frame.
 *
 * \param none
 *
 * \return none
 */
void rf_start_tx(void)
{
    /*Only start transmitting from RX state*/
    uint8_t trx_state = rf_if_read_trx_state();
    if(trx_state != RX_AACK_ON)
    {
        if(device_driver.phy_tx_done_cb){
            device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_CCA_FAIL, 1, 1);
        }
    }
    else
    {
        /*RF state change: ->PLL_ON->RF_TX_START*/
        rf_if_change_trx_state(FORCE_PLL_ON);
        rf_flags_clear(RFF_RX);
        rf_if_enable_tx_end_interrupt();
        rf_flags_set(RFF_TX);
        rf_if_change_trx_state(RF_TX_START);
    }
}

/*
 * \brief Function sets the RF in RX state.
 *
 * \param none
 *
 * \return none
 */
void rf_receive(void)
{
    uint16_t while_counter = 0;
    if(rf_flags_check(RFF_ON) == 0)
    {
        rf_on();
    }
    /*If not yet in RX state set it*/
    if(rf_flags_check(RFF_RX) == 0)
    {
        platform_enter_critical();
        /*Wait while receiving data*/
        while(rf_if_read_trx_state() == BUSY_RX_AACK)
        {
            while_counter++;
            if(while_counter == 0xffff)
            {
                break;
            }
        }
        //TODO: rf_if_delay_function(50);
        /*Wake up from sleep state*/
        uint8_t state = rf_if_read_trx_state();
        if(state == 0x00 || state == 0x1f)
        {
            rf_if_disable_slptr();
            rf_poll_trx_state_change(TRX_OFF);
        }

        rf_if_change_trx_state(PLL_ON);

        if(rf_mode == RF_MODE_SNIFFER)
        {
            rf_if_change_trx_state(RX_ON);
        }
        else
        {
            /*ACK is always received in promiscuous mode to bypass address filters*/
            if(rf_rx_mode)
            {
                rf_rx_mode = 0;
                rf_if_enable_promiscuous_mode();
            }
            else
            {
                rf_if_disable_promiscuous_mode();
            }
            rf_if_change_trx_state(RX_AACK_ON);
        }
        /*If calibration timer was unable to calibrate the RF, run calibration now*/
        if(!rf_tuned)
        {
            /*Start calibration. This can be done in states TRX_OFF, PLL_ON or in any receive state*/
            rf_if_calibration();
            /*RF is tuned now*/
            rf_tuned = 1;
        }

        rf_channel_set(rf_phy_channel);
        rf_flags_set(RFF_RX);
        rf_if_enable_rx_end_interrupt();
        platform_exit_critical();
    }
    /*Stop the running CCA process*/
    if(rf_flags_check(RFF_CCA))
        rf_cca_abort();
}

/*
 * \brief Function calibrates the radio.
 *
 * \param none
 *
 * \return none
 */
void rf_calibration_cb(void)
{
    /*clear tuned flag to start tuning in rf_receive*/
    rf_tuned = 0;
    /*If RF is in default receive state, start calibration*/
    if(rf_if_read_trx_state() == RX_AACK_ON)
    {
        platform_enter_critical();
        /*Set RF in PLL_ON state*/
        rf_if_change_trx_state(PLL_ON);
        /*Set RF in TRX_OFF state to start PLL tuning*/
        rf_if_change_trx_state(TRX_OFF);
        /*Set RF in RX_ON state to calibrate*/
        rf_if_change_trx_state(RX_ON);
        /*Calibrate FTN*/
        rf_if_calibration();
        /*RF is tuned now*/
        rf_tuned = 1;
        /*Back to default receive state*/
        rf_flags_clear(RFF_RX);
        rf_receive();
        platform_exit_critical();
    }
}

/*
 * \brief Function sets RF_ON flag when radio is powered.
 *
 * \param none
 *
 * \return none
 */
void rf_on(void)
{
    /*Set RFF_ON flag*/
    if(rf_flags_check(RFF_ON) == 0)
    {
        rf_flags_set(RFF_ON);
        /*Enable Antenna diversity*/
        if(rf_use_antenna_diversity)
            /*Set ANT_EXT_SW_EN to enable controlling of antenna diversity*/
            rf_if_enable_ant_div();
    }
}

/*
 * \brief Function handles the received ACK frame.
 *
 * \param seq_number Sequence number of received ACK
 * \param data_pending Pending bit state in received ACK
 *
 * \return none
 */
void rf_handle_ack(uint8_t seq_number, uint8_t data_pending)
{
    phy_link_tx_status_e phy_status;
    platform_enter_critical();
    /*Received ACK sequence must be equal with transmitted packet sequence*/
    if(tx_sequence == seq_number)
    {
        rf_ack_wait_timer_stop();
        /*When data pending bit in ACK frame is set, inform NET library*/
        if(data_pending)
            phy_status = PHY_LINK_TX_DONE_PENDING;
        else
            phy_status = PHY_LINK_TX_DONE;
        /*Call PHY TX Done API*/
        if(device_driver.phy_tx_done_cb){
            device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle,phy_status, 1, 1);
        }
    }
    platform_exit_critical();
}

/*
 * \brief Function is a call back for RX end interrupt.
 *
 * \param none
 *
 * \return none
 */
void rf_handle_rx_end(void)
{
    uint8_t rf_lqi = 0;
    int8_t rf_rssi = 0;

    /*Start receiver*/
    rf_flags_clear(RFF_RX);
    rf_receive();

    /*Frame received interrupt*/
    if(rf_flags_check(RFF_RX))
    {
        /*Check CRC_valid bit*/
        if(rf_if_check_crc())
        {
            uint8_t *rf_rx_ptr;
            uint8_t receiving_ack = 0;
            /*Read length*/
            uint8_t len = rf_if_read_received_frame_length();

            rf_rx_ptr = rf_buffer;
            /*ACK frame*/
            if(len == 5)
            {
                /*Read ACK in static ACK buffer*/
                receiving_ack = 1;
            }
            /*Check the length is valid*/
            if(len > 1 && len < RF_BUFFER_SIZE)
            {
                /*Read received packet*/
                rf_if_read_packet(rf_rx_ptr, len);

                if(!receiving_ack)
                {
                    rf_rssi = rf_if_read_rssi();
                    /*Scale LQI using received RSSI*/
                    rf_lqi = rf_scale_lqi(rf_rssi);
                }
                if(rf_mode == RF_MODE_SNIFFER)
                {
                    if( device_driver.phy_rx_cb ){
                        device_driver.phy_rx_cb(rf_buffer,len - 2, rf_lqi, rf_rssi, rf_radio_driver_id);
                    }
                }
                else
                {
                    /*Handle received ACK*/
                    if(receiving_ack && ((rf_buffer[0] & 0x07) == 0x02))
                    {
                        uint8_t pending = 0;
                        /*Check if data is pending*/
                        if ((rf_buffer[0] & 0x10))
                        {
                            pending=1;
                        }
                        /*Send sequence number in ACK handler*/
                        rf_handle_ack(rf_buffer[2], pending);
                    }
                    else
                    {
                        if( device_driver.phy_rx_cb ){
                            device_driver.phy_rx_cb(rf_buffer,len - 2, rf_lqi, rf_rssi, rf_radio_driver_id);
                        }
                    }
                }
            }
        }
    }
}

/*
 * \brief Function is called when MAC is shutting down the radio.
 *
 * \param none
 *
 * \return none
 */
void rf_shutdown(void)
{
    /*Call RF OFF*/
    rf_off();
    /*Clear RF flags*/
    rf_flags_reset();
}

/*
 * \brief Function is a call back for TX end interrupt.
 *
 * \param none
 *
 * \return none
 */
void rf_handle_tx_end(void)
{
    phy_link_tx_status_e phy_status = PHY_LINK_TX_SUCCESS;

    rf_rx_mode = 0;
    /*If ACK is needed for this transmission*/
    if(need_ack && rf_flags_check(RFF_TX))
    {
        rf_ack_wait_timer_start(rf_ack_wait_duration);
        rf_rx_mode = 1;
    }
    rf_flags_clear(RFF_RX);
    /*Start receiver*/
    rf_receive();

    /*Call PHY TX Done API*/
    if(device_driver.phy_tx_done_cb){
        device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, phy_status, 1, 1);
    }
}

/*
 * \brief Function is a call back for CCA ED done interrupt.
 *
 * \param none
 *
 * \return none
 */
void rf_handle_cca_ed_done(void)
{
    rf_flags_clear(RFF_CCA);
    /*Check the result of CCA process*/
    if(rf_if_check_cca())
    {
        rf_start_tx();
    }
    else
    {
        /*Send CCA fail notification*/
        if(device_driver.phy_tx_done_cb){
            device_driver.phy_tx_done_cb(rf_radio_driver_id, mac_tx_handle, PHY_LINK_CCA_FAIL, 1, 1);
        }
    }
}

/*
 * \brief Function sets the TX power variable.
 *
 * \param power TX power setting
 *
 * \return 0 Success
 * \return -1 Fail
 */
int8_t rf_tx_power_set(uint8_t power)
{
    int8_t ret_val = -1;

    radio_tx_power = power;
    rf_if_write_set_tx_power_register(radio_tx_power);
    ret_val = 0;

    return ret_val;
}

/*
 * \brief Function returns the TX power variable.
 *
 * \param none
 *
 * \return radio_tx_power TX power variable
 */
uint8_t rf_tx_power_get(void)
{
  return radio_tx_power;
}

/*
 * \brief Function enables the usage of Antenna diversity.
 *
 * \param none
 *
 * \return 0 Success
 */
int8_t rf_enable_antenna_diversity(void)
{
    int8_t ret_val = 0;
    rf_use_antenna_diversity = 1;
    return ret_val;
}

/*
 * \brief Function gives the control of RF states to MAC.
 *
 * \param new_state RF state
 * \param rf_channel RF channel
 *
 * \return 0 Success
 */
static int8_t rf_interface_state_control(phy_interface_state_e new_state, uint8_t rf_channel)
{
    int8_t ret_val = 0;
    switch (new_state)
    {
        /*Reset PHY driver and set to idle*/
        case PHY_INTERFACE_RESET:
            break;
        /*Disable PHY Interface driver*/
        case PHY_INTERFACE_DOWN:
            rf_shutdown();
            break;
        /*Enable PHY Interface driver*/
        case PHY_INTERFACE_UP:
            rf_channel_set(rf_channel);
            rf_receive();
            break;
        /*Enable wireless interface ED scan mode*/
        case PHY_INTERFACE_RX_ENERGY_STATE:
            break;
        case PHY_INTERFACE_SNIFFER_STATE:             /**< Enable Sniffer state */
            rf_mode = RF_MODE_SNIFFER;
            rf_channel_set(rf_channel);
            rf_flags_clear(RFF_RX);
            rf_receive();
            break;
    }
    return ret_val;
}

/*
 * \brief Function controls the ACK pending, channel setting and energy detection.
 *
 * \param extension_type Type of control
 * \param data_ptr Data from NET library
 *
 * \return 0 Success
 */
static int8_t rf_extension(phy_extension_type_e extension_type, uint8_t *data_ptr)
{
    switch (extension_type)
    {
        /*Control MAC pending bit for Indirect data transmission*/
        case PHY_EXTENSION_CTRL_PENDING_BIT:
            if(*data_ptr)
            {
                rf_if_ack_pending_ctrl(1);
            }
            else
            {
                rf_if_ack_pending_ctrl(0);
            }
            break;
        /*Return frame pending status*/
        case PHY_EXTENSION_READ_LAST_ACK_PENDING_STATUS:
            *data_ptr = rf_if_last_acked_pending();
            break;
        /*Set channel*/
        case PHY_EXTENSION_SET_CHANNEL:
            break;
        /*Read energy on the channel*/
        case PHY_EXTENSION_READ_CHANNEL_ENERGY:
            break;
        /*Read status of the link*/
        case PHY_EXTENSION_READ_LINK_STATUS:
            break;
        default:
            break;
    }
    return 0;
}

/*
 * \brief Function sets the addresses to RF address filters.
 *
 * \param address_type Type of address
 * \param address_ptr Pointer to given address
 *
 * \return 0 Success
 */
static int8_t rf_address_write(phy_address_type_e address_type, uint8_t *address_ptr)
{
    int8_t ret_val = 0;
    switch (address_type)
    {
        /*Set 48-bit address*/
        case PHY_MAC_48BIT:
            break;
            /*Set 64-bit address*/
        case PHY_MAC_64BIT:
            rf_set_address(address_ptr);
            break;
        /*Set 16-bit address*/
        case PHY_MAC_16BIT:
            rf_set_short_adr(address_ptr);
            break;
        /*Set PAN Id*/
        case PHY_MAC_PANID:
            rf_set_pan_id(address_ptr);
            break;
    }
    return ret_val;
}

/*
 * \brief Function initialises the ACK wait time and returns the used PHY mode.
 *
 * \param none
 *
 * \return tmp Used PHY mode
 */
void rf_init_phy_mode(void)
{
    uint8_t tmp = 0;
    uint8_t part = rf_if_read_part_num();
    /*Read used PHY Mode*/
    tmp = rf_if_read_register(TRX_CTRL_2);
    /*Set ACK wait time for used data rate*/
    if(part == PART_AT86RF212)
    {
        if((tmp & 0x1f) == 0x00)
        {
            rf_sensitivity = -110;
            rf_ack_wait_duration = 938;
            tmp = BPSK_20;
        }
        else if((tmp & 0x1f) == 0x04)
        {
            rf_sensitivity = -108;
            rf_ack_wait_duration = 469;
            tmp = BPSK_40;
        }
        else if((tmp & 0x1f) == 0x14)
        {
            rf_sensitivity = -108;
            rf_ack_wait_duration = 469;
            tmp = BPSK_40_ALT;
        }
        else if((tmp & 0x1f) == 0x08)
        {
            rf_sensitivity = -101;
            rf_ack_wait_duration = 50;
            tmp = OQPSK_SIN_RC_100;
        }
        else if((tmp & 0x1f) == 0x09)
        {
            rf_sensitivity = -99;
            rf_ack_wait_duration = 30;
            tmp = OQPSK_SIN_RC_200;
        }
        else if((tmp & 0x1f) == 0x18)
        {
            rf_sensitivity = -102;
            rf_ack_wait_duration = 50;
            tmp = OQPSK_RC_100;
        }
        else if((tmp & 0x1f) == 0x19)
        {
            rf_sensitivity = -100;
            rf_ack_wait_duration = 30;
            tmp = OQPSK_RC_200;
        }
        else if((tmp & 0x1f) == 0x0c)
        {
            rf_sensitivity = -100;
            rf_ack_wait_duration = 20;
            tmp = OQPSK_SIN_250;
        }
        else if((tmp & 0x1f) == 0x0d)
        {
            rf_sensitivity = -98;
            rf_ack_wait_duration = 25;
            tmp = OQPSK_SIN_500;
        }
        else if((tmp & 0x1f) == 0x0f)
        {
            rf_sensitivity = -98;
            rf_ack_wait_duration = 25;
            tmp = OQPSK_SIN_500_ALT;
        }
        else if((tmp & 0x1f) == 0x1c)
        {
            rf_sensitivity = -101;
            rf_ack_wait_duration = 20;
            tmp = OQPSK_RC_250;
        }
        else if((tmp & 0x1f) == 0x1d)
        {
            rf_sensitivity = -99;
            rf_ack_wait_duration = 25;
            tmp = OQPSK_RC_500;
        }
        else if((tmp & 0x1f) == 0x1f)
        {
            rf_sensitivity = -99;
            rf_ack_wait_duration = 25;
            tmp = OQPSK_RC_500_ALT;
        }
        else if((tmp & 0x3f) == 0x2A)
        {
            rf_sensitivity = -91;
            rf_ack_wait_duration = 25;
            tmp = OQPSK_SIN_RC_400_SCR_ON;
        }
        else if((tmp & 0x3f) == 0x0A)
        {
            rf_sensitivity = -91;
            rf_ack_wait_duration = 25;
            tmp = OQPSK_SIN_RC_400_SCR_OFF;
        }
        else if((tmp & 0x3f) == 0x3A)
        {
            rf_sensitivity = -97;
            rf_ack_wait_duration = 25;
            tmp = OQPSK_RC_400_SCR_ON;
        }
        else if((tmp & 0x3f) == 0x1A)
        {
            rf_sensitivity = -97;
            rf_ack_wait_duration = 25;
            tmp = OQPSK_RC_400_SCR_OFF;
        }
        else if((tmp & 0x3f) == 0x2E)
        {
            rf_sensitivity = -93;
            rf_ack_wait_duration = 13;
            tmp = OQPSK_SIN_1000_SCR_ON;
        }
        else if((tmp & 0x3f) == 0x0E)
        {
            rf_sensitivity = -93;
            rf_ack_wait_duration = 13;
            tmp = OQPSK_SIN_1000_SCR_OFF;
        }
        else if((tmp & 0x3f) == 0x3E)
        {
            rf_sensitivity = -95;
            rf_ack_wait_duration = 13;
            tmp = OQPSK_RC_1000_SCR_ON;
        }
        else if((tmp & 0x3f) == 0x1E)
        {
            rf_sensitivity = -95;
            rf_ack_wait_duration = 13;
            tmp = OQPSK_RC_1000_SCR_OFF;
        }
    }
    else
    {
        rf_sensitivity = -101;
        rf_ack_wait_duration = 20;
    }
    /*Board design might reduces the sensitivity*/
    //rf_sensitivity += RF_SENSITIVITY_CALIBRATION;
}


uint8_t rf_scale_lqi(int8_t rssi)
{
    uint8_t scaled_lqi;

    /*rssi < RF sensitivity*/
    if(rssi < rf_sensitivity)
        scaled_lqi=0;
    /*-91 dBm < rssi < -81 dBm (AT86RF233 XPro)*/
    /*-90 dBm < rssi < -80 dBm (AT86RF212B XPro)*/
    else if(rssi < (rf_sensitivity + 10))
        scaled_lqi=31;
    /*-81 dBm < rssi < -71 dBm (AT86RF233 XPro)*/
    /*-80 dBm < rssi < -70 dBm (AT86RF212B XPro)*/
    else if(rssi < (rf_sensitivity + 20))
        scaled_lqi=207;
    /*-71 dBm < rssi < -61 dBm (AT86RF233 XPro)*/
    /*-70 dBm < rssi < -60 dBm (AT86RF212B XPro)*/
    else if(rssi < (rf_sensitivity + 30))
        scaled_lqi=255;
    /*-61 dBm < rssi < -51 dBm (AT86RF233 XPro)*/
    /*-60 dBm < rssi < -50 dBm (AT86RF212B XPro)*/
    else if(rssi < (rf_sensitivity + 40))
        scaled_lqi=255;
    /*-51 dBm < rssi < -41 dBm (AT86RF233 XPro)*/
    /*-50 dBm < rssi < -40 dBm (AT86RF212B XPro)*/
    else if(rssi < (rf_sensitivity + 50))
        scaled_lqi=255;
    /*-41 dBm < rssi < -31 dBm (AT86RF233 XPro)*/
    /*-40 dBm < rssi < -30 dBm (AT86RF212B XPro)*/
    else if(rssi < (rf_sensitivity + 60))
        scaled_lqi=255;
    /*-31 dBm < rssi < -21 dBm (AT86RF233 XPro)*/
    /*-30 dBm < rssi < -20 dBm (AT86RF212B XPro)*/
    else if(rssi < (rf_sensitivity + 70))
        scaled_lqi=255;
    /*rssi > RF saturation*/
    else if(rssi > (rf_sensitivity + 80))
        scaled_lqi=111;
    /*-21 dBm < rssi < -11 dBm (AT86RF233 XPro)*/
    /*-20 dBm < rssi < -10 dBm (AT86RF212B XPro)*/
    else
        scaled_lqi=255;

    return scaled_lqi;
}
