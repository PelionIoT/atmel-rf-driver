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
#ifndef AT2MAC_H
#define AT24MAC_H

/*
 * AT24MAC drivers.
 *
 * This is a EEPROM chip designed to contain factory programmed read-only EUI-64 or EUI-48,
 * a 128bit serial number and some user probrammable EEPROM.
 *
 * AT24MAC602 contains EUI-64, use at24mac_read_eui64()
 * AT24MAC402 contains EUI-64, use at24mac_read_eui48()
 *
 * NOTE: You cannot use both EUI-64 and EUI-48. Chip contains only one of those.
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Read uniqua serial number from chip.
 * \param buf pointer to write serial number to. Must have space for 16 bytes.
 * \return zero on success, negative number on failure
 */
int at24mac_read_serial(void *buf);

/**
 * Read EUI-64 from chip.
 * \param buf pointer to write EUI-64 to. Must have space for 8 bytes.
 * \return zero on success, negative number on failure
 */
int at24mac_read_eui64(void *buf);
/**
 * Read EUI-48 from chip.
 * \param buf pointer to write EUI-48 to. Must have space for 6 bytes.
 * \return zero on success, negative number on failure
 */
int at24mac_read_eui48(void *buf);

#ifdef __cplusplus
}
#endif

#endif /* AT24MAC_H */
