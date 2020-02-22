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

#ifndef AT86RF215REG_H_
#define AT86RF215REG_H_
#ifdef __cplusplus
extern "C" {
#endif


/*Register addresses*/
#define RF09_IRQS                   0x0000
#define RF24_IRQS                   0x01
#define BBC0_IRQS                   0x0002
#define BBC1_IRQS                   0x0003
#define RF_CFG                      0x0006
#define RF_IQIFC1                   0x0B
#define RF_PN                       0x000D
#define RF_VN                       0x000E
#define RF09_IRQM                   0x0100
#define RF_IRQM                     0x00
#define RF_STATE                    0x02
#define RF_CMD                      0x03
#define RF_CCF0L                    0x05
#define RF_CCF0H                    0x06
#define RF_CNL                      0x07
#define RF_CNM                      0x08
#define RF_EDC                      0x0E
#define BBC0_IRQM                   0x0300
#define BBC1_IRQM                   0x0400

// RF_IQIFC1
#define CHPM                        0x70
#define RF_MODE_BBRF                (0 << 4)
#define RF_MODE_RF                  (1 << 4)
#define RF_MODE_BBRF09              (4 << 4)
#define RF_MODE_BBRF24              (5 << 4)

/*RF_CFG bits*/
#define IRQMM                       0x08
#define IRQP                        0x04

/*RFn_IRQM bits*/
#define TRXRDY                      0x02
#define EDC                         0x04

/*RFn_EDC bits*/
#define EDM                         0x03
#define RF_EDAUTO                   (0 << 0)
#define RF_EDSINGLE                 (1 << 0)
#define RF_EDCONT                   (2 << 0)
#define RF_EDOFF                    (3 << 0)


/*Masks*/
#define CNH                         0x01
#define EDM                         0x03
#define CHPM                        0x70

#ifdef __cplusplus
}
#endif

#endif /* AT86RF215REG_H_ */
