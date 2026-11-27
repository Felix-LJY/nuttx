/****************************************************************************
 * arch/arm/src/n32h7/hardware/n32h76x_i2c.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_N32H7_HARDWARE_N32H76X_I2C_H
#define __ARCH_ARM_SRC_N32H7_HARDWARE_N32H76X_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"
#include "hardware/n32h7_memorymap.h"

#if defined(CONFIG_N32H7_N32H76X)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The N32H7 I2C has the register layout of the ST I2C v2 IP (STM32 F0, F3,
 * F7, G0, G4, H7, L0, L4) under other names.  All registers are 32-bit.
 * Bit names are those of the vendor CMSIS device header.
 */

/* Register Offsets *********************************************************/
#define N32_I2C_CTRL1_OFFSET       0x0000  /* Control register 1 */
#define N32_I2C_CTRL2_OFFSET       0x0004  /* Control register 2 */
#define N32_I2C_ADR1_OFFSET        0x0008  /* Own address register 1 */
#define N32_I2C_ADR2_OFFSET        0x000c  /* Own address register 2 */
#define N32_I2C_BUSTM_OFFSET       0x0010  /* Bus timing register */
#define N32_I2C_TMOUTR_OFFSET      0x0014  /* Timeout register */
#define N32_I2C_STSINT_OFFSET      0x0018  /* Interrupt and status register */
#define N32_I2C_INTCLR_OFFSET      0x001c  /* Interrupt clear register */
#define N32_I2C_CRCR_OFFSET        0x0020  /* PEC register */
#define N32_I2C_RDR_OFFSET         0x0024  /* Receive data register */
#define N32_I2C_WDR_OFFSET         0x0028  /* Transmit data register */
#define N32_I2C_HSBUSTM_OFFSET     0x002c  /* High speed mode timing register */
#define N32_I2C_FIFOCSR_OFFSET     0x0030  /* FIFO control and status register */
#define N32_I2C_QCMD_OFFSET        0x0034  /* Quick command address register */
#define N32_I2C_GFLTRCTRL_OFFSET   0x0038  /* Analog filter register */

/* Register Addresses *******************************************************/
#define N32_I2C_CTRL1(b)           ((b) + N32_I2C_CTRL1_OFFSET)
#define N32_I2C_CTRL2(b)           ((b) + N32_I2C_CTRL2_OFFSET)
#define N32_I2C_ADR1(b)            ((b) + N32_I2C_ADR1_OFFSET)
#define N32_I2C_ADR2(b)            ((b) + N32_I2C_ADR2_OFFSET)
#define N32_I2C_BUSTM(b)           ((b) + N32_I2C_BUSTM_OFFSET)
#define N32_I2C_TMOUTR(b)          ((b) + N32_I2C_TMOUTR_OFFSET)
#define N32_I2C_STSINT(b)          ((b) + N32_I2C_STSINT_OFFSET)
#define N32_I2C_INTCLR(b)          ((b) + N32_I2C_INTCLR_OFFSET)
#define N32_I2C_CRCR(b)            ((b) + N32_I2C_CRCR_OFFSET)
#define N32_I2C_RDR(b)             ((b) + N32_I2C_RDR_OFFSET)
#define N32_I2C_WDR(b)             ((b) + N32_I2C_WDR_OFFSET)
#define N32_I2C_HSBUSTM(b)         ((b) + N32_I2C_HSBUSTM_OFFSET)
#define N32_I2C_FIFOCSR(b)         ((b) + N32_I2C_FIFOCSR_OFFSET)
#define N32_I2C_QCMD(b)            ((b) + N32_I2C_QCMD_OFFSET)
#define N32_I2C_GFLTRCTRL(b)       ((b) + N32_I2C_GFLTRCTRL_OFFSET)

/* Register Bit Definitions *************************************************/

/* Control register 1: the interrupt enable bits live here together with
 * I2CEN.  Clearing I2CEN is a software reset and I2CEN must stay low for at
 * least three PCLK or three prescaled kernel clock periods.
 */

#define I2C_CTRL1_I2CEN            (1 << 0)   /* Bit 0: Peripheral enable */
#define I2C_CTRL1_WDRIE            (1 << 1)   /* Bit 1: TX interrupt enable (WRAVL) */
#define I2C_CTRL1_RDRIE            (1 << 2)   /* Bit 2: RX interrupt enable (RDAVL) */
#define I2C_CTRL1_NAKIE            (1 << 4)   /* Bit 4: NACK received interrupt enable */
#define I2C_CTRL1_TFCIE            (1 << 6)   /* Bit 6: Transfer complete interrupt enable (TFC, TFCR) */
#define I2C_CTRL1_ERRIE            (1 << 7)   /* Bit 7: Error interrupts enable */

/* Control register 2 */

#define I2C_CTRL2_SADR_SHIFT       (0)        /* Bits 0-9: Slave address (master) */
#define I2C_CTRL2_SADR_MASK        (0x3ff << I2C_CTRL2_SADR_SHIFT)
#define I2C_CTRL2_SADR7_SHIFT      (1)        /* Bits 1-7: 7-bit slave address (master) */
#define I2C_CTRL2_SADR7_MASK       (0x7f << I2C_CTRL2_SADR7_SHIFT)
#define I2C_CTRL2_RWN              (1 << 10)  /* Bit 10: Transfer direction, 1 = read (master) */
#define I2C_CTRL2_START            (1 << 13)  /* Bit 13: START generation */
#define I2C_CTRL2_STOP             (1 << 14)  /* Bit 14: STOP generation (master) */
#define I2C_CTRL2_BYTECNT_SHIFT    (16)       /* Bits 16-23: Number of bytes */
#define I2C_CTRL2_BYTECNT_MASK     (0xff << I2C_CTRL2_BYTECNT_SHIFT)
#define I2C_CTRL2_REFILL           (1 << 24)  /* Bit 24: BYTECNT reload mode */

/* Bus timing register.  tCKDIV = (CKDIV + 1) / f(kernel clock) is the time
 * unit of the other fields: tLSCL = (LSCL + 1) tCKDIV (LSCL >= 1),
 * tHSCL = (HSCL + 1) tCKDIV, tDSCL = (DSCL + 1) tCKDIV and
 * tDSDA = DSDA tCKDIV.
 */

#define I2C_BUSTM_LSCL_SHIFT       (0)        /* Bits 0-7: SCL low period (master) */
#define I2C_BUSTM_LSCL_MASK        (0xff << I2C_BUSTM_LSCL_SHIFT)
#define I2C_BUSTM_HSCL_SHIFT       (8)        /* Bits 8-15: SCL high period (master) */
#define I2C_BUSTM_HSCL_MASK        (0xff << I2C_BUSTM_HSCL_SHIFT)
#define I2C_BUSTM_DSDA_SHIFT       (16)       /* Bits 16-19: Data hold time */
#define I2C_BUSTM_DSDA_MASK        (0xf << I2C_BUSTM_DSDA_SHIFT)
#define I2C_BUSTM_DSCL_SHIFT       (20)       /* Bits 20-23: Data setup time */
#define I2C_BUSTM_DSCL_MASK        (0xf << I2C_BUSTM_DSCL_SHIFT)
#define I2C_BUSTM_CKDIV_SHIFT      (28)       /* Bits 28-31: Kernel clock divider */
#define I2C_BUSTM_CKDIV_MASK       (0xf << I2C_BUSTM_CKDIV_SHIFT)

/* Interrupt and status register */

#define I2C_STSINT_WRE             (1 << 0)   /* Bit 0: Transmit data register empty; writing 1 flushes WDR */
#define I2C_STSINT_WRAVL           (1 << 1)   /* Bit 1: Transmit buffer available */
#define I2C_STSINT_RDAVL           (1 << 2)   /* Bit 2: Receive data available */
#define I2C_STSINT_ADR             (1 << 3)   /* Bit 3: Address matched (slave) */
#define I2C_STSINT_NAKF            (1 << 4)   /* Bit 4: NACK received flag */
#define I2C_STSINT_STOPF           (1 << 5)   /* Bit 5: STOP detection flag */
#define I2C_STSINT_TFC             (1 << 6)   /* Bit 6: Transfer complete (master) */
#define I2C_STSINT_TFCR            (1 << 7)   /* Bit 7: Transfer complete reload */
#define I2C_STSINT_BSER            (1 << 8)   /* Bit 8: Bus error */
#define I2C_STSINT_ABLO            (1 << 9)   /* Bit 9: Arbitration lost */
#define I2C_STSINT_OVF             (1 << 10)  /* Bit 10: Overrun/underrun */
#define I2C_STSINT_CRCERR          (1 << 11)  /* Bit 11: PEC error in reception */
#define I2C_STSINT_TMOUT           (1 << 12)  /* Bit 12: Timeout or tLOW detection flag */
#define I2C_STSINT_BUSY            (1 << 15)  /* Bit 15: Bus busy */

#define I2C_STSINT_ERRORMASK (I2C_STSINT_BSER | I2C_STSINT_ABLO | I2C_STSINT_OVF | \
                              I2C_STSINT_CRCERR | I2C_STSINT_TMOUT)

/* Interrupt clear register: write 1 to clear, each clear bit sits at the
 * position of its status bit.  Write the mask with a plain assignment, a
 * read-modify-write would return the set flags and clear them.
 * ADRCLR also clears CTRL2.START.
 */

#define I2C_INTCLR_ADRCLR          (1 << 3)   /* Bit 3: Address matched clear */
#define I2C_INTCLR_NAKCLR          (1 << 4)   /* Bit 4: NACK clear */
#define I2C_INTCLR_STOPCLR         (1 << 5)   /* Bit 5: STOP detection clear */
#define I2C_INTCLR_BSERCLR         (1 << 8)   /* Bit 8: Bus error clear */
#define I2C_INTCLR_ABLOCLR         (1 << 9)   /* Bit 9: Arbitration lost clear */
#define I2C_INTCLR_OVFCLR          (1 << 10)  /* Bit 10: Overrun/underrun clear */
#define I2C_INTCLR_CRCCLR          (1 << 11)  /* Bit 11: PEC error clear */
#define I2C_INTCLR_TMOUTCLR        (1 << 12)  /* Bit 12: Timeout clear */
#define I2C_INTCLR_ALRTCLR         (1 << 13)  /* Bit 13: SMBus alert clear */

#define I2C_INTCLR_CLEARMASK (I2C_INTCLR_ADRCLR | I2C_INTCLR_NAKCLR | \
                              I2C_INTCLR_STOPCLR | I2C_INTCLR_BSERCLR | \
                              I2C_INTCLR_ABLOCLR | I2C_INTCLR_OVFCLR | \
                              I2C_INTCLR_CRCCLR | I2C_INTCLR_TMOUTCLR | \
                              I2C_INTCLR_ALRTCLR)

/* Receive data register */

#define I2C_RDR_RDAT               (0xff)     /* Bits 0-7: 8-bit receive data */

/* Transmit data register: only write when STSINT.WRE is set */

#define I2C_WDR_WDAT               (0xff)     /* Bits 0-7: 8-bit transmit data */

#endif /* CONFIG_N32H7_N32H76X */
#endif /* __ARCH_ARM_SRC_N32H7_HARDWARE_N32H76X_I2C_H */
