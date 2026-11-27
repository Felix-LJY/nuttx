/****************************************************************************
 * arch/arm/src/n32h7/hardware/n32h76x_adc.h
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

#ifndef __ARCH_ARM_SRC_N32H7_HARDWARE_N32H76X_ADC_H
#define __ARCH_ARM_SRC_N32H7_HARDWARE_N32H76X_ADC_H

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

/* ADC1 to ADC3 have the same registers, 32 bits wide each.  Bit names are
 * those of the vendor CMSIS device header.  CTRL2 and CTRL3 contain reserved
 * bits whose reset value is one, so they only accept read-modify-write.
 */

/* Register Offsets *********************************************************/

#define N32_ADC_STS_OFFSET         0x0000  /* Status register */
#define N32_ADC_CTRL1_OFFSET       0x0004  /* Control register 1 */
#define N32_ADC_CTRL2_OFFSET       0x0008  /* Control register 2 */
#define N32_ADC_CTRL3_OFFSET       0x000c  /* Control register 3 */
#define N32_ADC_SAMPT1_OFFSET      0x0010  /* Sampling time of channels 0-7 */
#define N32_ADC_SAMPT2_OFFSET      0x0014  /* Sampling time of channels 8-15 */
#define N32_ADC_SAMPT3_OFFSET      0x0018  /* Sampling time of channels 16-19 */
#define N32_ADC_DIFSEL_OFFSET      0x001c  /* Differential mode selection */
#define N32_ADC_AWDCTRL_OFFSET     0x0020  /* Watchdog output to timer */
#define N32_ADC_OFFSET1_OFFSET     0x0024  /* Data offset 1 */
#define N32_ADC_OFFSET2_OFFSET     0x0028  /* Data offset 2 */
#define N32_ADC_OFFSET3_OFFSET     0x002c  /* Data offset 3 */
#define N32_ADC_OFFSET4_OFFSET     0x0030  /* Data offset 4 */
#define N32_ADC_AWD1HIGH_OFFSET    0x0034  /* Watchdog 1 high threshold */
#define N32_ADC_AWD1LOW_OFFSET     0x0038  /* Watchdog 1 low threshold */
#define N32_ADC_AWD2HIGH_OFFSET    0x003c  /* Watchdog 2 high threshold */
#define N32_ADC_AWD2LOW_OFFSET     0x0040  /* Watchdog 2 low threshold */
#define N32_ADC_AWD3HIGH_OFFSET    0x0044  /* Watchdog 3 high threshold */
#define N32_ADC_AWD3LOW_OFFSET     0x0048  /* Watchdog 3 low threshold */
#define N32_ADC_AWD2EN_OFFSET      0x004c  /* Watchdog 2 monitored channels */
#define N32_ADC_AWD3EN_OFFSET      0x0050  /* Watchdog 3 monitored channels */
#define N32_ADC_AWD2INTEN_OFFSET   0x0054  /* Watchdog 2 interrupt enable */
#define N32_ADC_AWD3INTEN_OFFSET   0x0058  /* Watchdog 3 interrupt enable */
#define N32_ADC_AWD2STS_OFFSET     0x005c  /* Watchdog 2 status */
#define N32_ADC_AWD3STS_OFFSET     0x0060  /* Watchdog 3 status */
#define N32_ADC_RSEQ1_OFFSET       0x0064  /* Regular sequence 19, 20, length */
#define N32_ADC_RSEQ2_OFFSET       0x0068  /* Regular sequence 13-18 */
#define N32_ADC_RSEQ3_OFFSET       0x006c  /* Regular sequence 7-12 */
#define N32_ADC_RSEQ4_OFFSET       0x0070  /* Regular sequence 1-6 */
#define N32_ADC_DAT_OFFSET         0x0088  /* Regular data */
#define N32_ADC_FIFOCFG_OFFSET     0x008c  /* FIFO configuration */
#define N32_ADC_FIFOSTS_OFFSET     0x0090  /* FIFO status */
#define N32_ADC_DLYSMP_OFFSET      0x0094  /* Delayed sampling */
#define N32_ADC_OSCFG_OFFSET       0x0098  /* Oversampling */
#define N32_ADC_GCOMP_OFFSET       0x00a0  /* Gain compensation */

/* Register Addresses *******************************************************/

#define N32_ADC_STS(b)            ((b) + N32_ADC_STS_OFFSET)
#define N32_ADC_CTRL1(b)          ((b) + N32_ADC_CTRL1_OFFSET)
#define N32_ADC_CTRL2(b)          ((b) + N32_ADC_CTRL2_OFFSET)
#define N32_ADC_CTRL3(b)          ((b) + N32_ADC_CTRL3_OFFSET)
#define N32_ADC_SAMPT1(b)         ((b) + N32_ADC_SAMPT1_OFFSET)
#define N32_ADC_SAMPT2(b)         ((b) + N32_ADC_SAMPT2_OFFSET)
#define N32_ADC_SAMPT3(b)         ((b) + N32_ADC_SAMPT3_OFFSET)
#define N32_ADC_DIFSEL(b)         ((b) + N32_ADC_DIFSEL_OFFSET)
#define N32_ADC_AWDCTRL(b)        ((b) + N32_ADC_AWDCTRL_OFFSET)
#define N32_ADC_OFFSET(b, n)      ((b) + N32_ADC_OFFSET1_OFFSET + 4 * ((n) - 1))
#define N32_ADC_AWD1HIGH(b)       ((b) + N32_ADC_AWD1HIGH_OFFSET)
#define N32_ADC_AWD1LOW(b)        ((b) + N32_ADC_AWD1LOW_OFFSET)
#define N32_ADC_AWD2HIGH(b)       ((b) + N32_ADC_AWD2HIGH_OFFSET)
#define N32_ADC_AWD2LOW(b)        ((b) + N32_ADC_AWD2LOW_OFFSET)
#define N32_ADC_AWD3HIGH(b)       ((b) + N32_ADC_AWD3HIGH_OFFSET)
#define N32_ADC_AWD3LOW(b)        ((b) + N32_ADC_AWD3LOW_OFFSET)
#define N32_ADC_AWD2EN(b)         ((b) + N32_ADC_AWD2EN_OFFSET)
#define N32_ADC_AWD3EN(b)         ((b) + N32_ADC_AWD3EN_OFFSET)
#define N32_ADC_AWD2INTEN(b)      ((b) + N32_ADC_AWD2INTEN_OFFSET)
#define N32_ADC_AWD3INTEN(b)      ((b) + N32_ADC_AWD3INTEN_OFFSET)
#define N32_ADC_AWD2STS(b)        ((b) + N32_ADC_AWD2STS_OFFSET)
#define N32_ADC_AWD3STS(b)        ((b) + N32_ADC_AWD3STS_OFFSET)
#define N32_ADC_RSEQ1(b)          ((b) + N32_ADC_RSEQ1_OFFSET)
#define N32_ADC_RSEQ2(b)          ((b) + N32_ADC_RSEQ2_OFFSET)
#define N32_ADC_RSEQ3(b)          ((b) + N32_ADC_RSEQ3_OFFSET)
#define N32_ADC_RSEQ4(b)          ((b) + N32_ADC_RSEQ4_OFFSET)
#define N32_ADC_DAT(b)            ((b) + N32_ADC_DAT_OFFSET)
#define N32_ADC_FIFOCFG(b)        ((b) + N32_ADC_FIFOCFG_OFFSET)
#define N32_ADC_FIFOSTS(b)        ((b) + N32_ADC_FIFOSTS_OFFSET)
#define N32_ADC_DLYSMP(b)         ((b) + N32_ADC_DLYSMP_OFFSET)
#define N32_ADC_OSCFG(b)          ((b) + N32_ADC_OSCFG_OFFSET)
#define N32_ADC_GCOMP(b)          ((b) + N32_ADC_GCOMP_OFFSET)

/* Register Bit Definitions *************************************************/

/* Status register.  Hardware sets these flags, software clears them by
 * writing one.  RDY and PDRDY of CTRL3 repeat RDYF and PDRDYF.
 */

#define ADC_STS_ENDC              (1 << 0)   /* Bit 0: Regular sequence done */
#define ADC_STS_ENDCA             (1 << 1)   /* Bit 1: Regular conversion done */
#define ADC_STS_JSTR              (1 << 2)   /* Bit 2: Injected sequence started */
#define ADC_STS_JENDC             (1 << 3)   /* Bit 3: Injected sequence done */
#define ADC_STS_JENDCA            (1 << 4)   /* Bit 4: Injected conversion done */
#define ADC_STS_STR               (1 << 5)   /* Bit 5: Regular sequence started */
#define ADC_STS_AWDG1             (1 << 6)   /* Bit 6: Watchdog 1 event */
#define ADC_STS_RDYF              (1 << 8)   /* Bit 8: ADC ready */
#define ADC_STS_PDRDYF            (1 << 9)   /* Bit 9: ADC powered down */
#define ADC_STS_EOSAMP            (1 << 11)  /* Bit 11: Sampling of a regular channel done */
#define ADC_STS_ROSOVF            (1 << 12)  /* Bit 12: Oversampled regular data overflow */
#define ADC_STS_JOSOVF            (1 << 13)  /* Bit 13: Oversampled injected data overflow */
#define ADC_STS_ROSOVFDSMU        (1 << 14)  /* Bit 14: 15-bit regular data overflow */
#define ADC_STS_REFLAG            (1 << 16)  /* Bit 16: Read of regular data when none was pending */
#define ADC_STS_WEFLAG            (1 << 17)  /* Bit 17: Regular data lost, it was not read in time */

/* Control register 1 */

#define ADC_CTRL1_SCANMD          (1 << 0)   /* Bit 0: Scan the whole sequence per trigger */
#define ADC_CTRL1_ENDCAIEN        (1 << 1)   /* Bit 1: ENDCA interrupt enable */
#define ADC_CTRL1_ENDCIEN         (1 << 2)   /* Bit 2: ENDC interrupt enable */
#define ADC_CTRL1_JENDCAIEN       (1 << 3)   /* Bit 3: JENDCA interrupt enable */
#define ADC_CTRL1_JENDCIEN        (1 << 4)   /* Bit 4: JENDC interrupt enable */
#define ADC_CTRL1_AWD1IEN         (1 << 5)   /* Bit 5: Watchdog 1 interrupt enable */
#define ADC_CTRL1_AWD1SGLEN       (1 << 6)   /* Bit 6: Watchdog 1 watches AWD1CH only */
#define ADC_CTRL1_AUTOJC          (1 << 7)   /* Bit 7: Injected sequence follows regular one */
#define ADC_CTRL1_MULTMODE_SHIFT  (11)       /* Bits 11-15: Multi ADC mode */
#define ADC_CTRL1_MULTMODE_MASK   (31 << ADC_CTRL1_MULTMODE_SHIFT)
#define ADC_CTRL1_AWD1CH_SHIFT    (16)       /* Bits 16-20: Channel watched by watchdog 1 */
#define ADC_CTRL1_AWD1CH_MASK     (31 << ADC_CTRL1_AWD1CH_SHIFT)
#define ADC_CTRL1_AWD1EJCH        (1 << 21)  /* Bit 21: Watchdog 1 on injected channels */
#define ADC_CTRL1_AWD1ERCH        (1 << 22)  /* Bit 22: Watchdog 1 on regular channels */
#define ADC_CTRL1_DREGCH          (1 << 23)  /* Bit 23: Discontinuous regular sequence */
#define ADC_CTRL1_DJCH            (1 << 24)  /* Bit 24: Discontinuous injected sequence */
#define ADC_CTRL1_RDYIEN          (1 << 26)  /* Bit 26: RDYF interrupt enable */
#define ADC_CTRL1_PDRDYIEN        (1 << 27)  /* Bit 27: PDRDYF interrupt enable */
#define ADC_CTRL1_EOSMPIEN        (1 << 28)  /* Bit 28: EOSAMP interrupt enable */
#define ADC_CTRL1_REIEN           (1 << 29)  /* Bit 29: REFLAG interrupt enable */
#define ADC_CTRL1_WEIEN           (1 << 30)  /* Bit 30: WEFLAG interrupt enable */

/* Control register 2 */

#define ADC_CTRL2_ON              (1 << 0)   /* Bit 0: ADC on */
#define ADC_CTRL2_CTU             (1 << 1)   /* Bit 1: Convert continuously */
#define ADC_CTRL2_ENCAL           (1 << 2)   /* Bit 2: Calibration running */
#define ADC_CTRL2_DMNGT_SHIFT     (4)        /* Bits 4-5: Where regular data goes */
#define ADC_CTRL2_DMNGT_MASK      (3 << ADC_CTRL2_DMNGT_SHIFT)
/* The results go to the data register, to the DSMU, or the DMA reads them */

#  define ADC_CTRL2_DMNGT_DR      (0 << ADC_CTRL2_DMNGT_SHIFT)
#  define ADC_CTRL2_DMNGT_DSMU    (2 << ADC_CTRL2_DMNGT_SHIFT)
#  define ADC_CTRL2_DMNGT_DMA     (3 << ADC_CTRL2_DMNGT_SHIFT)
#define ADC_CTRL2_DMAMD_SHIFT     (7)        /* Bits 7-8: DMA mode of multi ADC modes */
#define ADC_CTRL2_DMAMD_MASK      (3 << ADC_CTRL2_DMAMD_SHIFT)
#define ADC_CTRL2_EXTPJSEL_SHIFT  (9)        /* Bits 9-10: Injected trigger edge */
#define ADC_CTRL2_ALIG            (1 << 11)  /* Bit 11: Left align the data */
#define ADC_CTRL2_EXTJSEL_SHIFT   (13)       /* Bits 13-18: Injected trigger source */
#define ADC_CTRL2_SWSTRJCH        (1 << 21)  /* Bit 21: Start the injected sequence */
#define ADC_CTRL2_SWSTRRCH        (1 << 22)  /* Bit 22: Start the regular sequence */
#define ADC_CTRL2_TEMPEN          (1 << 23)  /* Bit 23: Temperature sensor on */
#define ADC_CTRL2_EXTRSEL_SHIFT   (24)       /* Bits 24-29: Regular trigger source */
#define ADC_CTRL2_EXTRSEL_MASK    (0x3f << ADC_CTRL2_EXTRSEL_SHIFT)
#define ADC_CTRL2_EXTPRSEL_SHIFT  (30)       /* Bits 30-31: Regular trigger edge */
#define ADC_CTRL2_EXTPRSEL_MASK   (3 << ADC_CTRL2_EXTPRSEL_SHIFT)
#  define ADC_CTRL2_EXTPRSEL_SW   (0 << ADC_CTRL2_EXTPRSEL_SHIFT)  /* Software */
#  define ADC_CTRL2_EXTPRSEL_RISE (1 << ADC_CTRL2_EXTPRSEL_SHIFT)  /* Rising edge */

/* Control register 3 */

#define ADC_CTRL3_RES_12BIT       (1 << 1)   /* Bit 1: 12 bit result, 0 is 10 bit */
#define ADC_CTRL3_CKMOD           (1 << 4)   /* Bit 4: PLL clock instead of AHB */
#define ADC_CTRL3_RDY             (1 << 5)   /* Bit 5: ADC ready */
#define ADC_CTRL3_PDRDY           (1 << 6)   /* Bit 6: ADC powered down */
#define ADC_CTRL3_BPCAL           (1 << 7)   /* Bit 7: Bypass the calibration */
#define ADC_CTRL3_VREFINTEN       (1 << 8)   /* Bit 8: VREFINT on */
#define ADC_CTRL3_VBATMEN         (1 << 9)   /* Bit 9: Battery monitor on */
#define ADC_CTRL3_RSTART          (1 << 24)  /* Bit 24: Regular sequence started */
#define ADC_CTRL3_JSTART          (1 << 25)  /* Bit 25: Injected sequence started */
#define ADC_CTRL3_SWRSTOP         (1 << 26)  /* Bit 26: Stop the regular sequence */
#define ADC_CTRL3_SWJSTOP         (1 << 27)  /* Bit 27: Stop the injected sequence */
#define ADC_CTRL3_GCOMPEN         (1 << 28)  /* Bit 28: Gain compensation on */
#define ADC_CTRL3_GCOMP_SATEN     (1 << 29)  /* Bit 29: Gain compensation saturates */

/* Sampling time registers, four bits per channel: 1, 2, 3, 4, 5, 6, 7, 10,
 * 13, 17, 24, 32, 83, 93, 215 or 397 clock cycles for codes 0 to 15
 */

#define ADC_SAMPT_SAMPX_SHIFT(ch) (4 * ((ch) & 7))
#define ADC_SAMPT_SAMPX_MASK(ch)  (15 << ADC_SAMPT_SAMPX_SHIFT(ch))

/* Differential mode selection: channel i + 1 is differential to its
 * negative input when bit i is set
 */

#define ADC_DIFSEL_DIFSEL         (0x000ffffe)

/* Watchdog output to timer *************************************************/

#define ADC_AWDCTRL_AWD1TIMEN     (1 << 0)   /* Bit 0: Watchdog 1 event starts a timer */
#define ADC_AWDCTRL_AWD2TIMEN     (1 << 1)   /* Bit 1: Watchdog 2 event starts a timer */
#define ADC_AWDCTRL_AWD3TIMEN     (1 << 2)   /* Bit 2: Watchdog 3 event starts a timer */

/* Data offset registers 1 to 4 */

#define ADC_OFFSET_OFFSCHxDAT     (0x0fff)   /* Bits 0-11: Offset */
#define ADC_OFFSET_OFFSCHxDIR     (1 << 24)  /* Bit 24: Add the offset */
#define ADC_OFFSET_OFFSCHxSATEN   (1 << 25)  /* Bit 25: Saturate at 0 and full scale */
#define ADC_OFFSET_OFFSCHxCH_SHIFT (26)      /* Bits 26-30: Channel the offset applies to */
#define ADC_OFFSET_OFFSCHxCH_MASK (31 << ADC_OFFSET_OFFSCHxCH_SHIFT)
#define ADC_OFFSET_OFFSCHxEN      (1 << 31)  /* Bit 31: Offset on */

/* Watchdog thresholds: 12-bit values a conversion is compared with after
 * offset and gain compensation and before alignment
 */

#define ADC_AWDxHIGH_HTH          (0x0fff)   /* Bits 0-11: High threshold */
#define ADC_AWDxLOW_LTH           (0x0fff)   /* Bits 0-11: Low threshold */
#define ADC_AWD1LOW_AWDFIL_SHIFT  (12)       /* Bits 12-14: Watchdog 1 raises its flag after this many events in a row, 0 disables the filter */
#define ADC_AWD1LOW_AWDFIL_MASK   (7 << ADC_AWD1LOW_AWDFIL_SHIFT)

/* Watchdog 2 and 3: one bit per channel in all four registers */

#define ADC_AWDx_CHANNELMASK      (0x000fffff)

/* Regular sequence.  The first conversion is the lowest five bits of
 * RSEQ4, the length counts conversions from one.
 */

#define ADC_RSEQ_LEN_SHIFT        (10)       /* RSEQ1 bits 10-14: Conversions per sequence */
#define ADC_RSEQ_LEN_MASK         (31 << ADC_RSEQ_LEN_SHIFT)
#define ADC_RSEQ_SQx_SHIFT(rank)  (5 * (((rank) - 1) % 6))
#define ADC_RSEQ_SQx_MASK(rank)   (31 << ADC_RSEQ_SQx_SHIFT(rank))

#define ADC_DAT_DAT               (0x0000ffff)  /* Bits 0-15: Result of right aligned data */

#define ADC_FIFOCFG_FINTEN        (1 << 2)   /* Bit 2: FIFO full interrupt enable */
#define ADC_FIFOCFG_EINTEN        (1 << 3)   /* Bit 3: FIFO empty interrupt enable */
#define ADC_FIFOCFG_HFINTEN       (1 << 4)   /* Bit 4: FIFO half full interrupt enable */
#define ADC_FIFOCFG_EN            (1 << 5)   /* Bit 5: FIFO on */
#define ADC_FIFOCFG_WL_SHIFT      (6)        /* Bits 6-9: Water level minus one */
#define ADC_FIFOCFG_WL_MASK       (15 << ADC_FIFOCFG_WL_SHIFT)
#define ADC_FIFOCFG_CLR           (1 << 10)  /* Bit 10: Discard the FIFO contents */
#define ADC_FIFOCFG_NEINTEN       (1 << 11)  /* Bit 11: FIFO not empty interrupt enable */

#define ADC_FIFOSTS_FFLAG         (1 << 2)   /* Bit 2: FIFO full */
#define ADC_FIFOSTS_EFLAG         (1 << 3)   /* Bit 3: FIFO empty */
#define ADC_FIFOSTS_HFFLAG        (1 << 4)   /* Bit 4: FIFO half full */
#define ADC_FIFOSTS_DATCNT_SHIFT  (5)        /* Bits 5-9: Results waiting in the FIFO */
#define ADC_FIFOSTS_DATCNT_MASK   (31 << ADC_FIFOSTS_DATCNT_SHIFT)
#define ADC_FIFOSTS_NEFLAG        (1 << 11)  /* Bit 11: FIFO not empty */

/* Delayed sampling *********************************************************/

#define ADC_DLYSMP_INTLEADVAL_SHIFT (28)     /* Bits 28-31: Delay of interleaved modes minus one cycle */
#define ADC_DLYSMP_DLYVAL_SHIFT   (24)       /* Bits 24-27: Delay before sampling minus one cycle */
#define ADC_DLYSMP_DLYVAL_MASK    (15 << ADC_DLYSMP_DLYVAL_SHIFT)
#define ADC_DLYSMP_DLYSMPEN       (1 << 23)  /* Bit 23: Delayed sampling on */

/* Oversampling of regular channels.  Results are 16 bit wide and neither
 * offsets nor alignment apply to them.
 */

#define ADC_OSCFG_OSR_SHIFT       (14)       /* Bits 14-17: Conversions summed, 2 power code */
#define ADC_OSCFG_OSR_MASK        (15 << ADC_OSCFG_OSR_SHIFT)
#define ADC_OSCFG_OSS_SHIFT       (18)       /* Bits 18-21: Right shift of the sum */
#define ADC_OSCFG_OSS_MASK        (15 << ADC_OSCFG_OSS_SHIFT)
#define ADC_OSCFG_OSRTRIG         (1 << 22)  /* Bit 22: Every conversion needs its own trigger */
#define ADC_OSCFG_OSRMD           (1 << 23)  /* Bit 23: A trigger restarts instead of resuming */
#define ADC_OSCFG_OSAWD           (1 << 24)  /* Bit 24: Watchdogs see the 12 low bits of the sum */
#define ADC_OSCFG_OSJE            (1 << 13)  /* Bit 13: Injected oversampling on */
#define ADC_OSCFG_OSRE            (1 << 12)  /* Bit 12: Regular oversampling on */

#define ADC_GCOMP_GCOMPDAT         (0x3fff)   /* Bits 0-13: Gain, 4096 is one */

#endif /* CONFIG_N32H7_N32H76X */
#endif /* __ARCH_ARM_SRC_N32H7_HARDWARE_N32H76X_ADC_H */
