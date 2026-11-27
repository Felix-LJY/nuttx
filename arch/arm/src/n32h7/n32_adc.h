/****************************************************************************
 * arch/arm/src/n32h7/n32_adc.h
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

#ifndef __ARCH_ARM_SRC_N32H7_N32_ADC_H
#define __ARCH_ARM_SRC_N32H7_N32_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

#include <nuttx/analog/adc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Internal channels of ADC3 */

#define N32_ADC_VBAT_CH         17  /* VBAT divided by four */
#define N32_ADC_TEMP_CH         18  /* Temperature sensor */
#define N32_ADC_VREFINT_CH      19  /* Internal reference voltage */

/* Flags for the channels member of struct n32_adc_wdog_s, above bit 19 */

#define N32_ADC_WDOG_ALL        (1u << 30)  /* Watchdog 1: every channel */
#define N32_ADC_WDOG_SINGLE     (1u << 31)  /* Watchdog 1: the channel ch */

/* DMA request of each ADC.  A board may route it to another controller by
 * defining the name in board.h.
 */

#ifdef CONFIG_N32H7_ADC1_DMA
#  ifndef DMAMAP_ADC1
#    define DMAMAP_ADC1         DMAMAP_DMA1_ADC1
#  endif
#endif

#ifdef CONFIG_N32H7_ADC2_DMA
#  ifndef DMAMAP_ADC2
#    define DMAMAP_ADC2         DMAMAP_DMA1_ADC2
#  endif
#endif

#ifdef CONFIG_N32H7_ADC3_DMA
#  ifndef DMAMAP_ADC3
#    define DMAMAP_ADC3         DMAMAP_DMA1_ADC3
#  endif
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* What n32_adc_setwatchdog() monitors.  channels has one bit per channel
 * 0 to 19, events asks for an interrupt whenever a monitored conversion is
 * out of range.
 */

struct n32_adc_wdog_s
{
  uint16_t upper;       /* Highest in-range value */
  uint16_t lower;       /* Lowest in-range value */
  uint32_t channels;    /* Monitored channels, bit n is channel n */
  bool     events;      /* Interrupt on out of range conversions */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: n32_adc_initialize
 *
 * Description:
 *   Prepare ADC intf to sample a list of channels.  The upper half is bound
 *   with adc_register().
 *
 * Input Parameters:
 *   intf        - ADC number, 1, 2 or 3
 *   chanlist    - The channels to convert, 0 to 19, in the order they are
 *                 converted.  ADC3 converts the battery monitor (17), the
 *                 temperature sensor (18) and VREFINT (19).
 *   nchannels   - Number of channels in the list.
 *
 * Returned Value:
 *   A reference to the ADC device structure, NULL on failure.
 *
 ****************************************************************************/

EXTERN FAR struct adc_dev_s *n32_adc_initialize(int intf,
                                                FAR const uint8_t *chanlist,
                                                int nchannels);

/****************************************************************************
 * Name: n32_adc_setoffset
 *
 * Description:
 *   Add or subtract a value from the conversions of one channel.  Without
 *   saturation the result may be negative, with saturation it stays
 *   unsigned from zero to full scale, which the watchdogs need.  Of two
 *   offsets for the same channel the lower number wins.  Offsets have no
 *   effect while the ADC oversamples.
 *
 *   The setting is written to the ADC when the device is next opened.
 *
 * Input Parameters:
 *   dev         - The ADC device returned by n32_adc_initialize()
 *   n           - Offset register, 1 to 4
 *   ch          - Channel the offset applies to, 0 to 19
 *   value       - Offset, 0 to 4095
 *   positive    - Add the offset instead of subtracting it
 *   saturate    - Saturate at zero and full scale
 *
 * Returned Value:
 *   OK on success, a negated errno value on failure.
 *
 ****************************************************************************/

EXTERN int n32_adc_setoffset(FAR struct adc_dev_s *dev, uint8_t n,
                             uint8_t ch, uint16_t value, bool positive,
                             bool saturate);

/****************************************************************************
 * Name: n32_adc_setsampletime
 *
 * Description:
 *   Set the sampling time of one channel of the list, instead of the value
 *   the driver selected for it.  n32_adc_initialize() gives every channel
 *   of its list the selected time, so this follows it.  The setting is
 *   written to the ADC when the device is next opened.
 *
 * Input Parameters:
 *   dev         - The ADC device returned by n32_adc_initialize()
 *   ch          - Channel of the list, 0 to 19
 *   cycles      - Sampling time in ADC clock cycles: 1, 2, 3, 4, 5, 6, 7,
 *                 10, 13, 17, 24, 32, 83, 93, 215 or 397
 *
 * Returned Value:
 *   OK on success, a negated errno value on failure.
 *
 ****************************************************************************/

EXTERN int n32_adc_setsampletime(FAR struct adc_dev_s *dev, uint8_t ch,
                                 uint16_t cycles);

/****************************************************************************
 * Name: n32_adc_setwatchdog
 *
 * Description:
 *   Have a watchdog watch channels.  A conversion is out of range when it
 *   is below wdog->lower or above wdog->upper.  Watchdog 1 watches every
 *   converted channel or one channel, watchdogs 2 and 3 watch any set of
 *   channels.  With events set the driver stops converting when the
 *   watchdog interrupts.
 *
 *   The setting is written to the ADC when the device is next opened.
 *
 * Input Parameters:
 *   dev         - The ADC device returned by n32_adc_initialize()
 *   n           - Watchdog, 1, 2 or 3
 *   wdog        - Thresholds, channels and events.  A channel set without
 *                 channels switches the watchdog off.  For watchdog 1 it
 *                 carries N32_ADC_WDOG_ALL or N32_ADC_WDOG_SINGLE.
 *   ch          - The channel watchdog 1 watches with N32_ADC_WDOG_SINGLE
 *
 * Returned Value:
 *   OK on success, a negated errno value on failure.
 *
 ****************************************************************************/

EXTERN int n32_adc_setwatchdog(FAR struct adc_dev_s *dev, uint8_t n,
                               FAR const struct n32_adc_wdog_s *wdog,
                               uint8_t ch);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_N32H7_N32_ADC_H */
