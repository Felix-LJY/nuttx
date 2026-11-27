/****************************************************************************
 * boards/arm/n32h7/bft_v1/src/n32_adc.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <syslog.h>

#include <nuttx/analog/adc.h>

#include "n32_adc.h"
#include "bft_v1.h"

#ifdef CONFIG_ADC

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The board's analog inputs are PC0 and PC1, channel 10 and 11 of every
 * ADC.  The pins of port F, the other external channels of ADC3, are not
 * bonded in the BGA100 package.
 */

#ifdef CONFIG_N32H7_ADC1
/* ADC1 converts the supply voltage ADC_V and the current ADC_A */

static const uint8_t g_adc1_chanlist[] =
  {
    10, 11
  };
#endif

#ifdef CONFIG_N32H7_ADC2
/* ADC2 converts the supply voltage ADC_V and the current ADC_A */

static const uint8_t g_adc2_chanlist[] =
  {
    10, 11
  };
#endif

#ifdef CONFIG_N32H7_ADC3
/* ADC3 converts the supply voltage, the current, the temperature sensor and
 * VREFINT.  The battery monitor is not converted: the VBAT pin of the board
 * is not wired to anything.
 */

static const uint8_t g_adc3_chanlist[] =
  {
    10, 11, N32_ADC_TEMP_CH, N32_ADC_VREFINT_CH
  };
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_adc_register
 *
 * Description:
 *   Initialize one ADC with its channel list and register the device as the
 *   next /dev/adcN: the ADCs are numbered in the order they are set up,
 *   whichever of the three are enabled.
 *
 * Input Parameters:
 *   intf      - The ADC, 1 to 3
 *   chanlist  - The channels, in the order they are converted
 *   nchannels - Number of channels in the list
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_N32H7_ADC1) || defined(CONFIG_N32H7_ADC2) || \
    defined(CONFIG_N32H7_ADC3)
static int n32_adc_register(int intf, FAR const uint8_t *chanlist,
                            int nchannels)
{
  static int minor = 0;
  FAR struct adc_dev_s *dev;
  char path[12];
  int ret;

  dev = n32_adc_initialize(intf, chanlist, nchannels);
  if (dev == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get ADC%d interface\n", intf);
      return -ENODEV;
    }

  snprintf(path, sizeof(path), "/dev/adc%d", minor);
  ret = adc_register(path, dev);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register %s: %d\n", path, ret);
    }
  else
    {
      minor++;
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_adc_setup
 *
 * Description:
 *   Initialize the ADCs and register the ADC drivers.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int n32_adc_setup(void)
{
  int ret;

#ifdef CONFIG_N32H7_ADC1
  ret = n32_adc_register(1, g_adc1_chanlist,
                         sizeof(g_adc1_chanlist) /
                         sizeof(*g_adc1_chanlist));
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_N32H7_ADC2
  ret = n32_adc_register(2, g_adc2_chanlist,
                         sizeof(g_adc2_chanlist) /
                         sizeof(*g_adc2_chanlist));
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_N32H7_ADC3
  ret = n32_adc_register(3, g_adc3_chanlist,
                         sizeof(g_adc3_chanlist) /
                         sizeof(*g_adc3_chanlist));
  if (ret < 0)
    {
      return ret;
    }
#endif

  return OK;
}

#endif /* CONFIG_ADC */
