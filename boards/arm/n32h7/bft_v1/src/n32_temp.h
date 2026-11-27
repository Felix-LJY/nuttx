/****************************************************************************
 * boards/arm/n32h7/bft_v1/src/n32_temp.h
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

#ifndef __BOARDS_ARM_N32H7_BFT_V1_SRC_N32_TEMP_H
#define __BOARDS_ARM_N32H7_BFT_V1_SRC_N32_TEMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The temperature sensor of the chip, as the data sheet (table 4-72) and the
 * user manual describe it: its voltage drops by the slope per degree and is
 * V30 at 30 degrees, and the manual takes an empirical 1.25 degrees off the
 * result.  The ADC measures against VDDA, of which only the nominal value
 * is known: the internal reference of the chip varies too much to correct
 * with.
 */

#define N32_TEMP_VDDA_MV     3300   /* Reference of the ADC */
#define N32_TEMP_V30_MV      1370   /* Sensor voltage at 30 degrees */
#define N32_TEMP_SLOPE_UV    3400   /* Volts per degree, typical */
#define N32_TEMP_T30         30     /* The reference temperature */
#define N32_TEMP_TOFFSET     1.25f  /* Empirical offset of the manual */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int n32_temp_setup(void);

/****************************************************************************
 * Name: n32_adc_temp
 *
 * Description:
 *   Degrees Celsius for the raw result of converting the temperature sensor.
 *
 ****************************************************************************/

static inline float n32_adc_temp(int32_t raw)
{
  float vsense = (float)raw * N32_TEMP_VDDA_MV / 4095.0f;

  return (N32_TEMP_V30_MV - vsense) * 1000.0f / N32_TEMP_SLOPE_UV +
         N32_TEMP_T30 - N32_TEMP_TOFFSET;
}

#endif /* __BOARDS_ARM_N32H7_BFT_V1_SRC_N32_TEMP_H */
