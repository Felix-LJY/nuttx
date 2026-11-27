/****************************************************************************
 * boards/arm/n32h7/bft_v1/src/n32_bringup.c
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

#include <sys/types.h>
#include <syslog.h>
#include <errno.h>

#include <arch/board/board.h>

#include <nuttx/fs/fs.h>

#include "bft_v1.h"

#if defined(CONFIG_SENSORS) && defined(CONFIG_N32H7_ADC3)
#  include "n32_temp.h"
#endif

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#if CONFIG_ONESHOT
#include <nuttx/timers/oneshot.h>
#endif

#ifdef CONFIG_CAPTURE
#include <nuttx/timers/capture.h>
#endif

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#ifdef CONFIG_N32H7_I2C
#  include <nuttx/i2c/i2c_master.h>
#  include "n32_i2c.h"
#endif

/* The MS85010 RTC is the only device on I2C1 */

#if defined(CONFIG_RTC_PCF8563) && defined(CONFIG_N32H7_I2C1)
#  include <nuttx/arch.h>
#  include <nuttx/clock.h>
#  include <nuttx/timers/pcf8563.h>
#  define RTC_I2C_BUS 1
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Forward declarations */

#ifdef CONFIG_TIMER
extern int n32_timer_initialize(const char *devpath, int timer);
#endif

#ifdef CONFIG_CAPTURE
extern struct cap_lowerhalf_s *n32_cap_initialize(int timer);
#endif

/****************************************************************************
 * Name: n32_i2c_register
 *
 * Description:
 *   Initialize one I2C port and register its character driver as the next
 *   /dev/i2cN: the buses are numbered in the order they are set up,
 *   whichever of the eight are enabled.
 *
 * Input Parameters:
 *   port - The I2C, 1 to 8
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_N32H7_I2C
static void n32_i2c_register(int port)
{
  FAR struct i2c_master_s *i2c;
#ifdef CONFIG_I2C_DRIVER
  static int minor = 0;
  int ret;
#endif

  i2c = n32_i2cbus_initialize(port);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", port);
      return;
    }

#ifdef CONFIG_I2C_DRIVER
  ret = i2c_register(i2c, minor);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register I2C%d as /dev/i2c%d: %d\n",
             port, minor, ret);
      n32_i2cbus_uninitialize(i2c);
    }
  else
    {
      minor++;
    }
#endif
}
#endif

/****************************************************************************
 * Name: n32_i2c_setup
 *
 * Description:
 *   Initialize and register the I2C ports enabled in the configuration.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_N32H7_I2C
static void n32_i2c_setup(void)
{
#ifdef CONFIG_N32H7_I2C1
  n32_i2c_register(1);
#endif

#ifdef CONFIG_N32H7_I2C2
  n32_i2c_register(2);
#endif

#ifdef CONFIG_N32H7_I2C3
  n32_i2c_register(3);
#endif

#ifdef CONFIG_N32H7_I2C4
  n32_i2c_register(4);
#endif

#ifdef CONFIG_N32H7_I2C5
  n32_i2c_register(5);
#endif

#ifdef CONFIG_N32H7_I2C6
  n32_i2c_register(6);
#endif

#ifdef CONFIG_N32H7_I2C7
  n32_i2c_register(7);
#endif

#ifdef CONFIG_N32H7_I2C8
  n32_i2c_register(8);
#endif
}
#endif

/****************************************************************************
 * Name: n32_rtc_setup
 *
 * Description:
 *   Make the MS85010 RTC on I2C1 the system's clock, and take the system
 *   time from it.  Called once the bus exists, since an I2C RTC is not
 *   available as early as the system clock is started.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef RTC_I2C_BUS
static void n32_rtc_setup(void)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  /* A second reference to the bus that n32_i2c_setup() has brought up */

  i2c = n32_i2cbus_initialize(RTC_I2C_BUS);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", RTC_I2C_BUS);
      return;
    }

  ret = pcf8563_rtc_initialize(i2c);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pcf8563_rtc_initialize() failed: %d\n", ret);
      return;
    }

  /* The chip answers with an error rather than a time it cannot vouch for,
   * in which case the system clock stays where it started.
   */

  clock_synchronize(NULL);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Nothing can be done this early: the board's only RTC hangs off I2C,
 *   which does not exist yet.  n32_rtc_setup() binds it once it does.
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

#ifdef RTC_I2C_BUS
int up_rtc_initialize(void)
{
  return OK;
}
#endif

/****************************************************************************
 * Name: n32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y &&
 *   CONFIG_NSH_ARCHINIT:
 *     Called from the NSH library
 *
 ****************************************************************************/

int n32_bringup(void)
{
  int ret = OK;

  UNUSED(ret);

#ifdef CONFIG_N32H7_I2C
  /* Register the I2C buses */

  n32_i2c_setup();
#endif

#ifdef RTC_I2C_BUS
  /* Start the system clock from the RTC */

  n32_rtc_setup();
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, N32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d\n",  ret);
    }
#endif /* CONFIG_FS_PROCFS */

#if defined(CONFIG_USERLED_LOWER)
  /* Register the LED driver */

  ret = userled_lower_initialize(LED_DRIVER_PATH);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif /* !CONFIG_ARCH_LEDS && CONFIG_USERLED_LOWER */

#ifdef CONFIG_INPUT_BUTTONS_LOWER
  /* Register the BUTTON driver */

  ret = btn_lower_initialize(BTN_DRIVER_PATH);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif /* CONFIG_INPUT_BUTTONS */

#ifdef CONFIG_MATH_CORDIC
  /* Initialize CORDIC and register the CORDIC driver. */

  ret = n32_cordic_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: n32_cordic_setup failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_CDCACM) && !defined(CONFIG_CDCACM_CONSOLE) && \
    !defined(CONFIG_CDCACM_COMPOSITE)
  /* Initialize CDCACM */

  syslog(LOG_INFO, "Initialize CDCACM device\n");

  ret = cdcacm_initialize(0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: cdcacm_initialize failed: %d\n", ret);
    }
#endif /* CONFIG_CDCACM & !CONFIG_CDCACM_CONSOLE */

  /* ========================================================================
   * Timer driver initialization
   * ========================================================================
   * Register timer devices for each enabled timer.
   * Timer number mapping (see n32_tim_lowerhalf.c):
   *   1..4  = ATIM1-4
   *   3..11 = GTIMA1-7  (3=GTIMA1, 4=GTIMA2, ..., 11=GTIMA7)
   *   12..14= GTIMB1-3
   *   15..18= BTIM1-4
   * ========================================================================
   */

#ifdef CONFIG_TIMER

#ifdef CONFIG_N32H7_BTIM1
  /* Register BTIM1 as /dev/timer0 (timer number 15) */

  ret = n32_timer_initialize("/dev/timer0", 15);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register /dev/timer0: %d\n", ret);
    }
#endif

#ifdef CONFIG_N32H7_BTIM2
  ret = n32_timer_initialize("/dev/timer1", 16);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register /dev/timer1: %d\n", ret);
    }
#endif

#ifdef CONFIG_N32H7_BTIM3
  ret = n32_timer_initialize("/dev/timer2", 17);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register /dev/timer2: %d\n", ret);
    }
#endif

#ifdef CONFIG_N32H7_BTIM4
  ret = n32_timer_initialize("/dev/timer3", 18);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register /dev/timer3: %d\n", ret);
    }
#endif

#ifdef CONFIG_N32H7_GTIMA1
  ret = n32_timer_initialize("/dev/timer4", 3);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register /dev/timer4: %d\n", ret);
    }
#endif

#endif /* CONFIG_TIMER */

  /* ========================================================================
   * Oneshot driver initialization
   * ========================================================================
   * Register oneshot devices as /dev/oneshotX using NuttX oneshot_register()
   *
   * oneshot_initialize() returns a struct oneshot_lowerhalf_s *
   * oneshot_register() registers it as a device node.
   * ========================================================================
   */

#ifdef CONFIG_N32H7_ONESHOT

#  ifdef CONFIG_N32H7_GTIMA1
  struct oneshot_lowerhalf_s *oneshot;
  int chan;

  /* Use GTIMA1 (timer 3) with channel 1, 20us resolution */

  chan = (3 << 4) | 1;  /* timer=3, channel=1 */
  oneshot = oneshot_initialize(chan, 20);
  if (oneshot == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize oneshot (GTIMA1)\n");
    }
  else
    {
      ret = oneshot_register("/dev/oneshot0", oneshot);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register /dev/oneshot0: %d\n",
                  ret);
        }
    }

  /* Use GTIMA1 (timer 3) with channel 2, 20us resolution */

  chan = (3 << 4) | 2;  /* timer=3, channel=2 */
  oneshot = oneshot_initialize(chan, 20);
  if (oneshot == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize oneshot (GTIMA1)\n");
    }
  else
    {
      ret = oneshot_register("/dev/oneshot1", oneshot);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register /dev/oneshot1: %d\n",
                  ret);
        }
    }

  /* Use GTIMA1 (timer 3) with channel 3, 20us resolution */

  chan = (3 << 4) | 3;  /* timer=3, channel=3 */
  oneshot = oneshot_initialize(chan, 20);
  if (oneshot == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize oneshot (GTIMA1)\n");
    }
  else
    {
      ret = oneshot_register("/dev/oneshot2", oneshot);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register /dev/oneshot2: %d\n",
                  ret);
        }
    }

  /* Use GTIMA1 (timer 3) with channel 4, 20us resolution */

  chan = (3 << 4) | 4;  /* timer=3, channel=4 */
  oneshot = oneshot_initialize(chan, 20);
  if (oneshot == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize oneshot (GTIMA1)\n");
    }
  else
    {
      ret = oneshot_register("/dev/oneshot3", oneshot);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register /dev/oneshot3: %d\n",
                  ret);
        }
    }
#  endif /* CONFIG_N32H7_GTIMA1 */

#endif /* CONFIG_N32H7_ONESHOT */

#ifdef CONFIG_MMCSD
  /* Initialize the MMC/SD driver */

  ret = n32_sdmmc_initialize(CONFIG_NSH_MMCSDMINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SD slot %d: %d\n",
             CONFIG_NSH_MMCSDMINOR, ret);
    }
#endif

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = n32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: n32_pwm_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = n32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: n32_adc_setup() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_SENSORS) && defined(CONFIG_N32H7_ADC3)
  ret = n32_temp_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: n32_temp_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_CAPTURE
#  ifdef CONFIG_N32H7_GTIMA7_CAP
  struct cap_lowerhalf_s *cap = n32_cap_initialize(11);

  if (cap == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize capture (GTIMA7)\n");
    }
  else
    {
      ret = cap_register("/dev/capture0", cap);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register /dev/capture0: %d\n",
                 ret);
        }
    }
#  endif /* CONFIG_N32H7_GTIMA7_CAP */
#endif /* CONFIG_CAPTURE */

#ifdef CONFIG_MTD
#ifdef HAVE_PROGMEM_CHARDEV
  ret = n32_progmem_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize MTD progmem: %d\n",
             ret);
    }
#endif /* HAVE_PROGMEM_CHARDEV */
#endif /* CONFIG_MTD */

  return OK;
}
