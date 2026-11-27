/****************************************************************************
 * boards/arm/n32h7/bft_v1/src/n32_temp.c
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
 * The temperature sensor of the chip as a node of the sensor framework.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <syslog.h>
#include <semaphore.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/clock.h>
#include <nuttx/kthread.h>
#include <nuttx/sensors/sensor.h>

#include "n32_adc.h"
#include "n32_temp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define N32_TEMP_DEVICE     0             /* /dev/uorb/sensor_temp0 */
#define N32_TEMP_ADC        "/dev/adc1"   /* ADC3, which converts channel 18 */
#define N32_TEMP_PRIORITY   100
#define N32_TEMP_STACK      2048
#define N32_TEMP_INTERVAL   USEC_PER_SEC  /* A sample per second by default */
#define N32_TEMP_MINIMUM    USEC_PER_TICK /* The system clock sets the limit */
#define N32_TEMP_MESSAGES   8             /* Results read before one is lost */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct n32_temp_s
{
  struct sensor_lowerhalf_s lower;    /* Must be first */
  sem_t     sem;                      /* Wakes the sampler */
  uint32_t  interval;                 /* Microseconds between samples */
  int       fd;                       /* The ADC, kept open */
  volatile bool active;               /* A reader wants samples */
  volatile bool due;                  /* A sample is due now */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  n32_temp_activate(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep, bool enable);
static int  n32_temp_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                  FAR struct file *filep,
                                  FAR uint32_t *period_us);
static int  n32_temp_sample(FAR struct sensor_temp *sample);
static int  n32_temp_thread(int argc, FAR char *argv[]);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct n32_temp_s g_temp;

static const struct sensor_ops_s g_temp_ops =
{
  .activate     = n32_temp_activate,
  .set_interval = n32_temp_set_interval,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_temp_activate
 *
 * Description:
 *   The framework calls this when the first reader opens the node and when
 *   the last one closes it.
 *
 ****************************************************************************/

static int n32_temp_activate(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep, bool enable)
{
  FAR struct n32_temp_s *priv = (FAR struct n32_temp_s *)lower;

  UNUSED(filep);

  priv->active = enable;

  /* The reader that just arrived gets a sample at once */

  if (enable)
    {
      priv->due = true;
    }

  nxsem_post(&priv->sem);
  return OK;
}

/****************************************************************************
 * Name: n32_temp_set_interval
 *
 * Description:
 *   The framework passes the shortest interval any of its readers asked for.
 *   Faster than the system clock ticks is not possible, and gives that.
 *
 ****************************************************************************/

static int n32_temp_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 FAR uint32_t *period_us)
{
  FAR struct n32_temp_s *priv = (FAR struct n32_temp_s *)lower;

  UNUSED(filep);

  priv->interval = *period_us < N32_TEMP_MINIMUM ? N32_TEMP_MINIMUM
                                                 : *period_us;

  /* The sampler wakes, reads the interval and waits that long */

  nxsem_post(&priv->sem);
  return OK;
}

/****************************************************************************
 * Name: n32_temp_sample
 *
 * Description:
 *   Convert the sensor once and take its result.  The node keeps the ADC to
 *   itself: every sequence is its own, so its results are in order and
 *   nobody else's.  The sensor is the third of the four conversions.
 *
 ****************************************************************************/

static int n32_temp_sample(FAR struct sensor_temp *sample)
{
  struct adc_msg_s msg;
  bool found = false;
  int i;

  if (ioctl(g_temp.fd, ANIOC_TRIGGER, 0) < 0)
    {
      return ERROR;
    }

  for (i = 0; i < N32_TEMP_MESSAGES && !found; i++)
    {
      if (read(g_temp.fd, &msg, sizeof(msg)) != sizeof(msg))
        {
          return ERROR;
        }

      found = msg.am_channel == N32_ADC_TEMP_CH;
    }

  if (!found)
    {
      return ERROR;
    }

  sample->temperature = n32_adc_temp(msg.am_data);
  sample->timestamp   = sensor_get_timestamp();
  return OK;
}

/****************************************************************************
 * Name: n32_temp_thread
 *
 * Description:
 *   Take a sample whenever a reader wants one.  The read of the ADC waits
 *   for the conversion without a timeout, which is why this is a thread of
 *   its own: should the ADC ever fail to deliver, nothing else is held up.
 *   A kernel thread gets no signals, so the waits cannot fail.
 *
 ****************************************************************************/

static int n32_temp_thread(int argc, FAR char *argv[])
{
  FAR struct n32_temp_s *priv = &g_temp;
  struct sensor_temp sample;
  uint32_t ticks;

  UNUSED(argc);
  UNUSED(argv);

  for (; ; )
    {
      /* What happens is decided by the state read below, so the wake-ups
       * asked for so far are spent here: a listener looks the node up,
       * which opens and closes it, then subscribes, and all of that can be
       * done before this thread gets to run.
       */

      while (nxsem_trywait(&priv->sem) == OK)
        {
        }

      if (!priv->active)
        {
          /* Idle until a reader shows up */

          nxsem_wait(&priv->sem);
          continue;
        }

      if (priv->due)
        {
          priv->due = false;
          if (n32_temp_sample(&sample) == OK)
            {
              priv->lower.push_event(priv->lower.priv, &sample,
                                     sizeof(sample));
            }
        }

      /* Wait for the interval, or for a change of state: a reader left, or
       * asked for another interval.  Only the elapsed time brings a sample.
       */

      ticks = USEC2TICK(priv->interval);
      if (nxsem_tickwait(&priv->sem, ticks > 0 ? ticks : 1) != OK)
        {
          priv->due = true;
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_temp_setup
 *
 * Description:
 *   Open the ADC, start the sampler and register the node.  Called after the
 *   ADC exists.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int n32_temp_setup(void)
{
  FAR struct n32_temp_s *priv = &g_temp;
  int ret;

  /* The ADC stays open for good: the last one to close it switches it off,
   * and the next one pays its start-up time.  A reader of the same device
   * would take the samples of this node; the inputs are on /dev/adc0 too.
   */

  priv->fd = open(N32_TEMP_ADC, O_RDONLY);
  if (priv->fd < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to open %s: %d\n", N32_TEMP_ADC, errno);
      return -errno;
    }

  nxsem_init(&priv->sem, 0, 0);
  priv->interval     = N32_TEMP_INTERVAL;
  priv->lower.ops    = &g_temp_ops;
  priv->lower.type   = SENSOR_TYPE_TEMPERATURE;

  ret = kthread_create("temp", N32_TEMP_PRIORITY, N32_TEMP_STACK,
                       n32_temp_thread, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to start the sampler: %d\n", ret);
      close(priv->fd);
      return ret;
    }

  ret = sensor_register(&priv->lower, N32_TEMP_DEVICE);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sensor_register failed: %d\n", ret);
      close(priv->fd);
    }

  return ret;
}
