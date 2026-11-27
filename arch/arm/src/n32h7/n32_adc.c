/****************************************************************************
 * arch/arm/src/n32h7/n32_adc.c
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
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/compiler.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#ifdef CONFIG_PM
#  include <nuttx/pm.h>
#endif

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "n32_rcc.h"
#include "n32_dma.h"
#include "hardware/n32h7_rcc.h"
#include "hardware/n32h7_adc.h"
#include "hardware/n32h7_afio.h"
#include "hardware/n32h7_tim.h"
#include "n32_adc.h"

#ifdef CONFIG_N32H7_ADC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef container_of
#  define container_of(ptr, type, member) \
          ((type *)((intptr_t)(ptr) - offsetof(type, member)))
#endif

#if CONFIG_N32H7_ADC_MAX_SAMPLES > 20
#  error "A regular sequence holds at most 20 conversions"
#endif

#define ADC_CLOCK_MAX      20000000  /* Highest ADC clock, data sheet */
#define ADC_READY_LOOPS    100       /* Polls of ADC_READY_DELAY */
#define ADC_READY_DELAY    10        /* Microseconds; power up takes 20 */
#define ADC_FIFO_DEPTH     16        /* Results the FIFO holds */
#define ADC_THRESHOLD_MAX  4095      /* Watchdog thresholds are 12 bit */

/* Internal channels, bits of the mask n32_adc_initialize() builds */

#define ADC_INT_VBAT       (1 << 0)
#define ADC_INT_TEMP       (1 << 1)
#define ADC_INT_VREFINT    (1 << 2)

/* Kernel clock: the divided AHB clock, or the number of the PLL output */

#define ADC_CLK_AHB        0

/* Interrupt enable bits of CTRL1 the driver uses */

#define ADC_CTRL1_INTMASK  (ADC_CTRL1_ENDCAIEN | ADC_CTRL1_ENDCIEN | \
                            ADC_CTRL1_AWD1IEN | ADC_CTRL1_WEIEN)

/* Status bits the driver acknowledges */

#define ADC_STS_CLRMASK    (ADC_STS_ENDC | ADC_STS_ENDCA | ADC_STS_STR | \
                            ADC_STS_EOSAMP | ADC_STS_REFLAG)

/* A timer event that cannot trigger the ADC */

#define ADC_NOTRIG         0xff

/* Kernel clocks a status flag clear needs to reach the converter.  A
 * conversion that ends on the way loses its flags.
 */

#define ADC_SYNC_CLOCKS     16

/* The analog channel switch of the pin an external channel converts: the
 * AFIO_ANAEN_CFG register and the bit in it.
 */

#define ADC_ANAEN(r, b)    (((r) << 5) | (b))
#define ADC_NOPIN          0xff

/* Kconfig value NAME of ADC n: CONFIG_N32H7_ADCn_NAME */

#define ADC_CFG2(n, name)  CONFIG_N32H7_ADC##n##_##name
#define ADC_CFG(n, name)   ADC_CFG2(n, name)

/* Resolution, oversampling and delayed sampling as the registers hold
 * them.
 */

#define ADC_RES(n)         (ADC_CFG(n, RESOLUTION) == 12 ? \
                            ADC_CTRL3_RES_12BIT : 0)
#define ADC_OSCFG(n)       (ADC_CFG(n, OVSR) > 0 ? \
                            (ADC_OSCFG_OSRE | \
                             (ADC_CFG(n, OVSR) << ADC_OSCFG_OSR_SHIFT) | \
                             (ADC_CFG(n, OVSS) << ADC_OSCFG_OSS_SHIFT)) : 0)
#define ADC_DLYSMP(n)      (ADC_CFG(n, DLYSAMP) > 0 ? \
                            (ADC_DLYSMP_DLYSMPEN | \
                             ((ADC_CFG(n, DLYSAMP) - 1) << \
                              ADC_DLYSMP_DLYVAL_SHIFT)) : 0)

/* Watchdog 1 as Kconfig sets it.  The full range of thresholds leaves it
 * off, a channel number below zero has it watch every converted channel.
 */

#define ADC_WDG1CTRL(n)    (ADC_CFG(n, WDG1_HIGH) != ADC_THRESHOLD_MAX || \
                            ADC_CFG(n, WDG1_LOW) != 0 ? \
                            (ADC_CTRL1_AWD1ERCH | ADC_CTRL1_AWD1IEN | \
                             (ADC_CFG(n, WDG1_CHAN) < 0 ? 0 : \
                              ADC_CTRL1_AWD1SGLEN | \
                              ((uint32_t)ADC_CFG(n, WDG1_CHAN) << \
                               ADC_CTRL1_AWD1CH_SHIFT))) : 0)
#define ADC_WDG1HIGH(n)    ((uint32_t)ADC_CFG(n, WDG1_HIGH))
#define ADC_WDG1LOW(n)     ((uint32_t)ADC_CFG(n, WDG1_LOW) | \
                            ((uint32_t)ADC_CFG(n, WDG1_FLT) << \
                             ADC_AWD1LOW_AWDFIL_SHIFT))

/* Timer reserved for each ADC, none unless Kconfig selects one */

#ifdef CONFIG_N32H7_ATIM1_ADC1
#  define ADC1_TIMER       &g_adc_atim1
#elif defined(CONFIG_N32H7_ATIM2_ADC1)
#  define ADC1_TIMER       &g_adc_atim2
#elif defined(CONFIG_N32H7_ATIM3_ADC1)
#  define ADC1_TIMER       &g_adc_atim3
#elif defined(CONFIG_N32H7_ATIM4_ADC1)
#  define ADC1_TIMER       &g_adc_atim4
#elif defined(CONFIG_N32H7_GTIMA1_ADC1)
#  define ADC1_TIMER       &g_adc_gtima1
#elif defined(CONFIG_N32H7_GTIMB1_ADC1)
#  define ADC1_TIMER       &g_adc_gtimb1
#elif defined(CONFIG_N32H7_GTIMB2_ADC1)
#  define ADC1_TIMER       &g_adc_gtimb2
#elif defined(CONFIG_N32H7_GTIMB3_ADC1)
#  define ADC1_TIMER       &g_adc_gtimb3
#endif

#ifdef CONFIG_N32H7_ATIM1_ADC2
#  define ADC2_TIMER       &g_adc_atim1
#elif defined(CONFIG_N32H7_ATIM2_ADC2)
#  define ADC2_TIMER       &g_adc_atim2
#elif defined(CONFIG_N32H7_ATIM3_ADC2)
#  define ADC2_TIMER       &g_adc_atim3
#elif defined(CONFIG_N32H7_ATIM4_ADC2)
#  define ADC2_TIMER       &g_adc_atim4
#elif defined(CONFIG_N32H7_GTIMA1_ADC2)
#  define ADC2_TIMER       &g_adc_gtima1
#elif defined(CONFIG_N32H7_GTIMB1_ADC2)
#  define ADC2_TIMER       &g_adc_gtimb1
#elif defined(CONFIG_N32H7_GTIMB2_ADC2)
#  define ADC2_TIMER       &g_adc_gtimb2
#elif defined(CONFIG_N32H7_GTIMB3_ADC2)
#  define ADC2_TIMER       &g_adc_gtimb3
#endif

#ifdef CONFIG_N32H7_ATIM1_ADC3
#  define ADC3_TIMER       &g_adc_atim1
#elif defined(CONFIG_N32H7_ATIM2_ADC3)
#  define ADC3_TIMER       &g_adc_atim2
#elif defined(CONFIG_N32H7_ATIM3_ADC3)
#  define ADC3_TIMER       &g_adc_atim3
#elif defined(CONFIG_N32H7_ATIM4_ADC3)
#  define ADC3_TIMER       &g_adc_atim4
#elif defined(CONFIG_N32H7_GTIMA1_ADC3)
#  define ADC3_TIMER       &g_adc_gtima1
#elif defined(CONFIG_N32H7_GTIMB1_ADC3)
#  define ADC3_TIMER       &g_adc_gtimb1
#elif defined(CONFIG_N32H7_GTIMB2_ADC3)
#  define ADC3_TIMER       &g_adc_gtimb2
#elif defined(CONFIG_N32H7_GTIMB3_ADC3)
#  define ADC3_TIMER       &g_adc_gtimb3
#endif

#if defined(CONFIG_N32H7_ADC1_DMA) || defined(CONFIG_N32H7_ADC2_DMA) || \
    defined(CONFIG_N32H7_ADC3_DMA)
#  define ADC_HAVE_DMA     1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Clock control of one ADC in the RCC */

struct n32_adc_rcc_s
{
  uint32_t en;          /* Register with the clock enables */
  uint32_t busen;       /* Bus clock, makes the registers reachable */
  uint32_t sysen;       /* Divided AHB clock */
  uint32_t pllen;       /* The selected PLL output */
  uint32_t rst;         /* Register with the reset */
  uint32_t rstbit;      /* Reset */
  uint8_t  divshift;    /* Position of the divider in AHB1DIV2 */
  uint8_t  pllshift;    /* Position of the select in AHB1SEL1 */
};

/* Timer types differ in where the trigger output is selected */

enum n32_adc_timtype_e
{
  ADC_TIM_ATIM = 0,
  ADC_TIM_GTIMA,
  ADC_TIM_GTIMB
};

/* A timer that can trigger conversions: the converter inputs of its TRGO
 * and of its TRGO2, ADC_NOTRIG where it has none
 */

struct n32_adc_tim_s
{
  uint32_t base;        /* Timer register base */
  uint32_t rccenr;      /* Register with the timer clock enable */
  uint32_t rccen;       /* Timer clock enable */
  uint8_t  type;        /* See enum n32_adc_timtype_e */
  uint8_t  trgo;        /* Converter input of TRGO */
  uint8_t  trgo2;       /* Converter input of TRGO2 */
};

/* Watchdog 2 or 3 */

struct n32_adc_awd_s
{
  uint32_t high;        /* Highest in-range value */
  uint32_t low;         /* Lowest in-range value */
  uint32_t channels;    /* Monitored channels, bit n is channel n */
  uint32_t inten;       /* Monitored channels that interrupt */
};

/* This structure describes the state of one ADC block */

struct n32_adc_s
{
  /* Configuration */

  const struct n32_adc_rcc_s *rcc;
  uint32_t base;        /* Base address of the ADC registers */
  uint32_t res;         /* CTRL3 resolution bit */
  uint32_t oscfg;       /* Oversampling register */
  uint32_t dlysmp;      /* Delayed sampling bits */
  uint32_t difsel;      /* Differential channels */
  uint32_t gcompdat;    /* Gain compensation coefficient, 0 switches it
                         * off */
  uint8_t  irq;         /* Interrupt of this ADC */
  uint8_t  intf;        /* ADC number */
  uint8_t  pll;         /* Kernel clock, see ADC_CLK_AHB */
  uint8_t  trigger;     /* Timer event: 0 to 3 are CC1 to CC4, 4 TRGO,
                         * 5 TRGO2 */
  uint32_t freq;        /* Sequences per second */
  uint8_t  syncus;      /* Microseconds a flag clear takes to reach the
                         * converter */

  /* NULL: the software trigger starts conversions */

  const struct n32_adc_tim_s *tim;

  /* Settings of n32_adc_set...(), written to the ADC when it is opened.
   * The sampling times go with the channel list.
   */

  uint8_t  samp[CONFIG_N32H7_ADC_MAX_SAMPLES];
  uint32_t offset[4];   /* Data offset registers */
  uint32_t awd1ctrl;    /* CTRL1 bits of watchdog 1 */
  uint32_t awd1high;    /* Watchdog 1 high threshold register */
  uint32_t awd1low;     /* Watchdog 1 low threshold register */

  /* Watchdogs 2 and 3 */

  struct n32_adc_awd_s awd[2];

#ifdef ADC_HAVE_DMA
  uint32_t dmamap;      /* DMA request and controller */
  uint16_t dmabatch;    /* Sequences the DMA collects per batch */
  uint16_t *dmabuffer;  /* Where the DMA puts them */
  DMA_HANDLE dma;       /* Allocated DMA channel */
#endif

  /* State */

  uint8_t  internal;     /* Internal channels of the list, see ADC_INT_* */
  uint8_t  cchannels;    /* Channels in the list */
  uint8_t  chanlist[CONFIG_N32H7_ADC_MAX_SAMPLES];
  uint8_t  rnchannels;   /* Conversions per sequence */
  uint8_t  current;      /* Channel of the next result */
  uint32_t ie;           /* CTRL1 interrupt enable bits */
  uint32_t overruns;     /* Sequences lost since the opening */
  bool     watching;     /* The watchdog ioctl armed: results are dropped */
  bool     opened;       /* The device is open */

  /* The callbacks of the upper half */

  const struct adc_callback_s *cb;
#ifdef CONFIG_PM
  struct pm_callback_s pm_callback;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void adc_modreg3(FAR struct n32_adc_s *priv, uint32_t clrbits,
                        uint32_t setbits);
static void adc_clear(FAR struct n32_adc_s *priv, uint32_t flags);
static void adc_rccreset(FAR struct n32_adc_s *priv, bool reset);
static void adc_clock(FAR struct n32_adc_s *priv);
static void adc_internal_power(uint8_t internal, bool on);
static int  adc_sampcode(uint16_t cycles);
static void adc_anaen(FAR struct n32_adc_s *priv);
static void adc_set_sampt(FAR struct n32_adc_s *priv);
static void adc_set_sequence(FAR struct n32_adc_s *priv);
static int  adc_timinit(FAR struct n32_adc_s *priv);
static void adc_timstart(FAR struct n32_adc_s *priv, bool enable);
static void adc_startconv(FAR struct n32_adc_s *priv, bool enable);
static int  adc_enable(FAR struct n32_adc_s *priv);
static uint8_t adc_awd23_event(FAR struct n32_adc_s *priv);
static void adc_wdog_enable(FAR struct n32_adc_s *priv);
static void adc_overrun(FAR struct adc_dev_s *dev);
#ifdef ADC_HAVE_DMA
static void adc_dmastart(FAR struct adc_dev_s *dev);
static void adc_dma_callback(DMA_HANDLE handle, uint8_t status,
                             FAR void *arg);
#endif
static int  adc_interrupt(int irq, FAR void *context, FAR void *arg);

static int  adc_bind(FAR struct adc_dev_s *dev,
                     FAR const struct adc_callback_s *callback);
static void adc_reset(FAR struct adc_dev_s *dev);
static int  adc_setup(FAR struct adc_dev_s *dev);
static void adc_shutdown(FAR struct adc_dev_s *dev);
static void adc_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  adc_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg);
#ifdef CONFIG_PM
static int  adc_pm_prepare(FAR struct pm_callback_s *cb, int domain,
                           enum pm_state_e state);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ADC interface operations */

static const struct adc_ops_s g_adcops =
{
  .ao_bind        = adc_bind,
  .ao_reset       = adc_reset,
  .ao_setup       = adc_setup,
  .ao_shutdown    = adc_shutdown,
  .ao_rxint       = adc_rxint,
  .ao_ioctl       = adc_ioctl,
};

/* RCC control of each ADC */

#ifdef CONFIG_N32H7_ADC1
static const struct n32_adc_rcc_s g_adc_rcc1 =
{
  .en            = N32_RCC_AHB1EN1,
  .busen         = RCC_AHB1EN1_M7ADC1BUSEN,
  .sysen         = RCC_AHB1EN1_M7ADC1SYSEN,
  .pllen         = RCC_AHB1EN1_M7ADC1PLLEN,
  .rst           = N32_RCC_AHB1RST1,
  .rstbit        = RCC_AHB1RST1_ADC1RST,
  .divshift      = RCC_AHB1DIV2_ADC1SYSDIV_SHIFT,
  .pllshift      = RCC_AHB1SEL1_ADC1PLLSEL_SHIFT,
};
#endif

#ifdef CONFIG_N32H7_ADC2
static const struct n32_adc_rcc_s g_adc_rcc2 =
{
  .en            = N32_RCC_AHB1EN4,
  .busen         = RCC_AHB1EN4_M7ADC2BUSEN,
  .sysen         = RCC_AHB1EN4_M7ADC2SYSEN,
  .pllen         = RCC_AHB1EN4_M7ADC2PLLEN,
  .rst           = N32_RCC_AHB1RST4,
  .rstbit        = RCC_AHB1RST4_ADC2RST,
  .divshift      = RCC_AHB1DIV2_ADC2SYSDIV_SHIFT,
  .pllshift      = RCC_AHB1SEL1_ADC2PLLSEL_SHIFT,
};
#endif

#ifdef CONFIG_N32H7_ADC3
static const struct n32_adc_rcc_s g_adc_rcc3 =
{
  .en            = N32_RCC_AHB1EN4,
  .busen         = RCC_AHB1EN4_M7ADC3BUSEN,
  .sysen         = RCC_AHB1EN4_M7ADC3SYSEN,
  .pllen         = RCC_AHB1EN4_M7ADC3PLLEN,
  .rst           = N32_RCC_AHB1RST4,
  .rstbit        = RCC_AHB1RST4_ADC3RST,
  .divshift      = RCC_AHB1DIV2_ADC3SYSDIV_SHIFT,
  .pllshift      = RCC_AHB1SEL1_ADC3PLLSEL_SHIFT,
};
#endif

/* Timers that can trigger conversions */

#if defined(CONFIG_N32H7_ATIM1_ADC)
static const struct n32_adc_tim_s g_adc_atim1 =
{
  .base          = N32_ATIMER1_BASE,
  .rccenr        = N32_RCC_APB2EN1,
  .rccen         = RCC_APB2EN1_M7ATIM1EN,
  .type          = ADC_TIM_ATIM,
  .trgo          = 4,
  .trgo2         = 8
};
#endif

#if defined(CONFIG_N32H7_ATIM2_ADC)
static const struct n32_adc_tim_s g_adc_atim2 =
{
  .base          = N32_ATIMER2_BASE,
  .rccenr        = N32_RCC_APB2EN1,
  .rccen         = RCC_APB2EN1_M7ATIM2EN,
  .type          = ADC_TIM_ATIM,
  .trgo          = 9,
  .trgo2         = 14
};
#endif

#if defined(CONFIG_N32H7_ATIM3_ADC)
static const struct n32_adc_tim_s g_adc_atim3 =
{
  .base          = N32_ATIMER3_BASE,
  .rccenr        = N32_RCC_APB5EN1,
  .rccen         = RCC_APB5EN1_M7ATIM3EN,
  .type          = ADC_TIM_ATIM,
  .trgo          = 15,
  .trgo2         = 17
};
#endif

#if defined(CONFIG_N32H7_ATIM4_ADC)
static const struct n32_adc_tim_s g_adc_atim4 =
{
  .base          = N32_ATIMER4_BASE,
  .rccenr        = N32_RCC_APB5EN1,
  .rccen         = RCC_APB5EN1_M7ATIM4EN,
  .type          = ADC_TIM_ATIM,
  .trgo          = 18,
  .trgo2         = 16
};
#endif

#if defined(CONFIG_N32H7_GTIMA1_ADC)
static const struct n32_adc_tim_s g_adc_gtima1 =
{
  .base          = N32_GTIMERA1_BASE,
  .rccenr        = N32_RCC_APB2EN1,
  .rccen         = RCC_APB2EN1_M7GTIMA1EN,
  .type          = ADC_TIM_GTIMA,
  .trgo          = 22,
  .trgo2         = ADC_NOTRIG
};
#endif

#if defined(CONFIG_N32H7_GTIMB1_ADC)
static const struct n32_adc_tim_s g_adc_gtimb1 =
{
  .base          = N32_GTIMERB1_BASE,
  .rccenr        = N32_RCC_APB1EN1,
  .rccen         = RCC_APB1EN1_M7GTIMB1EN,
  .type          = ADC_TIM_GTIMB,
  .trgo          = 19,
  .trgo2         = ADC_NOTRIG
};
#endif

#if defined(CONFIG_N32H7_GTIMB2_ADC)
static const struct n32_adc_tim_s g_adc_gtimb2 =
{
  .base          = N32_GTIMERB2_BASE,
  .rccenr        = N32_RCC_APB1EN1,
  .rccen         = RCC_APB1EN1_M7GTIMB2EN,
  .type          = ADC_TIM_GTIMB,
  .trgo          = 20,
  .trgo2         = ADC_NOTRIG
};
#endif

#if defined(CONFIG_N32H7_GTIMB3_ADC)
static const struct n32_adc_tim_s g_adc_gtimb3 =
{
  .base          = N32_GTIMERB3_BASE,
  .rccenr        = N32_RCC_APB1EN1,
  .rccen         = RCC_APB1EN1_M7GTIMB3EN,
  .type          = ADC_TIM_GTIMB,
  .trgo          = 21,
  .trgo2         = ADC_NOTRIG
};
#endif

/* Select field of AHB1SEL1 for the PLL outputs of ADC_CLK_AHB + 1 on:
 * PLL1B, PLL2B, PLL3B and PLL3C.
 */

static const uint8_t g_adc_pllselect[5] =
{
  0, 1, 0, 2, 3
};

/* Sampling time codes, in cycles */

static const uint16_t g_adc_sampcycles[16] =
{
  1, 2, 3, 4, 5, 6, 7, 10, 13, 17, 24, 32, 83, 93, 215, 397
};

static const uint32_t g_adc_anaenreg[4] =
{
  N32_AFIO_ANAEN_CFG0, N32_AFIO_ANAEN_CFG1, N32_AFIO_ANAEN_CFG2,
  N32_AFIO_ANAEN_CFG3
};

/* ADC1 and ADC2 convert the same pins */

static const uint8_t g_adc_anaen12[20] =
{
  ADC_NOPIN,          /* Channel 0: PA0_C, switched by AFIO_ADCSW_CFG */
  ADC_NOPIN,          /* Channel 1: PA1_C, switched by AFIO_ADCSW_CFG */
  ADC_NOPIN,          /* Channel 2: PF11 or PF13 */
  ADC_ANAEN(0, 8),    /* Channel 3: PA6 */
  ADC_ANAEN(2, 6),    /* Channel 4: PC4 */
  ADC_ANAEN(1, 1),    /* Channel 5: PB1 */
  ADC_NOPIN,          /* Channel 6: PF12 or PF14 */
  ADC_ANAEN(0, 9),    /* Channel 7: PA7 */
  ADC_ANAEN(2, 7),    /* Channel 8: PC5 */
  ADC_ANAEN(1, 0),    /* Channel 9: PB0 */
  ADC_ANAEN(2, 0),    /* Channel 10: PC0 */
  ADC_ANAEN(2, 1),    /* Channel 11: PC1 */
  ADC_ANAEN(2, 2),    /* Channel 12: PC2 */
  ADC_ANAEN(2, 4),    /* Channel 13: PC3 */
  ADC_ANAEN(0, 4),    /* Channel 14: PA2 */
  ADC_ANAEN(0, 5),    /* Channel 15: PA3 */
  ADC_ANAEN(0, 1),    /* Channel 16: PA0 */
  ADC_ANAEN(0, 3),    /* Channel 17: PA1 */
  ADC_ANAEN(0, 6),    /* Channel 18: PA4 */
  ADC_ANAEN(0, 7)     /* Channel 19: PA5 */
};

static const uint8_t g_adc_anaen3[20] =
{
  ADC_NOPIN,          /* Channel 0: PA0_C, switched by AFIO_ADCSW_CFG */
  ADC_NOPIN,          /* Channel 1: PA1_C, switched by AFIO_ADCSW_CFG */
  ADC_ANAEN(3, 6),    /* Channel 2: PF9 */
  ADC_ANAEN(3, 4),    /* Channel 3: PF7 */
  ADC_ANAEN(3, 2),    /* Channel 4: PF5 */
  ADC_ANAEN(3, 0),    /* Channel 5: PF3 */
  ADC_ANAEN(3, 7),    /* Channel 6: PF10 */
  ADC_ANAEN(3, 5),    /* Channel 7: PF8 */
  ADC_ANAEN(3, 3),    /* Channel 8: PF6 */
  ADC_ANAEN(3, 1),    /* Channel 9: PF4 */
  ADC_ANAEN(2, 0),    /* Channel 10: PC0 */
  ADC_ANAEN(2, 1),    /* Channel 11: PC1 */
  ADC_ANAEN(2, 2),    /* Channel 12: PC2 */
  ADC_NOPIN,          /* Channels 13 to 19 */
  ADC_NOPIN,
  ADC_NOPIN,
  ADC_NOPIN,
  ADC_NOPIN,
  ADC_NOPIN,
  ADC_NOPIN
};

/* ADC1 state */

#ifdef CONFIG_N32H7_ADC1
#ifdef CONFIG_N32H7_ADC1_DMA
static uint16_t g_adc1_dmabuffer[CONFIG_N32H7_ADC_MAX_SAMPLES *
                                 CONFIG_N32H7_ADC1_DMA_BATCH]
  locate_data(".axi_ram") aligned_data(ARMV7M_DCACHE_LINESIZE);
#endif

static struct n32_adc_s g_adcpriv1 =
{
  .rcc          = &g_adc_rcc1,
  .base         = N32_ADC1_BASE,
  .res          = ADC_RES(1),
  .oscfg        = ADC_OSCFG(1),
  .dlysmp       = ADC_DLYSMP(1),
  .difsel       = ADC_CFG(1, DIFFCHAN),
  .gcompdat     = ADC_CFG(1, GCOMPDAT) == 4096 ? 0 : ADC_CFG(1, GCOMPDAT),
  .awd1ctrl     = ADC_WDG1CTRL(1),
  .awd1high     = ADC_WDG1HIGH(1),
  .awd1low      = ADC_WDG1LOW(1),
  .irq          = N32_IRQ_ADC1,
  .intf         = 1,
  .pll          = ADC_CFG(1, CLK),
#ifdef ADC1_TIMER
  .tim          = ADC1_TIMER,
  .trigger      = ADC_CFG(1, TIMTRIG),
  .freq         = ADC_CFG(1, SAMPLE_FREQUENCY),
#endif
#ifdef CONFIG_N32H7_ADC1_DMA
  .dmamap       = DMAMAP_ADC1,
  .dmabatch     = CONFIG_N32H7_ADC1_DMA_BATCH,
  .dmabuffer    = g_adc1_dmabuffer,
#endif
#ifdef CONFIG_PM
  .pm_callback  =
    {
      .prepare = adc_pm_prepare,
    }
#endif
};

static struct adc_dev_s g_adcdev1 =
{
  .ad_ops       = &g_adcops,
  .ad_priv      = &g_adcpriv1,
};
#endif

/* ADC2 state */

#ifdef CONFIG_N32H7_ADC2
#ifdef CONFIG_N32H7_ADC2_DMA
static uint16_t g_adc2_dmabuffer[CONFIG_N32H7_ADC_MAX_SAMPLES *
                                 CONFIG_N32H7_ADC2_DMA_BATCH]
  locate_data(".axi_ram") aligned_data(ARMV7M_DCACHE_LINESIZE);
#endif

static struct n32_adc_s g_adcpriv2 =
{
  .rcc          = &g_adc_rcc2,
  .base         = N32_ADC2_BASE,
  .res          = ADC_RES(2),
  .oscfg        = ADC_OSCFG(2),
  .dlysmp       = ADC_DLYSMP(2),
  .difsel       = ADC_CFG(2, DIFFCHAN),
  .gcompdat     = ADC_CFG(2, GCOMPDAT) == 4096 ? 0 : ADC_CFG(2, GCOMPDAT),
  .awd1ctrl     = ADC_WDG1CTRL(2),
  .awd1high     = ADC_WDG1HIGH(2),
  .awd1low      = ADC_WDG1LOW(2),
  .irq          = N32_IRQ_ADC2,
  .intf         = 2,
  .pll          = ADC_CFG(2, CLK),
#ifdef ADC2_TIMER
  .tim          = ADC2_TIMER,
  .trigger      = ADC_CFG(2, TIMTRIG),
  .freq         = ADC_CFG(2, SAMPLE_FREQUENCY),
#endif
#ifdef CONFIG_N32H7_ADC2_DMA
  .dmamap       = DMAMAP_ADC2,
  .dmabatch     = CONFIG_N32H7_ADC2_DMA_BATCH,
  .dmabuffer    = g_adc2_dmabuffer,
#endif
#ifdef CONFIG_PM
  .pm_callback  =
    {
      .prepare = adc_pm_prepare,
    }
#endif
};

static struct adc_dev_s g_adcdev2 =
{
  .ad_ops       = &g_adcops,
  .ad_priv      = &g_adcpriv2,
};
#endif

/* ADC3 state */

#ifdef CONFIG_N32H7_ADC3
#ifdef CONFIG_N32H7_ADC3_DMA
static uint16_t g_adc3_dmabuffer[CONFIG_N32H7_ADC_MAX_SAMPLES *
                                 CONFIG_N32H7_ADC3_DMA_BATCH]
  locate_data(".axi_ram") aligned_data(ARMV7M_DCACHE_LINESIZE);
#endif

static struct n32_adc_s g_adcpriv3 =
{
  .rcc          = &g_adc_rcc3,
  .base         = N32_ADC3_BASE,
  .res          = ADC_RES(3),
  .oscfg        = ADC_OSCFG(3),
  .dlysmp       = ADC_DLYSMP(3),
  .difsel       = ADC_CFG(3, DIFFCHAN),
  .gcompdat     = ADC_CFG(3, GCOMPDAT) == 4096 ? 0 : ADC_CFG(3, GCOMPDAT),
  .awd1ctrl     = ADC_WDG1CTRL(3),
  .awd1high     = ADC_WDG1HIGH(3),
  .awd1low      = ADC_WDG1LOW(3),
  .irq          = N32_IRQ_ADC3,
  .intf         = 3,
  .pll          = ADC_CFG(3, CLK),
#ifdef ADC3_TIMER
  .tim          = ADC3_TIMER,
  .trigger      = ADC_CFG(3, TIMTRIG),
  .freq         = ADC_CFG(3, SAMPLE_FREQUENCY),
#endif
#ifdef CONFIG_N32H7_ADC3_DMA
  .dmamap       = DMAMAP_ADC3,
  .dmabatch     = CONFIG_N32H7_ADC3_DMA_BATCH,
  .dmabuffer    = g_adc3_dmabuffer,
#endif
#ifdef CONFIG_PM
  .pm_callback  =
    {
      .prepare = adc_pm_prepare,
    }
#endif
};

static struct adc_dev_s g_adcdev3 =
{
  .ad_ops       = &g_adcops,
  .ad_priv      = &g_adcpriv3,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Register access */

#define adc_getreg(priv, o)        getreg32((priv)->base + (o))
#define adc_putreg(priv, o, v)     putreg32((v), (priv)->base + (o))
#define adc_modreg(priv, o, c, s)  modifyreg32((priv)->base + (o), (c), (s))

/****************************************************************************
 * Name: adc_modreg3
 *
 * Description:
 *   Change bits of the control register 3.  Its ready and power down bits
 *   are flags that a written one clears, so they are never written back.
 *
 ****************************************************************************/

static void adc_modreg3(FAR struct n32_adc_s *priv, uint32_t clrbits,
                        uint32_t setbits)
{
  putreg32((adc_getreg(priv, N32_ADC_CTRL3_OFFSET) &
            ~(clrbits | ADC_CTRL3_RDY | ADC_CTRL3_PDRDY)) | setbits,
           priv->base + N32_ADC_CTRL3_OFFSET);
}

/****************************************************************************
 * Name: adc_clear
 *
 * Description:
 *   Clear status flags and wait until the clear has reached the converter,
 *   which runs in the kernel clock domain: a conversion that ends on the
 *   way loses its flags, and with them the interrupt that reports it.
 *   The wait is also long enough for a stopped converter to accept the
 *   next start, which it refuses for 0.25 us after the stop.
 *
 ****************************************************************************/

static void adc_clear(FAR struct n32_adc_s *priv, uint32_t flags)
{
  adc_putreg(priv, N32_ADC_STS_OFFSET, flags);
  up_udelay(priv->syncus);
}

/****************************************************************************
 * Name: adc_rccreset
 *
 * Description:
 *   Put the ADC into reset, or release it.
 *
 ****************************************************************************/

static void adc_rccreset(FAR struct n32_adc_s *priv, bool reset)
{
  irqstate_t flags = enter_critical_section();

  modifyreg32(priv->rcc->rst, reset ? 0 : priv->rcc->rstbit,
              reset ? priv->rcc->rstbit : 0);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: adc_clock
 *
 * Description:
 *   Switch the clocks of the ADC on: the bus clock makes the registers
 *   reachable, the kernel clock runs the converter.  They stay on, as the
 *   clock setup of the chip leaves them on too.
 *
 ****************************************************************************/

static void adc_clock(FAR struct n32_adc_s *priv)
{
  uint32_t enables = priv->rcc->busen | priv->rcc->sysen;
  uint32_t divider = 1;
  uint32_t hz;

  modifyreg32(priv->rcc->en, 0, priv->rcc->busen);

  if (priv->pll == ADC_CLK_AHB)
    {
      /* The kernel clock is the bus clock through a divider of 1 to 63 */

      divider = (N32_AHB_FREQUENCY + ADC_CLOCK_MAX - 1) / ADC_CLOCK_MAX;
      if (divider > 63)
        {
          divider = 63;
        }

      modifyreg32(N32_RCC_AHB1DIV2, 0x3f << priv->rcc->divshift,
                  divider << priv->rcc->divshift);
    }
  else
    {
      enables |= priv->rcc->pllen;
      modifyreg32(N32_RCC_AHB1SEL1, 3 << priv->rcc->pllshift,
                  (uint32_t)g_adc_pllselect[priv->pll] <<
                    priv->rcc->pllshift);
    }

  modifyreg32(priv->rcc->en, 0, enables);
  up_udelay(ADC_READY_DELAY);

  /* The kernel clock is the bus clock through the divider, or the PLL
   * output, of which the converter takes at most ADC_CLOCK_MAX.
   */

  hz = priv->pll == ADC_CLK_AHB ? N32_AHB_FREQUENCY / divider :
       ADC_CLOCK_MAX;
  priv->syncus = (ADC_SYNC_CLOCKS * 1000000 + hz - 1) / hz;
}

/****************************************************************************
 * Name: adc_internal_power
 *
 * Description:
 *   Switch the battery monitor, the temperature sensor and VREFINT on or
 *   off.  ADC1 switches them on with bits of its own registers, ADC3
 *   converts them through switches of the AFIO.
 *
 ****************************************************************************/

static void adc_internal_power(uint8_t internal, bool on)
{
  uint32_t ctrl2 = 0;
  uint32_t ctrl3 = 0;
  uint32_t sw = 0;

  if (internal == 0)
    {
      return;
    }

  if ((internal & ADC_INT_VBAT) != 0)
    {
      ctrl3 |= ADC_CTRL3_VBATMEN;
      sw    |= N32_AFIO_ADCSW_CFG_VBAT;
    }

  if ((internal & ADC_INT_TEMP) != 0)
    {
      ctrl2 |= ADC_CTRL2_TEMPEN;
      sw    |= N32_AFIO_ADCSW_CFG_TEMP;
    }

  if ((internal & ADC_INT_VREFINT) != 0)
    {
      ctrl3 |= ADC_CTRL3_VREFINTEN;
      sw    |= N32_AFIO_ADCSW_CFG_VREF;
    }

  /* ADC1 has to be clocked for its registers to be reachable, and the
   * switches belong to the analog function controller.
   */

  modifyreg32(N32_RCC_AHB1EN1, 0, RCC_AHB1EN1_M7ADC1BUSEN);
  modifyreg32(N32_RCC_APB5EN1, 0, RCC_APB5EN1_M7AFECEN);

  /* The ready and power down bits of CTRL3 are flags a written one clears */

  putreg32((getreg32(N32_ADC_CTRL3(N32_ADC1_BASE)) &
            ~(ctrl3 | ADC_CTRL3_RDY | ADC_CTRL3_PDRDY)) |
           (on ? ctrl3 : 0),
           N32_ADC_CTRL3(N32_ADC1_BASE));
  modifyreg32(N32_ADC_CTRL2(N32_ADC1_BASE), ADC_CTRL2_TEMPEN,
              on ? ctrl2 : 0);
  modifyreg32(N32_AFIO_ADCSW_CFG, sw, on ? sw : 0);
}

/****************************************************************************
 * Name: adc_sampcode
 *
 * Description:
 *   Look up the sampling time code of a number of ADC clock cycles.
 *
 * Returned Value:
 *   The code, ERROR when the number is not one the ADC offers.
 *
 ****************************************************************************/

static int adc_sampcode(uint16_t cycles)
{
  int code;

  for (code = 0; code < 16; code++)
    {
      if (g_adc_sampcycles[code] == cycles)
        {
          return code;
        }
    }

  return ERROR;
}

/****************************************************************************
 * Name: adc_anaen
 *
 * Description:
 *   Open the analog channel of every pin the list converts.  Through a
 *   closed switch the pin reaches the ADC weakly: one at the reference
 *   level reads a quarter low.  The switches stay open, another ADC may
 *   convert the same pin.
 *
 ****************************************************************************/

static void adc_anaen(FAR struct n32_adc_s *priv)
{
  FAR const uint8_t *anaen = priv->intf == 3 ? g_adc_anaen3 : g_adc_anaen12;
  uint32_t masks[4] =
    {
      0, 0, 0, 0
    };

  uint8_t code;
  int i;

  for (i = 0; i < priv->cchannels; i++)
    {
      code = anaen[priv->chanlist[i]];
      if (code != ADC_NOPIN)
        {
          masks[code >> 5] |= 1 << (code & 0x1f);
        }
    }

  /* The switches belong to the analog function controller */

  modifyreg32(N32_RCC_APB5EN1, 0, RCC_APB5EN1_M7AFECEN);
  for (i = 0; i < 4; i++)
    {
      if (masks[i] != 0)
        {
          modifyreg32(g_adc_anaenreg[i], 0, masks[i]);
        }
    }
}

/****************************************************************************
 * Name: adc_set_sampt
 *
 * Description:
 *   Write the sampling times of the channels of the list.
 *
 ****************************************************************************/

static void adc_set_sampt(FAR struct n32_adc_s *priv)
{
  uint32_t samp[3] =
    {
      0, 0, 0
    };

  uint8_t ch;
  int i;

  for (i = 0; i < priv->cchannels; i++)
    {
      ch = priv->chanlist[i];
      samp[ch >> 3] |= (uint32_t)priv->samp[i] << ADC_SAMPT_SAMPX_SHIFT(ch);
    }

  adc_putreg(priv, N32_ADC_SAMPT1_OFFSET, samp[0]);
  adc_putreg(priv, N32_ADC_SAMPT2_OFFSET, samp[1]);
  adc_putreg(priv, N32_ADC_SAMPT3_OFFSET, samp[2]);
}

/****************************************************************************
 * Name: adc_set_sequence
 *
 * Description:
 *   Write the conversion sequence from the channel list.
 *
 ****************************************************************************/

static void adc_set_sequence(FAR struct n32_adc_s *priv)
{
  uint32_t rseq[4] =
    {
      0, 0, 0, 0
    };

  int i;

  /* The sequence starts in RSEQ4, six conversions per register, and the
   * length sits in RSEQ1.
   */

  for (i = 0; i < priv->cchannels; i++)
    {
      rseq[i / 6] |=
        (uint32_t)priv->chanlist[i] << ADC_RSEQ_SQx_SHIFT(i % 6 + 1);
    }

  rseq[3] |= (uint32_t)(priv->cchannels - 1) << ADC_RSEQ_LEN_SHIFT;

  for (i = 0; i < 4; i++)
    {
      adc_putreg(priv, N32_ADC_RSEQ4_OFFSET - 4 * i, rseq[i]);

      /* The registers cross into the kernel clock domain: two writes need
       * a minimum time in between.
       */

      up_udelay(1);
    }

  priv->rnchannels = priv->cchannels;
  priv->current    = 0;
}

/****************************************************************************
 * Name: adc_timinit
 *
 * Description:
 *   Set the timer that triggers the ADC to the sampling frequency.
 *
 ****************************************************************************/

static int adc_timinit(FAR struct n32_adc_s *priv)
{
  FAR const struct n32_adc_tim_s *tim = priv->tim;
  uint32_t base = tim->base;
  uint32_t prescaler;
  uint32_t reload;
  uint32_t mmsel;
  uint32_t extsel;
  unsigned int ch = priv->trigger;

  if (priv->freq == 0 || ch > 5)
    {
      aerr("ERROR: ADC%d: the timer has no trigger %d\n",
           priv->intf, priv->trigger);
      return -EINVAL;
    }

  /* Conversions start on the rising edge of the master output of the
   * timer: TRGO at the end of every period for trigger 4, TRGO2 for
   * trigger 5, and for a compare match, triggers 0 to 3, the output of
   * channel trigger + 1, which goes high in the middle of the period.
   * The inputs of the converter for compare matches did not react to
   * one on the N32H760, and the TRGO of a GTIMA has no compare source.
   */

  if (ch < 4)
    {
      extsel = tim->trgo;
    }
  else
    {
      extsel = ch == 4 ? tim->trgo : tim->trgo2;
    }

  if (extsel == ADC_NOTRIG || (ch < 4 && tim->type == ADC_TIM_GTIMA))
    {
      aerr("ERROR: ADC%d: the timer has no trigger %d\n",
           priv->intf, priv->trigger);
      return -EINVAL;
    }

  adc_modreg(priv, N32_ADC_CTRL2_OFFSET,
             ADC_CTRL2_EXTPRSEL_MASK | ADC_CTRL2_EXTRSEL_MASK,
             ADC_CTRL2_EXTPRSEL_RISE |
             (uint32_t)extsel << ADC_CTRL2_EXTRSEL_SHIFT);

  modifyreg32(tim->rccenr, 0, tim->rccen);

  /* A period is timclk / freq timer clocks long.  Take the smallest
   * prescaler that keeps it in the 16 bit counter.
   */

  prescaler = (N32_AHB_FREQUENCY / priv->freq + 65534) / 65535;
  if (prescaler < 1)
    {
      prescaler = 1;
    }
  else if (prescaler > 65536)
    {
      prescaler = 65536;
    }

  reload = N32_AHB_FREQUENCY / prescaler / priv->freq;
  if (reload < 1)
    {
      reload = 1;
    }
  else if (reload > 65536)
    {
      reload = 65536;
    }

  /* Stop the counter and let it count up with every timer clock */

  putreg32(0, base + N32_ATIM_CTRL1_OFFSET);
  putreg32(prescaler - 1, base + N32_ATIM_PSC_OFFSET);
  putreg32(reload - 1, base + N32_ATIM_AR_OFFSET);

  /* The compare registers have the same layout for every timer type.
   * PWM mode 2 goes high at the compare value.
   */

  if (ch < 4)
    {
      modifyreg32(base + N32_ATIM_CCMOD1_OFFSET + 4 * (ch >> 1),
                  (N32_ATIM_CCMOD1_OC1MD_MASK |
                   N32_ATIM_CCMOD1_CC1SEL_MASK) << (8 * (ch & 1)),
                  (N32_ATIM_CCMOD1_OCMODE_PWM2 |
                   N32_ATIM_CCMOD1_OC1PEN) << (8 * (ch & 1)));
      putreg32(reload / 2, base + N32_ATIM_CCDAT1_OFFSET + 4 * ch);
      modifyreg32(base + N32_ATIM_CCEN_OFFSET, 0,
                  N32_ATIM_CCEN_CC1EN << (4 * ch));
    }

  /* What the master output carries.  Only the master mode field of TRGO
   * sits elsewhere in the GTIMAs.
   */

  if (ch < 4)
    {
      mmsel = (tim->type == ADC_TIM_GTIMB ? N32_GTIMB_CTRL2_MMSEL_OC1REF :
               N32_ATIM_CTRL2_MMSEL_OC1REF) +
              (ch << N32_ATIM_CTRL2_MMSEL_SHIFT);
    }
  else if (ch == 4)
    {
      mmsel = tim->type == ADC_TIM_GTIMA ? N32_GTIMA_CTRL2_MMSEL_UPDATE :
              tim->type == ADC_TIM_GTIMB ? N32_GTIMB_CTRL2_MMSEL_UPDATE :
              N32_ATIM_CTRL2_MMSEL_UPDATE;
    }
  else
    {
      mmsel = N32_ATIM_CTRL2_MMSEL2_UPDATE;
    }

  modifyreg32(base + N32_ATIM_CTRL2_OFFSET,
              ch == 5 ? N32_ATIM_CTRL2_MMSEL2_MASK :
              N32_ATIM_CTRL2_MMSEL_MASK, mmsel);

  /* Take the prescaler and the reload value */

  putreg32(N32_ATIM_ETGEN_UDGN, base + N32_ATIM_ETGEN_OFFSET);
  putreg32(0, base + N32_ATIM_STS_OFFSET);
  return OK;
}

/****************************************************************************
 * Name: adc_timstart
 *
 * Description:
 *   Start or stop the counter of the trigger timer.
 *
 ****************************************************************************/

static void adc_timstart(FAR struct n32_adc_s *priv, bool enable)
{
  modifyreg32(priv->tim->base + N32_ATIM_CTRL1_OFFSET, N32_ATIM_CTRL1_CNTEN,
              enable ? N32_ATIM_CTRL1_CNTEN : 0);
}

/****************************************************************************
 * Name: adc_startconv
 *
 * Description:
 *   Start converting, or stop.  A software trigger starts one sequence.
 *   With a timer every trigger starts one for as long as the ADC is not
 *   stopped.
 *
 ****************************************************************************/

static void adc_startconv(FAR struct n32_adc_s *priv, bool enable)
{
  if (!enable)
    {
      adc_modreg3(priv, 0, ADC_CTRL3_SWRSTOP);
    }
  else if (priv->tim == NULL)
    {
      /* The flags of the previous sequence have to go before a new one is
       * started.  They stay set while a sequence still runs, which is
       * also when another trigger would be ignored.
       */

      if ((adc_getreg(priv, N32_ADC_CTRL3_OFFSET) & ADC_CTRL3_RSTART) == 0)
        {
          adc_clear(priv, ADC_STS_CLRMASK);
          adc_modreg3(priv, 0, ADC_CTRL3_RSTART);
          adc_modreg(priv, N32_ADC_CTRL2_OFFSET, 0, ADC_CTRL2_SWSTRRCH);
        }
    }
  else
    {
      adc_modreg3(priv, 0, ADC_CTRL3_RSTART);
    }
}

/****************************************************************************
 * Name: adc_enable
 *
 * Description:
 *   Switch the ADC on and calibrate it.
 *
 ****************************************************************************/

static int adc_enable(FAR struct n32_adc_s *priv)
{
  int timeout;

  adc_modreg(priv, N32_ADC_CTRL2_OFFSET, 0, ADC_CTRL2_ON);

  for (timeout = 0; timeout < ADC_READY_LOOPS; timeout++)
    {
      if ((adc_getreg(priv, N32_ADC_STS_OFFSET) & ADC_STS_RDYF) != 0)
        {
          break;
        }

      up_udelay(ADC_READY_DELAY);
    }

  if (timeout >= ADC_READY_LOOPS)
    {
      aerr("ERROR: ADC%d did not get ready\n", priv->intf);
      return -EIO;
    }

#ifndef CONFIG_N32H7_ADC_BYPASSCAL
  adc_modreg(priv, N32_ADC_CTRL2_OFFSET, 0, ADC_CTRL2_ENCAL);

  for (timeout = 0; timeout < ADC_READY_LOOPS; timeout++)
    {
      if ((adc_getreg(priv, N32_ADC_CTRL2_OFFSET) & ADC_CTRL2_ENCAL) == 0)
        {
          break;
        }

      up_udelay(ADC_READY_DELAY);
    }

  if (timeout >= ADC_READY_LOOPS)
    {
      aerr("ERROR: ADC%d calibration did not end\n", priv->intf);
      return -EIO;
    }
#endif

  adc_putreg(priv, N32_ADC_STS_OFFSET,
             ADC_STS_CLRMASK | ADC_STS_RDYF | ADC_STS_WEFLAG);
  return OK;
}

/****************************************************************************
 * Name: adc_awd23_event
 *
 * Description:
 *   Look for an interrupting watchdog 2 or 3 event and acknowledge it.
 *
 * Returned Value:
 *   The watchdog that saw one, 0 if none did.
 *
 ****************************************************************************/

static uint8_t adc_awd23_event(FAR struct n32_adc_s *priv)
{
  uint32_t flags;
  int i;

  for (i = 0; i < 2; i++)
    {
      if (priv->awd[i].inten != 0)
        {
          flags = adc_getreg(priv, N32_ADC_AWD2STS_OFFSET + 4 * i) &
                  priv->awd[i].inten;
          if (flags != 0)
            {
              adc_putreg(priv, N32_ADC_AWD2STS_OFFSET + 4 * i, flags);
              return i + 2;
            }
        }
    }

  return 0;
}

/****************************************************************************
 * Name: adc_wdog_enable
 *
 * Description:
 *   Have watchdog 1 watch every conversion from now on.  As the STM32H7
 *   driver does, switch to continuous conversions so that one trigger is
 *   enough, and no result reaches the upper half again until the device
 *   is opened again.  The ADC cannot be told to overwrite unread results,
 *   so the end of conversion interrupt stays on and throws them away.
 *
 ****************************************************************************/

static void adc_wdog_enable(FAR struct n32_adc_s *priv)
{
  priv->watching = true;
  priv->ie = ADC_CTRL1_AWD1IEN | ADC_CTRL1_WEIEN;
#ifdef ADC_HAVE_DMA
  if (priv->dma == NULL)
#endif
    {
      priv->ie |= ADC_CTRL1_ENDCAIEN;
    }

  adc_modreg(priv, N32_ADC_CTRL1_OFFSET, ADC_CTRL1_INTMASK, priv->ie);
  adc_modreg(priv, N32_ADC_CTRL1_OFFSET,
             ADC_CTRL1_AWD1SGLEN | ADC_CTRL1_AWD1CH_MASK,
             ADC_CTRL1_AWD1ERCH);
  adc_modreg(priv, N32_ADC_CTRL2_OFFSET, ADC_CTRL2_CTU, ADC_CTRL2_CTU);
}

#ifdef ADC_HAVE_DMA
/****************************************************************************
 * Name: adc_dmastart
 *
 * Description:
 *   Let the DMA channel collect the next batch of sequences.
 *
 ****************************************************************************/

static void adc_dmastart(FAR struct adc_dev_s *dev)
{
  FAR struct n32_adc_s *priv = dev->ad_priv;
  n32_dmacfg_t cfg;

  memset(&cfg, 0, sizeof(cfg));
  cfg.src_addr   = priv->base + N32_ADC_DAT_OFFSET;
  cfg.dst_addr   = (uint32_t)(uintptr_t)priv->dmabuffer;
  cfg.block_size = priv->rnchannels * priv->dmabatch;
  cfg.ctrl       = DMA_CHCTRL_INTEN |
                   DMA_CHCTRL_DTW(DMA_WIDTH_16BITS) |
                   DMA_CHCTRL_STW(DMA_WIDTH_16BITS) |
                   DMA_CHCTRL_DINC(DMA_ADDR_INC) |
                   DMA_CHCTRL_SINC(DMA_ADDR_NOCHANGE) |
                   DMA_CHCTRL_DSTMSIZE(DMA_BURST_1) |
                   DMA_CHCTRL_SRCMSIZE(DMA_BURST_1) |
                   DMA_CHCTRL_SMS(DMA_MS_1) |
                   DMA_CHCTRL_DMS(DMA_MS_1) |
                   DMA_CHCTRL_TTFC(DMA_TTFC_PER_TO_MEM_DMA);
  cfg.src_hs_if  = n32_getchannel(priv->dma);
  cfg.dst_hs_if  = n32_getchannel(priv->dma);

  n32_dmasetup(priv->dma, &cfg);
  n32_dmastart(priv->dma, adc_dma_callback, dev);
}

/****************************************************************************
 * Name: adc_dma_callback
 *
 * Description:
 *   Hand a batch the DMA collected to the upper half and start the next.
 *
 ****************************************************************************/

static void adc_dma_callback(DMA_HANDLE handle, uint8_t status,
                             FAR void *arg)
{
  FAR struct adc_dev_s *dev = arg;
  FAR struct n32_adc_s *priv = dev->ad_priv;
  unsigned int n = priv->rnchannels * priv->dmabatch;
  unsigned int i;

  UNUSED(handle);

  if ((status & DMA_INT_ERROR) != 0)
    {
      aerr("ERROR: ADC%d DMA failed: 0x%02x\n", priv->intf, status);
      adc_startconv(priv, false);
      if (priv->cb != NULL && priv->cb->au_reset != NULL)
        {
          priv->cb->au_reset(dev);
        }

      return;
    }

  if ((status & DMA_INT_TFR) == 0)
    {
      return;
    }

  if (!priv->watching && priv->cb != NULL &&
      priv->cb->au_receive != NULL)
    {
#ifdef CONFIG_ARMV7M_DCACHE
      up_invalidate_dcache((uintptr_t)priv->dmabuffer,
                           (uintptr_t)(priv->dmabuffer + n));
#endif

      for (i = 0; i < n; i++)
        {
          priv->cb->au_receive(dev, priv->chanlist[priv->current],
                               priv->dmabuffer[i]);
          priv->current++;
          if (priv->current >= priv->rnchannels)
            {
              priv->current = 0;
            }
        }
    }

  /* Collect the next batch */

  adc_dmastart(dev);
}
#endif

/****************************************************************************
 * Name: adc_overrun
 *
 * Description:
 *   Results were lost: the ADC keeps the oldest ones and drops the newer.
 *   Where the loss began is unknown, so the results still waiting cannot be
 *   given to their channels.  Stop, tell the upper half, which fails the
 *   next read with EIO, and start over with the first channel of the list.
 *   With the software trigger the sequence is run again, else a read would
 *   wait for good.
 *
 ****************************************************************************/

static void adc_overrun(FAR struct adc_dev_s *dev)
{
  FAR struct n32_adc_s *priv = dev->ad_priv;
  int timeout;

  /* A message from here would hold the interrupts off for as long as
   * the line takes to send, and lose what is left of the FIFO at high
   * rates: the count is reported when the device is closed.
   */

  priv->overruns++;
  adc_startconv(priv, false);

  /* The stop crosses into the kernel clock domain */

  for (timeout = 0; timeout < ADC_READY_LOOPS; timeout++)
    {
      if ((adc_getreg(priv, N32_ADC_CTRL3_OFFSET) & ADC_CTRL3_RSTART) == 0)
        {
          break;
        }

      up_udelay(1);
    }

  adc_clear(priv, ADC_STS_CLRMASK | ADC_STS_WEFLAG);
  priv->current = 0;
  if (priv->cb != NULL && priv->cb->au_reset != NULL)
    {
      priv->cb->au_reset(dev);
    }

#ifdef ADC_HAVE_DMA
  /* The batch the DMA was collecting is incomplete */

  if (priv->dma != NULL)
    {
      n32_dmastop(priv->dma);
    }
#endif

  /* Starting a sequence discards the results still waiting, which the
   * DMA must not move into the next batch: it starts after them.
   */

  adc_startconv(priv, true);

#ifdef ADC_HAVE_DMA
  if (priv->dma != NULL)
    {
      adc_dmastart(dev);
    }
#endif
}

/****************************************************************************
 * Name: adc_interrupt
 *
 * Description:
 *   The ADC interrupt.  Results are counted off the channel list in the
 *   order in which they were converted.
 *
 ****************************************************************************/

static int adc_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct adc_dev_s *dev = arg;
  FAR struct n32_adc_s *priv = dev->ad_priv;
  uint32_t ctrl1;
  uint32_t pending;
  uint32_t sts;
  uint32_t value;
  unsigned int n;
  uint8_t wdog;

  UNUSED(irq);
  UNUSED(context);

  /* Results that end while the flags of the last ones are being cleared
   * would not interrupt again, see adc_clear: clear, then look again
   * until nothing more is waiting.
   */

  for (; ; )
    {
      ctrl1   = adc_getreg(priv, N32_ADC_CTRL1_OFFSET);
      sts     = adc_getreg(priv, N32_ADC_STS_OFFSET);

      pending = sts & (ADC_STS_STR | ADC_STS_EOSAMP | ADC_STS_REFLAG);

      if ((ctrl1 & ADC_CTRL1_AWD1IEN) != 0 && (sts & ADC_STS_AWDG1) != 0)
        {
          wdog = 1;
          pending |= ADC_STS_AWDG1;
        }
      else
        {
          wdog = adc_awd23_event(priv);
        }

      n = 0;
      if (wdog != 0)
        {
          /* Stop converting, or the flag would interrupt for good */

          awarn("WARNING: ADC%d watchdog %u: conversion out of range\n",
                priv->intf, (unsigned)wdog);
          adc_startconv(priv, false);
        }
      else if ((ctrl1 & ADC_CTRL1_WEIEN) != 0 &&
               (sts & ADC_STS_WEFLAG) != 0)
        {
          adc_overrun(dev);
          break;
        }
      else
        {
          n = (adc_getreg(priv, N32_ADC_FIFOSTS_OFFSET) &
               ADC_FIFOSTS_DATCNT_MASK) >> ADC_FIFOSTS_DATCNT_SHIFT;

          while (n-- > 0)
            {
              value = adc_getreg(priv, N32_ADC_DAT_OFFSET) & ADC_DAT_DAT;
              if (!priv->watching && priv->cb != NULL &&
                  priv->cb->au_receive != NULL)
                {
                  priv->cb->au_receive(dev, priv->chanlist[priv->current],
                                       value);
                }

              priv->current++;
              if (priv->current >= priv->rnchannels)
                {
                  priv->current = 0;
                }
            }

          pending |= ADC_STS_ENDC | ADC_STS_ENDCA;
        }

      if (pending != 0)
        {
          adc_clear(priv, pending);
        }
      else
        {
          up_udelay(priv->syncus);
        }

      /* A conversion that ended since the count was read is in the FIFO
       * now, and its flag went with the clear.
       */

      if (n == 0 || (adc_getreg(priv, N32_ADC_FIFOSTS_OFFSET) &
                     ADC_FIFOSTS_DATCNT_MASK) == 0)
        {
          break;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: adc_bind
 *
 * Description:
 *   Bind the upper half callbacks to the lower half implementation.
 *
 ****************************************************************************/

static int adc_bind(FAR struct adc_dev_s *dev,
                    FAR const struct adc_callback_s *callback)
{
  FAR struct n32_adc_s *priv = dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: adc_reset
 *
 * Description:
 *   Reset the ADC.  The upper half calls it when the device is registered,
 *   the driver calls it when the device is opened, before it configures
 *   the ADC.
 *
 ****************************************************************************/

static void adc_reset(FAR struct adc_dev_s *dev)
{
  FAR struct n32_adc_s *priv = dev->ad_priv;

  DEBUGASSERT(priv != NULL);

  adc_rccreset(priv, true);
  adc_rccreset(priv, false);

  /* A reset of ADC1 switches off the internal sources ADC3 converts */

#ifdef CONFIG_N32H7_ADC3
  if (priv->intf == 1 && g_adcpriv3.opened)
    {
      adc_internal_power(g_adcpriv3.internal, true);
    }
#endif
}

/****************************************************************************
 * Name: adc_setup
 *
 * Description:
 *   Configure the ADC.  The upper half calls it the first time the device
 *   is opened, with interrupts disabled.
 *
 ****************************************************************************/

static int adc_setup(FAR struct adc_dev_s *dev)
{
  FAR struct n32_adc_s *priv = dev->ad_priv;
  irqstate_t flags;
  uint32_t dmngt = ADC_CTRL2_DMNGT_DR;
  uint32_t ctrl3;
  bool hasdma = false;
  int ret;
  int i;

  DEBUGASSERT(priv != NULL);

  /* Attach and enable the ADC interrupt.  It fires once ao_rxint() lets
   * the ADC raise it.
   */

  ret = irq_attach(priv->irq, adc_interrupt, dev);
  if (ret < 0)
    {
      aerr("ERROR: irq_attach failed: %d\n", ret);
      return ret;
    }

  up_enable_irq(priv->irq);

  flags = enter_critical_section();

#ifdef ADC_HAVE_DMA
  if (priv->dmamap != 0)
    {
      if (priv->cchannels * priv->dmabatch > 4095)
        {
          aerr("ERROR: ADC%d: the DMA batch is too large\n", priv->intf);
          ret = -EINVAL;
          goto errout;
        }

      hasdma = true;
      dmngt  = ADC_CTRL2_DMNGT_DMA;
    }
#endif

  /* Start from the reset state of the ADC and clock it */

  adc_reset(dev);
  adc_clock(priv);
  adc_anaen(priv);

  /* Resolution, kernel clock, differential channels, gain compensation */

  ctrl3 = priv->res |
          (priv->pll == ADC_CLK_AHB ? 0 : ADC_CTRL3_CKMOD) |
          (priv->gcompdat == 0 ? 0 : ADC_CTRL3_GCOMPEN);
#ifdef CONFIG_N32H7_ADC_BYPASSCAL
  ctrl3 |= ADC_CTRL3_BPCAL;
#endif

  adc_modreg3(priv, ADC_CTRL3_RES_12BIT | ADC_CTRL3_CKMOD |
                 ADC_CTRL3_BPCAL | ADC_CTRL3_GCOMPEN, ctrl3);
  adc_putreg(priv, N32_ADC_DIFSEL_OFFSET, priv->difsel);
  adc_putreg(priv, N32_ADC_GCOMP_OFFSET, priv->gcompdat);

  /* Oversampling and delayed sampling */

  adc_putreg(priv, N32_ADC_OSCFG_OFFSET, priv->oscfg);
  adc_modreg(priv, N32_ADC_DLYSMP_OFFSET,
             ADC_DLYSMP_DLYSMPEN | ADC_DLYSMP_DLYVAL_MASK, priv->dlysmp);

  /* Sampling times, offsets and the sequence */

  adc_set_sampt(priv);
  for (i = 0; i < 4; i++)
    {
      adc_putreg(priv, N32_ADC_OFFSET1_OFFSET + 4 * i, priv->offset[i]);
    }

  adc_set_sequence(priv);

  /* Watchdogs */

  adc_putreg(priv, N32_ADC_AWD1HIGH_OFFSET, priv->awd1high);
  adc_putreg(priv, N32_ADC_AWD1LOW_OFFSET, priv->awd1low);
  for (i = 0; i < 2; i++)
    {
      adc_putreg(priv, N32_ADC_AWD2HIGH_OFFSET + 4 * i, priv->awd[i].high);
      adc_putreg(priv, N32_ADC_AWD2LOW_OFFSET + 4 * i, priv->awd[i].low);
      adc_putreg(priv, N32_ADC_AWD2EN_OFFSET + 4 * i,
                 priv->awd[i].channels);
      adc_putreg(priv, N32_ADC_AWD2INTEN_OFFSET + 4 * i,
                 priv->awd[i].inten);
    }

  /* The interrupts come with ao_rxint().  A sequence that fits into the
   * FIFO interrupts once at its end, a longer one after every conversion.
   */

  priv->watching = false;
  priv->overruns = 0;
  priv->ie = (priv->awd1ctrl & ADC_CTRL1_AWD1IEN) | ADC_CTRL1_WEIEN;
  if (!hasdma)
    {
      priv->ie |= priv->rnchannels <= ADC_FIFO_DEPTH ? ADC_CTRL1_ENDCIEN :
                  ADC_CTRL1_ENDCAIEN;
    }

  adc_putreg(priv, N32_ADC_CTRL1_OFFSET,
             (priv->cchannels > 1 ? ADC_CTRL1_SCANMD : 0) | priv->awd1ctrl);

  /* The results go to the data register through the FIFO, which the CPU
   * empties or the DMA: it asks for a result as soon as one is in the
   * FIFO, whose rest bridges the time between two of its batches.  A
   * timer selects its own trigger, else the software triggers.
   */

  adc_modreg(priv, N32_ADC_CTRL2_OFFSET, ADC_CTRL2_DMNGT_MASK, dmngt);
  if (priv->tim == NULL)
    {
      adc_modreg(priv, N32_ADC_CTRL2_OFFSET, ADC_CTRL2_EXTPRSEL_MASK,
                 ADC_CTRL2_EXTPRSEL_SW);
    }

  adc_modreg(priv, N32_ADC_FIFOCFG_OFFSET,
             ADC_FIFOCFG_EN | ADC_FIFOCFG_WL_MASK, ADC_FIFOCFG_EN);

  /* The internal sources of ADC3 */

  adc_internal_power(priv->internal, true);

  /* Switch on and calibrate */

  ret = adc_enable(priv);
  if (ret < 0)
    {
      goto errout;
    }

#ifdef ADC_HAVE_DMA
  if (hasdma)
    {
      priv->dma = n32_dmachannel(priv->dmamap);
      if (priv->dma == NULL)
        {
          aerr("ERROR: ADC%d: no DMA channel is free\n", priv->intf);
          ret = -ENOMEM;
          goto errout;
        }

      adc_dmastart(dev);
    }
#endif

  /* The timer starts sequences on its own from now on */

  if (priv->tim != NULL)
    {
      ret = adc_timinit(priv);
      if (ret < 0)
        {
          goto errout;
        }

      adc_startconv(priv, true);
      adc_timstart(priv, true);
    }

  priv->opened = true;
  leave_critical_section(flags);
  return OK;

errout:

  /* Leave everything as the shutdown would */

  leave_critical_section(flags);
  aerr("ERROR: ADC%d: setup failed: %d\n", priv->intf, ret);
  adc_shutdown(dev);
  return ret;
}

/****************************************************************************
 * Name: adc_shutdown
 *
 * Description:
 *   Disable the ADC.  The upper half calls it when the device is closed.
 *
 ****************************************************************************/

static void adc_shutdown(FAR struct adc_dev_s *dev)
{
  FAR struct n32_adc_s *priv = dev->ad_priv;
  int timeout;

  DEBUGASSERT(priv != NULL);

  /* Stop converting and stop the timer */

  adc_startconv(priv, false);
  if (priv->tim != NULL)
    {
      adc_timstart(priv, false);
      modifyreg32(priv->tim->rccenr, priv->tim->rccen, 0);
    }

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);

#ifdef ADC_HAVE_DMA
  if (priv->dma != NULL)
    {
      n32_dmastop(priv->dma);
      n32_dmafree(priv->dma);
      priv->dma = NULL;
    }
#endif

  if (priv->overruns != 0)
    {
      awarn("WARNING: ADC%d lost %lu conversions while it was open\n",
            priv->intf, (unsigned long)priv->overruns);
    }

  /* Power the ADC down and switch the internal sources off */

  adc_modreg(priv, N32_ADC_CTRL2_OFFSET, ADC_CTRL2_ON, 0);
  for (timeout = 0; timeout < ADC_READY_LOOPS; timeout++)
    {
      if ((adc_getreg(priv, N32_ADC_STS_OFFSET) & ADC_STS_PDRDYF) != 0)
        {
          break;
        }

      up_udelay(ADC_READY_DELAY);
    }

  priv->opened = false;
  adc_internal_power(priv->internal, false);
  adc_reset(dev);
}

/****************************************************************************
 * Name: adc_rxint
 *
 * Description:
 *   Enable or disable the ADC interrupt.
 *
 ****************************************************************************/

static void adc_rxint(FAR struct adc_dev_s *dev, bool enable)
{
  FAR struct n32_adc_s *priv = dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  adc_modreg(priv, N32_ADC_CTRL1_OFFSET, ADC_CTRL1_INTMASK,
             enable ? priv->ie : 0);
}

/****************************************************************************
 * Name: adc_ioctl
 *
 * Description:
 *   The upper half passes every ioctl it does not handle itself on to
 *   here.
 *
 ****************************************************************************/

static int adc_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct n32_adc_s *priv = dev->ad_priv;
  uint32_t other;
  int ret = OK;

  DEBUGASSERT(priv != NULL);

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        {
          /* Start one sequence.  A timer starts them on its own. */

          if (priv->tim == NULL)
            {
              adc_startconv(priv, true);
            }

          break;
        }

      case ANIOC_GET_NCHANNELS:
        {
          ret = priv->cchannels;
          break;
        }

      case ANIOC_WDOG_UPPER:     /* Set watchdog upper threshold */
        {
          other = priv->awd1low & ADC_AWDxLOW_LTH;
          if (arg > ADC_THRESHOLD_MAX || arg < other)
            {
              ret = -EINVAL;
              break;
            }

          priv->awd1high = arg;
          adc_putreg(priv, N32_ADC_AWD1HIGH_OFFSET, priv->awd1high);
          adc_wdog_enable(priv);
          break;
        }

      case ANIOC_WDOG_LOWER:     /* Set watchdog lower threshold */
        {
          other = priv->awd1high;
          if (arg > ADC_THRESHOLD_MAX || arg > other)
            {
              ret = -EINVAL;
              break;
            }

          priv->awd1low = (priv->awd1low & ADC_AWD1LOW_AWDFIL_MASK) | arg;
          adc_putreg(priv, N32_ADC_AWD1LOW_OFFSET, priv->awd1low);
          adc_wdog_enable(priv);
          break;
        }

      default:
        {
          ret = -ENOTTY;
          break;
        }
    }

  return ret;
}

#ifdef CONFIG_PM
/****************************************************************************
 * Name: adc_pm_prepare
 *
 * Description:
 *   Called by the power management framework when it wants to enter a low
 *   power state.  Refuse while a conversion is running.
 *
 ****************************************************************************/

static int adc_pm_prepare(FAR struct pm_callback_s *cb, int domain,
                          enum pm_state_e state)
{
  FAR struct n32_adc_s *priv =
    container_of(cb, struct n32_adc_s, pm_callback);

  UNUSED(domain);

  if (state >= PM_IDLE &&
      (adc_getreg(priv, N32_ADC_CTRL3_OFFSET) & ADC_CTRL3_RSTART) != 0)
    {
      return -EBUSY;
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_adc_initialize
 *
 * Description:
 *   Prepare ADC intf to sample a list of channels.  The upper half is bound
 *   with adc_register().
 *
 ****************************************************************************/

FAR struct adc_dev_s *n32_adc_initialize(int intf,
                                         FAR const uint8_t *chanlist,
                                         int nchannels)
{
  FAR struct adc_dev_s *dev;
  FAR struct n32_adc_s *priv;
  int code;
  int i;

  switch (intf)
    {
#ifdef CONFIG_N32H7_ADC1
      case 1:
        {
          dev = &g_adcdev1;
          break;
        }
#endif

#ifdef CONFIG_N32H7_ADC2
      case 2:
        {
          dev = &g_adcdev2;
          break;
        }
#endif

#ifdef CONFIG_N32H7_ADC3
      case 3:
        {
          dev = &g_adcdev3;
          break;
        }
#endif

      default:
        {
          aerr("ERROR: No ADC interface defined\n");
          return NULL;
        }
    }

  priv = dev->ad_priv;

  /* The list is what the device converts from its next opening on */

  if (chanlist == NULL || nchannels <= 0)
    {
      aerr("ERROR: ADC%d has no channels to convert\n", intf);
      return NULL;
    }

  code = adc_sampcode(CONFIG_N32H7_ADC_SAMPLETIME);
  if (code < 0)
    {
      aerr("ERROR: CONFIG_N32H7_ADC_SAMPLETIME is not a sampling time\n");
      return NULL;
    }

  if (nchannels > CONFIG_N32H7_ADC_MAX_SAMPLES)
    {
      nchannels = CONFIG_N32H7_ADC_MAX_SAMPLES;
    }

  priv->cchannels = nchannels;
  priv->internal  = 0;

  for (i = 0; i < nchannels; i++)
    {
      if (chanlist[i] > N32_ADC_VREFINT_CH)
        {
          aerr("ERROR: ADC%d has no channel %u\n", intf,
               (unsigned)chanlist[i]);
          return NULL;
        }

      priv->chanlist[i] = chanlist[i];

      /* The battery monitor, the temperature sensor and VREFINT are
       * channels 17 to 19 of ADC3.  They sample for the longest time the
       * ADC offers, the temperature sensor wants 17.1 us.
       */

      if (intf == 3 && chanlist[i] >= N32_ADC_VBAT_CH)
        {
          priv->internal |= 1 << (chanlist[i] - N32_ADC_VBAT_CH);
          priv->samp[i]   = 15;
        }
      else
        {
          priv->samp[i]   = code;
        }
    }

#ifdef CONFIG_PM
  if (pm_register(&priv->pm_callback) != OK)
    {
      aerr("ERROR: Power management registration failed\n");
      return NULL;
    }
#endif

  return dev;
}

/****************************************************************************
 * Name: n32_adc_setoffset
 *
 * Description:
 *   Add or subtract a value from the conversions of one channel.  See
 *   n32_adc.h.
 *
 ****************************************************************************/

int n32_adc_setoffset(FAR struct adc_dev_s *dev, uint8_t n, uint8_t ch,
                      uint16_t value, bool positive, bool saturate)
{
  FAR struct n32_adc_s *priv = dev->ad_priv;

  DEBUGASSERT(dev != NULL);
  if (n < 1 || n > 4 || ch > N32_ADC_VREFINT_CH ||
      value > ADC_OFFSET_OFFSCHxDAT)
    {
      return -EINVAL;
    }

  priv->offset[n - 1] = ADC_OFFSET_OFFSCHxEN |
                        ((uint32_t)ch << ADC_OFFSET_OFFSCHxCH_SHIFT) |
                        (positive ? ADC_OFFSET_OFFSCHxDIR : 0) |
                        (saturate ? ADC_OFFSET_OFFSCHxSATEN : 0) |
                        value;
  return OK;
}

/****************************************************************************
 * Name: n32_adc_setsampletime
 *
 * Description:
 *   Set the sampling time of one channel of the list.  See n32_adc.h.
 *
 ****************************************************************************/

int n32_adc_setsampletime(FAR struct adc_dev_s *dev, uint8_t ch,
                          uint16_t cycles)
{
  FAR struct n32_adc_s *priv = dev->ad_priv;
  int code;
  int i;

  DEBUGASSERT(dev != NULL);
  code = adc_sampcode(cycles);
  if (code < 0)
    {
      return -EINVAL;
    }

  for (i = 0; i < priv->cchannels; i++)
    {
      if (priv->chanlist[i] == ch)
        {
          priv->samp[i] = code;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: n32_adc_setwatchdog
 *
 * Description:
 *   Have a watchdog watch channels.  See n32_adc.h.
 *
 ****************************************************************************/

int n32_adc_setwatchdog(FAR struct adc_dev_s *dev, uint8_t n,
                        FAR const struct n32_adc_wdog_s *wdog, uint8_t ch)
{
  FAR struct n32_adc_s *priv = dev->ad_priv;
  uint32_t channels;

  DEBUGASSERT(dev != NULL && wdog != NULL);
  if (n < 1 || n > 3 || ch > N32_ADC_VREFINT_CH ||
      wdog->upper > ADC_THRESHOLD_MAX || wdog->lower > ADC_THRESHOLD_MAX ||
      wdog->upper < wdog->lower ||
      (wdog->channels & ~(ADC_AWDx_CHANNELMASK | N32_ADC_WDOG_ALL |
                          N32_ADC_WDOG_SINGLE)) != 0)
    {
      return -EINVAL;
    }

  channels = wdog->channels & ADC_AWDx_CHANNELMASK;

  if (n == 1)
    {
      priv->awd1high = wdog->upper;
      priv->awd1low  = (priv->awd1low & ADC_AWD1LOW_AWDFIL_MASK) |
                       wdog->lower;
      priv->awd1ctrl = 0;
      if ((wdog->channels & N32_ADC_WDOG_SINGLE) != 0)
        {
          priv->awd1ctrl = ADC_CTRL1_AWD1SGLEN |
                           ((uint32_t)ch << ADC_CTRL1_AWD1CH_SHIFT);
        }
      else if ((wdog->channels & N32_ADC_WDOG_ALL) != 0)
        {
          /* Every converted channel */
        }
      else
        {
          return OK;
        }

      priv->awd1ctrl |= ADC_CTRL1_AWD1ERCH |
                        (wdog->events ? ADC_CTRL1_AWD1IEN : 0);
    }
  else
    {
      /* Watchdogs 2 and 3 watch the channels of the set, an empty set
       * switches them off.
       */

      if ((wdog->channels & N32_ADC_WDOG_ALL) != 0)
        {
          channels = ADC_AWDx_CHANNELMASK;
        }

      priv->awd[n - 2].high     = wdog->upper;
      priv->awd[n - 2].low      = wdog->lower;
      priv->awd[n - 2].channels = channels;
      priv->awd[n - 2].inten    = wdog->events ? channels : 0;
    }

  return OK;
}

#endif /* CONFIG_N32H7_ADC */
