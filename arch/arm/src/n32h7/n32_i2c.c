/****************************************************************************
 * arch/arm/src/n32h7/n32_i2c.c
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

/* N32H7 I2C master driver.
 *
 * The N32H7 I2C is the ST "I2C v2" IP under other names: the register
 * offsets and the positions of the common bits are those of the STM32
 * F0/F3/F7/H7/L4 I2C.  This driver is therefore a port of the NuttX ST v2
 * driver (arch/arm/src/common/stm32/stm32_i2c_m3m4_v2.c).
 *
 * Supports:
 *  - Single master operation:
 *      Standard-mode (up to 100 kHz)
 *      Fast-mode (up to 400 kHz)
 *      Fast-mode Plus (up to 1 MHz)
 *  - Multiple instances (shared bus)
 *  - Interrupt based operation, or polling with CONFIG_I2C_POLLED
 *  - REFILL support (messages longer than 255 bytes, I2C_M_NOSTART)
 *
 * Kernel clock:
 *
 *  The driver selects the divided system bus clock (N32_AHB_FREQUENCY) as
 *  the kernel clock source of its instance and sets the I2C prescaler of
 *  the APB group of the instance to /4, once per group.  The prescaler is
 *  shared by all instances of a group, so every instance of a group must
 *  use this source.  The kernel clock is therefore N32_AHB_FREQUENCY / 4
 *  and the BUSTM value is computed from it.
 *
 * Deliberate limitations:
 *
 *  - Zero-length messages (a NULL buffer or a length below 1) are
 *    rejected with -EINVAL.
 *  - 10-bit addressing is not implemented; I2C_M_TEN is rejected with
 *    -ENOTSUP.
 *  - I2C_M_NOSTOP on the last message is ignored, a STOP is always sent.
 *
 * Vendor sequence details that are kept:
 *
 *  STSINT.WRE is the "transmit data register empty" flag and a software
 *  reset clears it, after which WDR cannot be written.  After every START
 *  into a write message the driver therefore writes 1 to WRE to flush the
 *  transmit data register.  Receive sequences do not need it.
 *
 *  The reference manual says a STOP is sent automatically after a NACK.
 *  The N32H760 does not do that: BUSY stays set until the STOP bit is
 *  written, so the ISR requests the STOP itself.
 *
 * Configuration:
 *
 *  CONFIG_N32H7_I2C               - enables the driver
 *  CONFIG_N32H7_I2Cn (n = 1..8)   - enables instance n
 *
 *  Fixed wait timeout (CONFIG_N32H7_I2C_DYNTIMEO=n):
 *
 *  CONFIG_N32H7_I2CTIMEOMS        - timeout per message in milliseconds
 *  CONFIG_N32H7_I2CTIMEOTICKS     - the same in system timer ticks
 *
 *  Dynamic timeout (CONFIG_N32H7_I2C_DYNTIMEO=y):
 *
 *  CONFIG_N32H7_I2C_DYNTIMEO_USECPERBYTE - timeout per byte in microseconds
 *  CONFIG_N32H7_I2C_DYNTIMEO_STARTSTOP    - timeout for a pending STOP
 *
 * Debugging output is enabled with CONFIG_DEBUG_FEATURES and
 * CONFIG_DEBUG_I2C_{ERROR|WARN|INFO}.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/compiler.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "n32_gpio.h"
#include "n32_i2c.h"
#include "hardware/n32h7_i2c.h"
#include "hardware/n32h7_rcc.h"

/* At least one I2C peripheral must be enabled */

#if defined(CONFIG_N32H7_I2C1) || defined(CONFIG_N32H7_I2C2) || \
    defined(CONFIG_N32H7_I2C3) || defined(CONFIG_N32H7_I2C4) || \
    defined(CONFIG_N32H7_I2C5) || defined(CONFIG_N32H7_I2C6) || \
    defined(CONFIG_N32H7_I2C7) || defined(CONFIG_N32H7_I2C8)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_I2C_POLLED may be set so that I2C interrupts will not be used.
 * Instead, CPU-intensive polling will be used.
 */

/* Interrupt wait timeout in milliseconds and in system timer ticks */

#ifndef CONFIG_N32H7_I2CTIMEOMS
#  define CONFIG_N32H7_I2CTIMEOMS 500    /* Default is 500 milliseconds */
#endif

#ifndef CONFIG_N32H7_I2CTIMEOTICKS
#  define CONFIG_N32H7_I2CTIMEOTICKS MSEC2TICK(CONFIG_N32H7_I2CTIMEOMS)
#endif

/* I2C kernel clock: divided system bus clock through the /4 prescaler of
 * the APB group, see n32_i2c_kerclk().
 */

#define I2C_KERDIV           4
#define I2C_KERNEL_HZ        (N32_AHB_FREQUENCY / I2C_KERDIV)

/* Highest prescaled kernel clock the user manual allows for SCL
 * frequencies from 100 kHz to 1 MHz.
 */

#define I2C_FCKDIV_MAX       8000000

/* Smallest CKDIV (prescaler = CKDIV + 1) that brings the prescaled clock
 * down to I2C_FCKDIV_MAX, and the resulting prescaled clock.
 */

#define I2C_CKDIV            ((((I2C_KERNEL_HZ) + (I2C_FCKDIV_MAX) - 1) / \
                              (I2C_FCKDIV_MAX)) - 1)
#define I2C_FCKDIV           ((I2C_KERNEL_HZ) / ((I2C_CKDIV) + 1))

/* Slowest SCL frequency the fixed CKDIV can express: LSCL and HSCL hold
 * at most 256 units each, so a period is at most 512 units and
 * ceil(I2C_FCKDIV / frequency) must not exceed 512.
 */

#define I2C_FREQ_MIN         (((I2C_FCKDIV) + 511) / 512)

#if I2C_CKDIV > 15
#  error "N32H7 I2C: the kernel clock is too high for the 8 MHz prescaled clock limit"
#endif

/* Bound of the wait for a free bus before the first START of a transfer:
 * I2C_BUSY_LOOPS times I2C_BUSY_DELAY microseconds.
 */

#define I2C_BUSY_DELAY       10
#define I2C_BUSY_LOOPS       50

/* CTRL1 interrupt enable bits used by the driver */

#define I2C_CTRL1_TXRX       (I2C_CTRL1_WDRIE | I2C_CTRL1_RDRIE)
#define I2C_CTRL1_ALLINTS    (I2C_CTRL1_TXRX | I2C_CTRL1_NAKIE | \
                              I2C_CTRL1_TFCIE | I2C_CTRL1_ERRIE)

/* Unused bit in STSINT used to communicate that a bad state has occurred
 * in the ISR processing.
 */

#define I2C_INT_BAD_STATE    0x8000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Interrupt state */

enum n32_intstate_e
{
  INTSTATE_IDLE = 0,      /* No I2C activity */
  INTSTATE_WAITING,       /* Waiting for completion of interrupt activity */
  INTSTATE_DONE,          /* Interrupt activity complete */
};

/* RCC groups.  The instances of a group share one I2C kernel clock
 * prescaler.
 */

enum n32_i2c_group_e
{
  I2C_GROUP_APB1 = 0,     /* I2C1-3 */
  I2C_GROUP_APB2,         /* I2C4-6 */
  I2C_GROUP_APB5,         /* I2C7-8 */
  I2C_GROUP_NGROUPS
};

/* I2C Device hardware configuration */

struct n32_i2c_config_s
{
  uint32_t base;              /* I2C base address */
  uint8_t group;              /* RCC group, see enum n32_i2c_group_e */
  uint32_t sel_reg;           /* Kernel clock source select register */
  uint32_t sel_mask;          /* Kernel clock source select field */
  uint32_t sel_sysdiv;        /* Value selecting the divided system bus clock */
  uint32_t div_reg;           /* Kernel clock prescaler register of the group */
  uint32_t div_mask;          /* Kernel clock prescaler field */
  uint32_t div_div4;          /* Value selecting /4 */
  uint32_t rst_reg;           /* Reset register */
  uint32_t rst_bit;           /* Reset bit */
  uint32_t scl_pin;           /* GPIO configuration for SCL as SCL */
  uint32_t sda_pin;           /* GPIO configuration for SDA as SDA */
  uint32_t scl_gpio;          /* GPIO configuration for SCL as open-drain output */
  uint32_t sda_gpio;          /* GPIO configuration for SDA as open-drain output */
#ifndef CONFIG_I2C_POLLED
  uint32_t ev_irq;            /* Event IRQ */
  uint32_t er_irq;            /* Error IRQ */
#endif
};

/* I2C Device Private Data */

struct n32_i2c_priv_s
{
  /* Port configuration */

  FAR const struct n32_i2c_config_s *config;

  int refs;                    /* Reference count */
  mutex_t lock;                /* Mutual exclusion mutex */
#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;               /* Interrupt wait semaphore */
#endif
  volatile uint8_t intstate;   /* Interrupt handshake (see enum n32_intstate_e) */

  uint8_t msgc;                /* Message count */
  FAR struct i2c_msg_s *msgv;  /* Message list */
  FAR uint8_t *ptr;            /* Current message buffer */
  uint32_t frequency;          /* Current I2C frequency */
  int dcnt;                    /* Current message bytes remaining to transfer */
  uint16_t flags;              /* Current message flags */
  bool astart;                 /* START sent */

  uint32_t status;             /* End of transfer status */
};

/* I2C Device, Instance */

struct n32_i2c_inst_s
{
  FAR const struct i2c_ops_s *ops;    /* Standard I2C operations */
  FAR struct n32_i2c_priv_s *priv;    /* Common driver private data structure */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t n32_i2c_getreg32(FAR struct n32_i2c_priv_s *priv,
                                        uint8_t offset);
static inline void n32_i2c_putreg32(FAR struct n32_i2c_priv_s *priv,
                                    uint8_t offset, uint32_t value);
static inline void n32_i2c_modifyreg32(FAR struct n32_i2c_priv_s *priv,
                                       uint8_t offset, uint32_t clearbits,
                                       uint32_t setbits);
static uint32_t n32_i2c_timing(uint32_t frequency);
#ifdef CONFIG_N32H7_I2C_DYNTIMEO
static uint32_t n32_i2c_toticks(int msgc, FAR struct i2c_msg_s *msgs);
#endif
#ifndef CONFIG_I2C_POLLED
static void n32_i2c_enableinterrupts(FAR struct n32_i2c_priv_s *priv,
                                     uint32_t mask);
#endif
static int n32_i2c_sem_waitdone(FAR struct n32_i2c_priv_s *priv);
static void n32_i2c_sem_waitstop(FAR struct n32_i2c_priv_s *priv);
static bool n32_i2c_waitnotbusy(FAR struct n32_i2c_priv_s *priv);
static void n32_i2c_setclock(FAR struct n32_i2c_priv_s *priv,
                             uint32_t frequency);
static inline void n32_i2c_set_7bit_address(FAR struct n32_i2c_priv_s *priv);
static inline void
n32_i2c_set_bytes_to_transfer(FAR struct n32_i2c_priv_s *priv,
                              uint8_t n_bytes);
static inline void
n32_i2c_set_write_transfer_dir(FAR struct n32_i2c_priv_s *priv);
static inline void
n32_i2c_set_read_transfer_dir(FAR struct n32_i2c_priv_s *priv);
static inline void n32_i2c_enable_reload(FAR struct n32_i2c_priv_s *priv);
static inline void n32_i2c_disable_reload(FAR struct n32_i2c_priv_s *priv);
static inline void n32_i2c_sendstart(FAR struct n32_i2c_priv_s *priv);
static inline void n32_i2c_sendstop(FAR struct n32_i2c_priv_s *priv);
static inline uint32_t n32_i2c_getstatus(FAR struct n32_i2c_priv_s *priv);
static inline void n32_i2c_clearinterrupts(FAR struct n32_i2c_priv_s *priv);
static int n32_i2c_isr_process(FAR struct n32_i2c_priv_s *priv);
#ifndef CONFIG_I2C_POLLED
static int n32_i2c_isr(int irq, FAR void *context, FAR void *arg);
#endif
static void n32_i2c_kerclk(FAR const struct n32_i2c_config_s *config);
static int n32_i2c_init(FAR struct n32_i2c_priv_s *priv);
static int n32_i2c_deinit(FAR struct n32_i2c_priv_s *priv);
static int n32_i2c_recover(FAR struct n32_i2c_priv_s *priv);
static int n32_i2c_process(FAR struct i2c_master_s *dev,
                           FAR struct i2c_msg_s *msgs, int count);
static int n32_i2c_transfer(FAR struct i2c_master_s *dev,
                            FAR struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int n32_i2c_reset(FAR struct i2c_master_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_N32H7_I2C1
static const struct n32_i2c_config_s n32_i2c1_config =
{
  .base          = N32_I2C1_BASE,
  .group         = I2C_GROUP_APB1,
  .sel_reg       = N32_RCC_APB1SEL1,
  .sel_mask      = RCC_APB1SEL1_I2C1KERSEL_MASK,
  .sel_sysdiv    = RCC_APB1SEL1_I2C1KERSEL_SYSDIV,
  .div_reg       = N32_RCC_APB1DIV1,
  .div_mask      = RCC_APB1DIV1_APB1I2CDIV_MASK,
  .div_div4      = RCC_APB1DIV1_APB1I2CDIV_DIV4,
  .rst_reg       = N32_RCC_APB1RST4,
  .rst_bit       = RCC_APB1RST4_I2C1RST,
  .scl_pin       = GPIO_I2C1_SCL,
  .sda_pin       = GPIO_I2C1_SDA,
  .scl_gpio      = GPIO_I2C1_SCL_GPIO,
  .sda_gpio      = GPIO_I2C1_SDA_GPIO,
#  ifndef CONFIG_I2C_POLLED
  .ev_irq        = N32_IRQ_I2C1_EVENT,
  .er_irq        = N32_IRQ_I2C1_ERR
#  endif
};

static struct n32_i2c_priv_s n32_i2c1_priv =
{
  .config        = &n32_i2c1_config,
  .refs          = 0,
  .lock          = NXMUTEX_INITIALIZER,
#  ifndef CONFIG_I2C_POLLED
  .sem_isr       = SEM_INITIALIZER(0),
#  endif
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .frequency     = 0,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0,
};
#endif

#ifdef CONFIG_N32H7_I2C2
static const struct n32_i2c_config_s n32_i2c2_config =
{
  .base          = N32_I2C2_BASE,
  .group         = I2C_GROUP_APB1,
  .sel_reg       = N32_RCC_APB1SEL1,
  .sel_mask      = RCC_APB1SEL1_I2C2KERSEL_MASK,
  .sel_sysdiv    = RCC_APB1SEL1_I2C2KERSEL_SYSDIV,
  .div_reg       = N32_RCC_APB1DIV1,
  .div_mask      = RCC_APB1DIV1_APB1I2CDIV_MASK,
  .div_div4      = RCC_APB1DIV1_APB1I2CDIV_DIV4,
  .rst_reg       = N32_RCC_APB1RST4,
  .rst_bit       = RCC_APB1RST4_I2C2RST,
  .scl_pin       = GPIO_I2C2_SCL,
  .sda_pin       = GPIO_I2C2_SDA,
  .scl_gpio      = GPIO_I2C2_SCL_GPIO,
  .sda_gpio      = GPIO_I2C2_SDA_GPIO,
#  ifndef CONFIG_I2C_POLLED
  .ev_irq        = N32_IRQ_I2C2_EVENT,
  .er_irq        = N32_IRQ_I2C2_ERR
#  endif
};

static struct n32_i2c_priv_s n32_i2c2_priv =
{
  .config        = &n32_i2c2_config,
  .refs          = 0,
  .lock          = NXMUTEX_INITIALIZER,
#  ifndef CONFIG_I2C_POLLED
  .sem_isr       = SEM_INITIALIZER(0),
#  endif
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .frequency     = 0,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0,
};
#endif

#ifdef CONFIG_N32H7_I2C3
static const struct n32_i2c_config_s n32_i2c3_config =
{
  .base          = N32_I2C3_BASE,
  .group         = I2C_GROUP_APB1,
  .sel_reg       = N32_RCC_APB1SEL1,
  .sel_mask      = RCC_APB1SEL1_I2C3KERSEL_MASK,
  .sel_sysdiv    = RCC_APB1SEL1_I2C3KERSEL_SYSDIV,
  .div_reg       = N32_RCC_APB1DIV1,
  .div_mask      = RCC_APB1DIV1_APB1I2CDIV_MASK,
  .div_div4      = RCC_APB1DIV1_APB1I2CDIV_DIV4,
  .rst_reg       = N32_RCC_APB1RST4,
  .rst_bit       = RCC_APB1RST4_I2C3RST,
  .scl_pin       = GPIO_I2C3_SCL,
  .sda_pin       = GPIO_I2C3_SDA,
  .scl_gpio      = GPIO_I2C3_SCL_GPIO,
  .sda_gpio      = GPIO_I2C3_SDA_GPIO,
#  ifndef CONFIG_I2C_POLLED
  .ev_irq        = N32_IRQ_I2C3_EVENT,
  .er_irq        = N32_IRQ_I2C3_ERR
#  endif
};

static struct n32_i2c_priv_s n32_i2c3_priv =
{
  .config        = &n32_i2c3_config,
  .refs          = 0,
  .lock          = NXMUTEX_INITIALIZER,
#  ifndef CONFIG_I2C_POLLED
  .sem_isr       = SEM_INITIALIZER(0),
#  endif
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .frequency     = 0,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0,
};
#endif

#ifdef CONFIG_N32H7_I2C4
static const struct n32_i2c_config_s n32_i2c4_config =
{
  .base          = N32_I2C4_BASE,
  .group         = I2C_GROUP_APB2,
  .sel_reg       = N32_RCC_APB2SEL1,
  .sel_mask      = RCC_APB2SEL1_I2C4KERSEL_MASK,
  .sel_sysdiv    = RCC_APB2SEL1_I2C4KERSEL_SYSCLK,
  .div_reg       = N32_RCC_APB2DIV1,
  .div_mask      = RCC_APB2DIV1_APB2I2CDIV_MASK,
  .div_div4      = RCC_APB2DIV1_APB2I2CDIV_DIV4,
  .rst_reg       = N32_RCC_APB2RST2,
  .rst_bit       = RCC_APB2RST2_I2C4RST,
  .scl_pin       = GPIO_I2C4_SCL,
  .sda_pin       = GPIO_I2C4_SDA,
  .scl_gpio      = GPIO_I2C4_SCL_GPIO,
  .sda_gpio      = GPIO_I2C4_SDA_GPIO,
#  ifndef CONFIG_I2C_POLLED
  .ev_irq        = N32_IRQ_I2C4_EVENT,
  .er_irq        = N32_IRQ_I2C4_ERR
#  endif
};

static struct n32_i2c_priv_s n32_i2c4_priv =
{
  .config        = &n32_i2c4_config,
  .refs          = 0,
  .lock          = NXMUTEX_INITIALIZER,
#  ifndef CONFIG_I2C_POLLED
  .sem_isr       = SEM_INITIALIZER(0),
#  endif
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .frequency     = 0,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0,
};
#endif

#ifdef CONFIG_N32H7_I2C5
static const struct n32_i2c_config_s n32_i2c5_config =
{
  .base          = N32_I2C5_BASE,
  .group         = I2C_GROUP_APB2,
  .sel_reg       = N32_RCC_APB2SEL1,
  .sel_mask      = RCC_APB2SEL1_I2C5KERSEL_MASK,
  .sel_sysdiv    = RCC_APB2SEL1_I2C5KERSEL_SYSCLK,
  .div_reg       = N32_RCC_APB2DIV1,
  .div_mask      = RCC_APB2DIV1_APB2I2CDIV_MASK,
  .div_div4      = RCC_APB2DIV1_APB2I2CDIV_DIV4,
  .rst_reg       = N32_RCC_APB2RST2,
  .rst_bit       = RCC_APB2RST2_I2C5RST,
  .scl_pin       = GPIO_I2C5_SCL,
  .sda_pin       = GPIO_I2C5_SDA,
  .scl_gpio      = GPIO_I2C5_SCL_GPIO,
  .sda_gpio      = GPIO_I2C5_SDA_GPIO,
#  ifndef CONFIG_I2C_POLLED
  .ev_irq        = N32_IRQ_I2C5_EVENT,
  .er_irq        = N32_IRQ_I2C5_ERR
#  endif
};

static struct n32_i2c_priv_s n32_i2c5_priv =
{
  .config        = &n32_i2c5_config,
  .refs          = 0,
  .lock          = NXMUTEX_INITIALIZER,
#  ifndef CONFIG_I2C_POLLED
  .sem_isr       = SEM_INITIALIZER(0),
#  endif
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .frequency     = 0,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0,
};
#endif

#ifdef CONFIG_N32H7_I2C6
static const struct n32_i2c_config_s n32_i2c6_config =
{
  .base          = N32_I2C6_BASE,
  .group         = I2C_GROUP_APB2,
  .sel_reg       = N32_RCC_APB2SEL1,
  .sel_mask      = RCC_APB2SEL1_I2C6KERSEL_MASK,
  .sel_sysdiv    = RCC_APB2SEL1_I2C6KERSEL_SYSCLK,
  .div_reg       = N32_RCC_APB2DIV1,
  .div_mask      = RCC_APB2DIV1_APB2I2CDIV_MASK,
  .div_div4      = RCC_APB2DIV1_APB2I2CDIV_DIV4,
  .rst_reg       = N32_RCC_APB2RST2,
  .rst_bit       = RCC_APB2RST2_I2C6RST,
  .scl_pin       = GPIO_I2C6_SCL,
  .sda_pin       = GPIO_I2C6_SDA,
  .scl_gpio      = GPIO_I2C6_SCL_GPIO,
  .sda_gpio      = GPIO_I2C6_SDA_GPIO,
#  ifndef CONFIG_I2C_POLLED
  .ev_irq        = N32_IRQ_I2C6_EVENT,
  .er_irq        = N32_IRQ_I2C6_ERR
#  endif
};

static struct n32_i2c_priv_s n32_i2c6_priv =
{
  .config        = &n32_i2c6_config,
  .refs          = 0,
  .lock          = NXMUTEX_INITIALIZER,
#  ifndef CONFIG_I2C_POLLED
  .sem_isr       = SEM_INITIALIZER(0),
#  endif
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .frequency     = 0,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0,
};
#endif

#ifdef CONFIG_N32H7_I2C7
static const struct n32_i2c_config_s n32_i2c7_config =
{
  .base          = N32_I2C7_BASE,
  .group         = I2C_GROUP_APB5,
  .sel_reg       = N32_RCC_APB5SEL1,
  .sel_mask      = RCC_APB5SEL1_I2C7KERSEL_MASK,
  .sel_sysdiv    = RCC_APB5SEL1_I2C7KERSEL_SYSCLK,
  .div_reg       = N32_RCC_APB5DIV1,
  .div_mask      = RCC_APB5DIV1_APB5I2CDIV_MASK,
  .div_div4      = RCC_APB5DIV1_APB5I2CDIV_DIV4,
  .rst_reg       = N32_RCC_APB5RST2,
  .rst_bit       = RCC_APB5RST2_I2C7RST,
  .scl_pin       = GPIO_I2C7_SCL,
  .sda_pin       = GPIO_I2C7_SDA,
  .scl_gpio      = GPIO_I2C7_SCL_GPIO,
  .sda_gpio      = GPIO_I2C7_SDA_GPIO,
#  ifndef CONFIG_I2C_POLLED
  .ev_irq        = N32_IRQ_I2C7_EVENT,
  .er_irq        = N32_IRQ_I2C7_ERR
#  endif
};

static struct n32_i2c_priv_s n32_i2c7_priv =
{
  .config        = &n32_i2c7_config,
  .refs          = 0,
  .lock          = NXMUTEX_INITIALIZER,
#  ifndef CONFIG_I2C_POLLED
  .sem_isr       = SEM_INITIALIZER(0),
#  endif
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .frequency     = 0,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0,
};
#endif

#ifdef CONFIG_N32H7_I2C8
static const struct n32_i2c_config_s n32_i2c8_config =
{
  .base          = N32_I2C8_BASE,
  .group         = I2C_GROUP_APB5,
  .sel_reg       = N32_RCC_APB5SEL1,
  .sel_mask      = RCC_APB5SEL1_I2C8KERSEL_MASK,
  .sel_sysdiv    = RCC_APB5SEL1_I2C8KERSEL_SYSCLK,
  .div_reg       = N32_RCC_APB5DIV1,
  .div_mask      = RCC_APB5DIV1_APB5I2CDIV_MASK,
  .div_div4      = RCC_APB5DIV1_APB5I2CDIV_DIV4,
  .rst_reg       = N32_RCC_APB5RST2,
  .rst_bit       = RCC_APB5RST2_I2C8RST,
  .scl_pin       = GPIO_I2C8_SCL,
  .sda_pin       = GPIO_I2C8_SDA,
  .scl_gpio      = GPIO_I2C8_SCL_GPIO,
  .sda_gpio      = GPIO_I2C8_SDA_GPIO,
#  ifndef CONFIG_I2C_POLLED
  .ev_irq        = N32_IRQ_I2C8_EVENT,
  .er_irq        = N32_IRQ_I2C8_ERR
#  endif
};

static struct n32_i2c_priv_s n32_i2c8_priv =
{
  .config        = &n32_i2c8_config,
  .refs          = 0,
  .lock          = NXMUTEX_INITIALIZER,
#  ifndef CONFIG_I2C_POLLED
  .sem_isr       = SEM_INITIALIZER(0),
#  endif
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .frequency     = 0,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0,
};
#endif

/* Kernel clock configured, one flag per RCC group */

static bool g_i2c_clkcfg[I2C_GROUP_NGROUPS];

/* Device Structures, Instantiation */

static const struct i2c_ops_s n32_i2c_ops =
{
  .transfer      = n32_i2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset         = n32_i2c_reset,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_i2c_getreg32
 *
 * Description:
 *   Get a 32-bit register value by offset.
 *
 ****************************************************************************/

static inline uint32_t n32_i2c_getreg32(FAR struct n32_i2c_priv_s *priv,
                                        uint8_t offset)
{
  return getreg32(priv->config->base + offset);
}

/****************************************************************************
 * Name: n32_i2c_putreg32
 *
 * Description:
 *   Put a 32-bit register value by offset.
 *
 ****************************************************************************/

static inline void n32_i2c_putreg32(FAR struct n32_i2c_priv_s *priv,
                                    uint8_t offset, uint32_t value)
{
  putreg32(value, priv->config->base + offset);
}

/****************************************************************************
 * Name: n32_i2c_modifyreg32
 *
 * Description:
 *   Modify a 32-bit register value by offset.
 *
 ****************************************************************************/

static inline void n32_i2c_modifyreg32(FAR struct n32_i2c_priv_s *priv,
                                       uint8_t offset, uint32_t clearbits,
                                       uint32_t setbits)
{
  modifyreg32(priv->config->base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: n32_i2c_timing
 *
 * Description:
 *   Compute the BUSTM value for an SCL frequency.  The frequency must be
 *   in the range I2C_FREQ_MIN to 1000000 Hz; the caller maps other values
 *   into that range.
 *
 *   One time unit is 1 / I2C_FCKDIV.  tLSCL = (LSCL + 1) units,
 *   tHSCL = (HSCL + 1) units, tDSCL = (DSCL + 1) units and tDSDA = DSDA
 *   units.  The SCL period never drops below the requested one and the
 *   mode minima are met; on the hardware SCL runs a few percent slower
 *   because of the line rise times.
 *
 * Input Parameters:
 *   frequency - SCL frequency in Hz.
 *
 * Returned Value:
 *   The value to write to BUSTM.
 *
 ****************************************************************************/

static uint32_t n32_i2c_timing(uint32_t frequency)
{
  int64_t period;
  int64_t low;
  int64_t high;
  int64_t dscl;
  int64_t tlow;
  int64_t thigh;
  int64_t tsudat;
  uint32_t dsda;

  /* Mode minima in ns: tLOW, tHIGH and tSU;DAT */

  if (frequency <= 100000)
    {
      tlow   = 4700;
      thigh  = 4000;
      tsudat = 250;
    }
  else if (frequency <= 400000)
    {
      tlow   = 1300;
      thigh  = 600;
      tsudat = 100;
    }
  else
    {
      tlow   = 500;
      thigh  = 260;
      tsudat = 50;
    }

  /* Whole SCL period in units: ceil(I2C_FCKDIV / frequency) */

  period = ((int64_t)I2C_FCKDIV + (int64_t)frequency - 1) /
           (int64_t)frequency;

  /* Mode minima in the same unit: ceil(t * I2C_FCKDIV / 1e9) */

  tlow   = (tlow * (int64_t)I2C_FCKDIV + 999999999) / 1000000000;
  thigh  = (thigh * (int64_t)I2C_FCKDIV + 999999999) / 1000000000;
  tsudat = (tsudat * (int64_t)I2C_FCKDIV + 999999999) / 1000000000;

  /* SCL low time: half of the period, rounded up, but never below the
   * mode minimum and never below 2 units (the hardware minimum for
   * LSCL + 1).
   */

  low = (period + 1) / 2;
  if (low < tlow)
    {
      low = tlow;
    }

  if (low < 2)
    {
      low = 2;
    }

  /* SCL high time: the rest of the period, but never below the mode
   * minimum.  A minimum that pushes the low time above half of the
   * period only makes SCL slower than requested.
   */

  high = period - low;
  if (high < thigh)
    {
      high = thigh;
    }

  if (high < 1)
    {
      high = 1;
    }

  /* Data setup time: ceil(tSU;DAT in units) - 1, clamped to the field */

  dscl = tsudat - 1;
  if (dscl < 0)
    {
      dscl = 0;
    }

  if (dscl > 15)
    {
      dscl = 15;
    }

  /* Data hold time: 1 unit up to 400 kHz, none above */

  if (frequency <= 400000)
    {
      dsda = 1;
    }
  else
    {
      dsda = 0;
    }

  return ((uint32_t)I2C_CKDIV << I2C_BUSTM_CKDIV_SHIFT) |
         ((uint32_t)dscl << I2C_BUSTM_DSCL_SHIFT) |
         (dsda << I2C_BUSTM_DSDA_SHIFT) |
         ((uint32_t)(high - 1) << I2C_BUSTM_HSCL_SHIFT) |
         (uint32_t)(low - 1);
}

/****************************************************************************
 * Name: n32_i2c_toticks
 *
 * Description:
 *   Return a number of system timer ticks that grows with the number of
 *   bytes of the messages that have not been processed yet.
 *
 ****************************************************************************/

#ifdef CONFIG_N32H7_I2C_DYNTIMEO
static uint32_t n32_i2c_toticks(int msgc, FAR struct i2c_msg_s *msgs)
{
  size_t bytecount = 0;
  int i;

  /* Count the number of bytes left to process */

  for (i = 0; i < msgc; i++)
    {
      bytecount += msgs[i].length;
    }

  /* Then return a number of ticks based on a user provided scaling
   * factor.
   */

  return USEC2TICK(CONFIG_N32H7_I2C_DYNTIMEO_USECPERBYTE * bytecount);
}
#endif

/****************************************************************************
 * Name: n32_i2c_enableinterrupts
 *
 * Description:
 *   Enable I2C interrupts.  I2CEN and the other CTRL1 bits are preserved.
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static void n32_i2c_enableinterrupts(FAR struct n32_i2c_priv_s *priv,
                                     uint32_t mask)
{
  n32_i2c_modifyreg32(priv, N32_I2C_CTRL1_OFFSET, 0, mask);

  /* Drain the domain write buffer: the interrupt enable bits must LAND in
   * the controller before the transfer is triggered, otherwise the
   * transfer can complete with its interrupt still masked (same ordering
   * rule as n32_sdmmc.c).
   */

  UP_DSB();
}
#endif

/****************************************************************************
 * Name: n32_i2c_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete.
 *
 *   There are two versions of this function.  The first is included when
 *   using interrupts while the second is used if polling
 *   (CONFIG_I2C_POLLED=y).
 *
 * Returned Value:
 *   OK when the transfer completed, a negated errno value when the wait
 *   timed out.
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int n32_i2c_sem_waitdone(FAR struct n32_i2c_priv_s *priv)
{
  irqstate_t flags;
  int ret;

  flags = enter_critical_section();

  /* Enable I2C interrupts */

  /* The WDRIE and RDRIE interrupts are enabled initially in
   * n32_i2c_process.  The remainder of the interrupts, including
   * error-related, are enabled here.
   */

  n32_i2c_enableinterrupts(priv, I2C_CTRL1_TFCIE | I2C_CTRL1_ERRIE);

  /* Signal the interrupt handler that we are waiting */

  priv->intstate = INTSTATE_WAITING;
  do
    {
      /* Wait until either the transfer is complete or the timeout
       * expires.
       */

#ifdef CONFIG_N32H7_I2C_DYNTIMEO
      ret = nxsem_tickwait_uninterruptible(&priv->sem_isr,
                         n32_i2c_toticks(priv->msgc, priv->msgv));
#else
      ret = nxsem_tickwait_uninterruptible(&priv->sem_isr,
                                           CONFIG_N32H7_I2CTIMEOTICKS);
#endif
      if (ret < 0)
        {
          /* Break out of the loop on irrecoverable errors.  This would
           * include timeouts and mystery errors reported by
           * nxsem_tickwait_uninterruptible.
           */

          break;
        }
    }

  /* Loop until the interrupt level transfer is complete. */

  while (priv->intstate != INTSTATE_DONE);

  /* Set the interrupt state back to IDLE */

  priv->intstate = INTSTATE_IDLE;

  /* Disable I2C interrupts */

  n32_i2c_modifyreg32(priv, N32_I2C_CTRL1_OFFSET, I2C_CTRL1_ALLINTS, 0);
  leave_critical_section(flags);
  return ret;
}
#else
static int n32_i2c_sem_waitdone(FAR struct n32_i2c_priv_s *priv)
{
  clock_t timeout;
  clock_t start;
  clock_t elapsed;
  int ret;

  /* Get the timeout value */

#ifdef CONFIG_N32H7_I2C_DYNTIMEO
  timeout = n32_i2c_toticks(priv->msgc, priv->msgv);
#else
  timeout = CONFIG_N32H7_I2CTIMEOTICKS;
#endif

  /* Signal the interrupt handler that we are waiting */

  priv->intstate = INTSTATE_WAITING;
  start = clock_systime_ticks();

  do
    {
      /* Calculate the elapsed time */

      elapsed = clock_systime_ticks() - start;

      /* Poll by simply calling the interrupt handler until it reports
       * that it is done.
       */

      n32_i2c_isr_process(priv);
    }

  /* Loop until the transfer is complete. */

  while (priv->intstate != INTSTATE_DONE && elapsed < timeout);

  ret = priv->intstate == INTSTATE_DONE ? OK : -ETIMEDOUT;

  /* Set the interrupt state back to IDLE */

  priv->intstate = INTSTATE_IDLE;
  return ret;
}
#endif

/****************************************************************************
 * Name: n32_i2c_sem_waitstop
 *
 * Description:
 *   Wait for a STOP to complete.
 *
 ****************************************************************************/

static void n32_i2c_sem_waitstop(FAR struct n32_i2c_priv_s *priv)
{
  clock_t start;
  clock_t elapsed;
  clock_t timeout;
  uint32_t cr;
  uint32_t sr;

  /* Select a timeout */

#ifdef CONFIG_N32H7_I2C_DYNTIMEO
  timeout = USEC2TICK(CONFIG_N32H7_I2C_DYNTIMEO_STARTSTOP);
#else
  timeout = CONFIG_N32H7_I2CTIMEOTICKS;
#endif

  /* Wait as stop might still be in progress */

  start = clock_systime_ticks();
  do
    {
      /* Calculate the elapsed time */

      elapsed = clock_systime_ticks() - start;

      /* Check for STOP condition */

      cr = n32_i2c_getreg32(priv, N32_I2C_CTRL2_OFFSET);
      if ((cr & I2C_CTRL2_STOP) == 0)
        {
          return;
        }

      /* Check for timeout error */

      sr = n32_i2c_getreg32(priv, N32_I2C_STSINT_OFFSET);
      if ((sr & I2C_STSINT_TMOUT) != 0)
        {
          return;
        }
    }

  /* Loop until the stop is complete or a timeout occurs. */

  while (elapsed < timeout);

  /* If we get here then a timeout occurred with the STOP condition
   * still pending.  The BUSY check before the next START recovers the
   * bus.
   */

  i2cwarn("WARNING: Timeout with CR: 0x%08" PRIx32 " SR: 0x%08" PRIx32 "\n",
          cr, sr);
}

/****************************************************************************
 * Name: n32_i2c_waitnotbusy
 *
 * Description:
 *   Wait with a bound for the BUSY flag to clear.
 *
 * Returned Value:
 *   true when the bus is free, false when BUSY is still set.
 *
 ****************************************************************************/

static bool n32_i2c_waitnotbusy(FAR struct n32_i2c_priv_s *priv)
{
  unsigned int count;

  /* Give the bus a few hundred microseconds to release */

  for (count = 0; count < I2C_BUSY_LOOPS; count++)
    {
      if ((n32_i2c_getreg32(priv, N32_I2C_STSINT_OFFSET) &
           I2C_STSINT_BUSY) == 0)
        {
          return true;
        }

      up_udelay(I2C_BUSY_DELAY);
    }

  return false;
}

/****************************************************************************
 * Name: n32_i2c_setclock
 *
 * Description:
 *   Set the I2C bus clock frequency by configuring the BUSTM register.
 *
 *   BUSTM can only be programmed while the peripheral is disabled and
 *   clearing I2CEN is a software reset, so the timing is only programmed
 *   when the frequency changes.  The peripheral is enabled on return,
 *   also after a software reset by the ISR.
 *
 *   Requests are met from I2C_FREQ_MIN to 1 MHz; high-speed mode is not
 *   supported and 0 selects 100 kHz.
 *
 ****************************************************************************/

static void n32_i2c_setclock(FAR struct n32_i2c_priv_s *priv,
                             uint32_t frequency)
{
  uint32_t timing;

  if (frequency != priv->frequency)
    {
      if (frequency == 0)
        {
          frequency = 100000;
        }
      else if (frequency > 1000000)
        {
          i2cwarn("WARNING: I2C frequency %" PRIu32 " not supported,"
                  " using 1000000\n", frequency);
          frequency = 1000000;
        }
      else if (frequency < I2C_FREQ_MIN)
        {
          i2cwarn("WARNING: I2C frequency %" PRIu32 " not supported,"
                  " using %" PRIu32 "\n", frequency,
                  (uint32_t)I2C_FREQ_MIN);
          frequency = I2C_FREQ_MIN;
        }

      timing = n32_i2c_timing(frequency);

      /* I2C peripheral must be disabled to update the timing.  This will
       * SW reset the device: I2CEN must stay low for at least three PCLK
       * or three prescaled kernel clock periods.
       */

      n32_i2c_modifyreg32(priv, N32_I2C_CTRL1_OFFSET, I2C_CTRL1_I2CEN, 0);
      up_udelay(1);

      n32_i2c_putreg32(priv, N32_I2C_BUSTM_OFFSET, timing);
      priv->frequency = frequency;
    }

  /* Enable I2C peripheral */

  n32_i2c_modifyreg32(priv, N32_I2C_CTRL1_OFFSET, 0, I2C_CTRL1_I2CEN);
}

/****************************************************************************
 * Name: n32_i2c_set_7bit_address
 *
 * Description:
 *   Set the 7-bit slave address of the current message.
 *
 ****************************************************************************/

static inline void n32_i2c_set_7bit_address(FAR struct n32_i2c_priv_s *priv)
{
  n32_i2c_modifyreg32(priv, N32_I2C_CTRL2_OFFSET, I2C_CTRL2_SADR7_MASK,
                      ((priv->msgv->addr & 0x7f) << I2C_CTRL2_SADR7_SHIFT));
}

/****************************************************************************
 * Name: n32_i2c_set_bytes_to_transfer
 *
 * Description:
 *   Set the number of bytes to transfer (BYTECNT).
 *
 ****************************************************************************/

static inline void
n32_i2c_set_bytes_to_transfer(FAR struct n32_i2c_priv_s *priv,
                              uint8_t n_bytes)
{
  n32_i2c_modifyreg32(priv, N32_I2C_CTRL2_OFFSET, I2C_CTRL2_BYTECNT_MASK,
                      ((uint32_t)n_bytes << I2C_CTRL2_BYTECNT_SHIFT));
}

/****************************************************************************
 * Name: n32_i2c_set_write_transfer_dir
 *
 * Description:
 *   Select the write direction.
 *
 ****************************************************************************/

static inline void
n32_i2c_set_write_transfer_dir(FAR struct n32_i2c_priv_s *priv)
{
  n32_i2c_modifyreg32(priv, N32_I2C_CTRL2_OFFSET, I2C_CTRL2_RWN, 0);
}

/****************************************************************************
 * Name: n32_i2c_set_read_transfer_dir
 *
 * Description:
 *   Select the read direction.
 *
 ****************************************************************************/

static inline void
n32_i2c_set_read_transfer_dir(FAR struct n32_i2c_priv_s *priv)
{
  n32_i2c_modifyreg32(priv, N32_I2C_CTRL2_OFFSET, 0, I2C_CTRL2_RWN);
}

/****************************************************************************
 * Name: n32_i2c_enable_reload
 *
 * Description:
 *   Enable the BYTECNT reload mode (REFILL).
 *
 ****************************************************************************/

static inline void n32_i2c_enable_reload(FAR struct n32_i2c_priv_s *priv)
{
  n32_i2c_modifyreg32(priv, N32_I2C_CTRL2_OFFSET, 0, I2C_CTRL2_REFILL);
}

/****************************************************************************
 * Name: n32_i2c_disable_reload
 *
 * Description:
 *   Disable the BYTECNT reload mode (REFILL).
 *
 ****************************************************************************/

static inline void n32_i2c_disable_reload(FAR struct n32_i2c_priv_s *priv)
{
  n32_i2c_modifyreg32(priv, N32_I2C_CTRL2_OFFSET, I2C_CTRL2_REFILL, 0);
}

/****************************************************************************
 * Name: n32_i2c_sendstart
 *
 * Description:
 *   Send the START condition / force Master mode.
 *
 *   A START condition in I2C consists of a single byte that contains both
 *   the 7 bit slave address and a read/write bit (0 = WRITE, 1 = READ).
 *   If the address is recognized by one of the slave devices that slave
 *   device will ACK the byte so that data transfers can begin.
 *
 *   A RESTART (or repeated START per the I2CSPEC) is simply a START
 *   condition issued in the middle of a transfer (i.e. after the initial
 *   START and before a STOP).  A RESTART sends a new address byte and R/W
 *   bit to the bus. A RESTART is optional in most cases but mandatory in
 *   the event the transfer direction is changed.
 *
 *   Most of the time reading data from an I2C slave requires a WRITE of
 *   the subaddress followed by a READ (and hence a RESTART in between).
 *   Writing to an I2C slave typically requires only WRITE operations and
 *   hence no RESTARTs.
 *
 *   This function is therefore called both at the beginning of a transfer
 *   (START) and at appropriate times during a transfer (RESTART).
 *
 ****************************************************************************/

static inline void n32_i2c_sendstart(FAR struct n32_i2c_priv_s *priv)
{
  bool next_norestart = false;

  /* Set the private "current message" data used in protocol processing.
   *
   * ptr:   A pointer to the start of the current message buffer.  This is
   *        advanced after each byte in the current message is transferred.
   *
   * dcnt:  A running counter of the bytes in the current message waiting
   *        to be transferred.  This is decremented each time a byte is
   *        transferred. The hardware normally accepts a maximum of 255
   *        bytes per transfer but can support more via the RELOAD
   *        mechanism.  If dcnt initially exceeds 255, the RELOAD mechanism
   *        will be enabled automatically.
   *
   * flags: Used to characterize handling of the current message.
   *
   *  The default flags value is 0 which specifies:
   *
   *   - A transfer direction of WRITE (R/W bit = 0)
   *   - RESTARTs between all messages
   *
   *  The following flags can be used to override this behavior as follows:
   *
   *   - I2C_M_READ: Sets the transfer direction to READ (R/W bit = 1)
   *   - I2C_M_NOSTART: Prevents a RESTART from being issued prior to the
   *      transfer of the message (where allowed by the protocol).
   *
   */

  priv->ptr   = priv->msgv->buffer;
  priv->dcnt  = priv->msgv->length;
  priv->flags = priv->msgv->flags;

  if ((priv->flags & I2C_M_NOSTART) == 0)
    {
      /* Flag the first byte as an address byte */

      priv->astart = true;
    }

  /* Enabling RELOAD allows the transfer of:
   *
   *  - individual messages with a payload exceeding 255 bytes
   *  - multiple messages back to back without a RESTART in between
   *
   * so we enable it if either of those conditions exist and disable
   * it otherwise.
   */

  /* Check if there are multiple messages and the next is a continuation */

  if (priv->msgc > 1)
    {
      next_norestart = (((priv->msgv + 1)->flags & I2C_M_NOSTART) != 0);
    }

  if (next_norestart || priv->dcnt > 255)
    {
      i2cinfo("REFILL enabled: dcnt = %i msgc = %i\n",
              priv->dcnt, priv->msgc);
      n32_i2c_enable_reload(priv);
    }
  else
    {
      i2cinfo("REFILL disabled: dcnt = %i msgc = %i\n",
              priv->dcnt, priv->msgc);
      n32_i2c_disable_reload(priv);
    }

  /* Set the number of bytes to transfer (CTRL2->BYTECNT) to the number of
   * bytes in the current message or 255, whichever is lower so as to not
   * exceed the hardware maximum allowed.
   */

  if (priv->dcnt > 255)
    {
      n32_i2c_set_bytes_to_transfer(priv, 255);
    }
  else
    {
      n32_i2c_set_bytes_to_transfer(priv, priv->dcnt);
    }

  /* Set the (7 bit) address.  10 bit addressing is not supported, such
   * requests are rejected by n32_i2c_process().
   */

  n32_i2c_set_7bit_address(priv);

  /* The flag of the current message is used to determine the direction of
   * transfer required for the current message.
   */

  if (priv->flags & I2C_M_READ)
    {
      n32_i2c_set_read_transfer_dir(priv);
    }
  else
    {
      n32_i2c_set_write_transfer_dir(priv);
    }

  /* Set the CTRL2->START bit to 1 to instruct the hardware to send the
   * START condition using the address and transfer direction data entered.
   */

  i2cinfo("Sending START: dcnt=%i msgc=%i flags=0x%04x\n",
          priv->dcnt, priv->msgc, priv->flags);

  n32_i2c_modifyreg32(priv, N32_I2C_CTRL2_OFFSET, 0, I2C_CTRL2_START);

  /* A software reset clears WRE, after which WDR cannot be written.  As
   * in the vendor master transmit sequences, write 1 to WRE after every
   * START into a write message to flush the transmit data register.  It
   * is a plain assignment: writing the read-back value would also write
   * WRAVL.
   */

  if ((priv->flags & I2C_M_READ) == 0)
    {
      n32_i2c_putreg32(priv, N32_I2C_STSINT_OFFSET, I2C_STSINT_WRE);
    }
}

/****************************************************************************
 * Name: n32_i2c_sendstop
 *
 * Description:
 *   Send the STOP condition.
 *
 *   A STOP condition can be requested by setting the STOP bit in the
 *   CTRL2 register.  Setting the STOP bit clears the TFC flag and the STOP
 *   condition is sent on the bus.
 *
 ****************************************************************************/

static inline void n32_i2c_sendstop(FAR struct n32_i2c_priv_s *priv)
{
  i2cinfo("Sending STOP\n");

  n32_i2c_modifyreg32(priv, N32_I2C_CTRL2_OFFSET, 0, I2C_CTRL2_STOP);
}

/****************************************************************************
 * Name: n32_i2c_getstatus
 *
 * Description:
 *   Get the STSINT status register.
 *
 ****************************************************************************/

static inline uint32_t n32_i2c_getstatus(FAR struct n32_i2c_priv_s *priv)
{
  return n32_i2c_getreg32(priv, N32_I2C_STSINT_OFFSET);
}

/****************************************************************************
 * Name: n32_i2c_clearinterrupts
 *
 * Description:
 *   Clear all interrupts.
 *
 ****************************************************************************/

static inline void n32_i2c_clearinterrupts(FAR struct n32_i2c_priv_s *priv)
{
  /* INTCLR is write-1-to-clear: write the mask, never read-modify-write
   * it.
   */

  n32_i2c_putreg32(priv, N32_I2C_INTCLR_OFFSET, I2C_INTCLR_CLEARMASK);
}

/****************************************************************************
 * Name: n32_i2c_isr_process
 *
 * Description:
 *  Common interrupt service routine (ISR) that handles I2C protocol logic.
 *  This is instantiated for each configured I2C interface
 *  (I2C1, I2C2, ...).
 *
 *  This ISR is activated and deactivated by:
 *
 *   n32_i2c_process
 *    and
 *   n32_i2c_sem_waitdone
 *
 * Input Parameters:
 *   priv - The private struct of the I2C driver.
 *
 * Returned Value:
 *   OK.
 *
 ****************************************************************************/

static int n32_i2c_isr_process(FAR struct n32_i2c_priv_s *priv)
{
  uint32_t status;
#ifdef CONFIG_I2C_POLLED
  irqstate_t state;
#endif

  /* Get state of the I2C controller */

  status = n32_i2c_getreg32(priv, N32_I2C_STSINT_OFFSET);

  /* Update private version of the state assuming a good state */

  priv->status = status & ~I2C_INT_BAD_STATE;

  /* ------------------- Start of I2C protocol handling ------------------ */

  /* I2C protocol logic follows. It's organized in an if else chain such
   * that only one mode of operation is executed every time the ISR is
   * called.
   *
   * If you need to add additional states to support new features be sure
   * they continue the chain (i.e. begin with "else if") and are placed
   * before the empty call / error states at the end of the chain.
   */

  /* NACK Handling
   *
   * This branch is only triggered when the NACK (Not Acknowledge Received)
   * flag is set.  The NAKF interrupt will only fire when the
   * CTRL1->NAKIE bit is 1.
   *
   * STSINT->NAKF is set by hardware when a NACK is received after a
   * byte is transmitted and the slave fails to acknowledge it.  This is
   * the opposite of, and mutually exclusive to, the STSINT->WRAVL event.
   *
   * The reference manual says the peripheral sends a STOP by itself after
   * a NACK, but on the N32H760 it does not: BUSY stays set until a STOP is
   * requested (measured on the bus, address NACK in both directions).  The
   * STOP is therefore requested here, then the transfer is terminated and
   * the failure reported.
   *
   * To differentiate an "address NACK" from a NACK that might occur during
   * the transfer of other bytes the "priv->astart" parameter is
   * used.  This flag is set to TRUE in sendstart() and set to FALSE when
   * the first WRAVL event is received, which would be after the first byte
   * (the address) is transmitted successfully (acknowledged).
   */

  if (status & I2C_STSINT_NAKF)
    {
      /* Set flags to terminate message transmission:
       *
       *   set message length to -1 to indicate last byte of message sent
       *   set message count to 0 to indicate no more messages to send
       *
       * As we fall through the logic in the ISR the message handling block
       * will be triggered by these flags and signal the ISR to terminate.
       */

      priv->dcnt = -1;
      priv->msgc = 0;

      /* Without this the bus would still be BUSY at the next transfer,
       * which would then be recovered for every unanswered address.
       */

      n32_i2c_sendstop(priv);
    }

  /* Transmit Interrupt Status (WRAVL) Handler
   *
   * This branch is only triggered when the WRAVL event occurs.
   *
   * This indicates the transmit data register WDR has been emptied
   * following the successful transmission of a byte and slave
   * acknowledgment.  In this state the WDR register is ready to accept
   * another byte for transmission.  The WRAVL bit will be cleared
   * automatically when the next byte is written to WDR.
   *
   * The number of WRAVL events during the transfer corresponds to BYTECNT.
   *
   * The WRAVL flag is not set when a NACK is received.
   *
   * When REFILL is disabled (REFILL=0) and BYTECNT data have been
   * transferred:
   *
   *   - In Automatic End Mode (AUTOSTOP=1), a STOP is automatically sent.
   *
   *     Note:  Automatic End Mode is not supported.
   *
   *   - In Software End Mode (AUTOSTOP=0), the TFC event occurs and the
   *     SCL line is stretched low in order to allow software actions
   *     (STOP, RESTART).
   *
   * When REFILL is enabled (REFILL=1) and BYTECNT bytes have been
   * transferred a TFCR event occurs instead and that handler simply
   * updates BYTECNT which causes WRAVL events to continue.  The process
   * repeats until all bytes in the message have been transferred.
   */

  else if ((priv->flags & I2C_M_READ) == 0 &&
           (status & I2C_STSINT_WRAVL) != 0)
    {
      /* WRAVL event occurred, address valid, ready to transmit */

      /* The first event after the address byte is sent will be either
       * WRAVL or NAKF so it's safe to set the astart flag to false on
       * the first WRAVL event to indicate that it is no longer necessary
       * to check for address validity.
       */

      if (priv->astart)
        {
          /* First byte of the message: a TFC left over from before a
           * RESTART is gone by now, let it report the end again.
           */

          priv->astart = false;
          n32_i2c_modifyreg32(priv, N32_I2C_CTRL1_OFFSET,
                              0, I2C_CTRL1_TFCIE);
        }

      /* If one or more bytes in the current message are ready to
       * transmit.
       */

      if (priv->dcnt > 0)
        {
          /* Decrement byte counter */

          priv->dcnt--;

          /* If we are about to transmit the last byte in the current
           * message.
           */

          if (priv->dcnt == 0)
            {
              /* If this is also the last message to send, disable REFILL
               * so TFC fires next and issues STOP condition.  If we don't
               * do this TFCR will fire next, and since there are no bytes
               * to send we can't write BYTECNT to clear TFCR so it will
               * fire forever.
               */

              if (priv->msgc == 1)
                {
                  n32_i2c_disable_reload(priv);
                }
            }

          /* Transmit current byte */

          n32_i2c_putreg32(priv, N32_I2C_WDR_OFFSET, *priv->ptr);

          /* Advance to next byte */

          priv->ptr++;
        }
      else
        {
          /* Unsupported state: indicate the bad state, so that on
           * termination HW will be reset.
           */

          priv->status |= I2C_INT_BAD_STATE;
        }
    }

  /* Receive Buffer Not Empty (RDAVL) State Handler
   *
   * This branch is only triggered when the RDAVL event occurs.
   *
   * This indicates data has been received from the bus and is waiting to
   * be read from the RDR register.  When RDR is read this bit is
   * automatically cleared and then an ACK or NACK is sent depending on
   * whether we have more bytes to receive.
   *
   * When REFILL is disabled and bytes remain to be transferred an
   * acknowledge is automatically sent on the bus and the RDAVL events
   * continue until the last byte is received.
   *
   * When REFILL is disabled (REFILL=0) and BYTECNT bytes have been
   * transferred:
   *
   *   - In Automatic End Mode (AUTOSTOP=1), a NACK and a STOP are
   *     automatically sent after the last received byte.
   *
   *     Note:  Automatic End Mode is not supported.
   *
   *   - In Software End Mode (AUTOSTOP=0), a NACK is automatically sent
   *     after the last received byte, the TFC event occurs and the SCL
   *     line is stretched low in order to allow software actions (STOP,
   *     RESTART).
   *
   * When REFILL is enabled (REFILL=1) and BYTECNT bytes have been
   * transferred a TFCR event occurs and that handler simply updates
   * BYTECNT which causes RDAVL events to continue until all bytes have
   * been transferred.
   */

  else if ((priv->flags & I2C_M_READ) != 0 &&
           (status & I2C_STSINT_RDAVL) != 0)
    {
      /* When read flag is set and the receive buffer is not empty
       * (RDAVL is set) then the driver can read from the data register.
       */

      if (priv->astart)
        {
          /* As in the transmit branch above */

          priv->astart = false;
          n32_i2c_modifyreg32(priv, N32_I2C_CTRL1_OFFSET,
                              0, I2C_CTRL1_TFCIE);
        }

      /* If more bytes in the current message */

      if (priv->dcnt > 0)
        {
          /* No interrupts or context switches may occur in the following
           * sequence.  Otherwise, additional bytes may be received.
           */

#ifdef CONFIG_I2C_POLLED
          state = enter_critical_section();
#endif

          /* Receive a byte */

          *priv->ptr = (uint8_t)n32_i2c_getreg32(priv, N32_I2C_RDR_OFFSET);

          /* Advance buffer to the next byte in the message */

          priv->ptr++;

          /* Signal byte received */

          priv->dcnt--;

#ifdef CONFIG_I2C_POLLED
          leave_critical_section(state);
#endif
        }
      else
        {
          /* Unsupported state */

          /* Set signals that will terminate ISR and wake waiting thread */

          priv->status |= I2C_INT_BAD_STATE;
          priv->dcnt = -1;
          priv->msgc = 0;
        }
    }

  /* Transfer Complete (TFC) State Handler
   *
   * This branch is only triggered when the TFC event occurs.  This
   * event will only fire when:
   *
   * CTRL1->TFCIE = 1 (Transfer Complete Interrupts Enabled)
   * CTRL2->REFILL = 0 (Reload Mode Disabled)
   * CTRL2->AUTOSTOP = 0 (Automatic End Mode Disabled, i.e. Software End
   * Mode)
   *
   * This event indicates that the number of bytes initially defined
   * in BYTECNT, meaning, the number of bytes in the current message
   * (priv->dcnt) has been successfully transmitted or received.
   *
   * When the TFC event occurs we have two choices to clear it and
   * move on, regardless of the transfer direction:
   *
   *    - if more messages follow, perform a repeated START if required
   *      and then fall through to transmit or receive the next message.
   *
   *    - if no messages follow, perform a STOP and set flags needed to
   *      exit the ISR.
   *
   * The fact that the hardware must either RESTART or STOP when a TFC
   * event occurs explains why, when messages must be sent back to back
   * (i.e. without a restart by specifying the I2C_M_NOSTART flag),
   * REFILL mode must be enabled and TFCR event(s) must be generated
   * instead.  See the TFCR handler for more.
   */

  else if ((status & I2C_STSINT_TFC) != 0)
    {
      /* TFC while bytes of the current message are still outstanding is
       * the one of the message before a RESTART: on the N32H760 START does
       * not clear the flag until the repeated START is on the wire, so its
       * interrupt stays pending.  Leave it to the first byte of the new
       * message to let TFC report again.
       */

      if (priv->dcnt != 0)
        {
          n32_i2c_modifyreg32(priv, N32_I2C_CTRL1_OFFSET,
                              I2C_CTRL1_TFCIE, 0);
          return OK;
        }

      /* Prior message has been sent successfully. Or there could have
       * been an error that set msgc to 0; So test for that case as
       * we do not want to decrement msgc less then zero nor move msgv
       * past the last message.
       */

      if (priv->msgc > 0)
        {
          priv->msgc--;
        }

      /* Are there additional messages remain to be transmitted / received? */

      if (priv->msgc > 0)
        {
          i2cinfo("TFC: RESTART: dcnt=%i, msgc=%i\n",
                  priv->dcnt, priv->msgc);

          /* Issue a START condition.
           *
           * Note that the first thing sendstart does is update the
           * private structure "current message" data (ptr, dcnt, flags)
           * so they all reflect the next message in the list so we
           * update msgv before we get there.
           */

          /* Advance to the next message in the list */

          priv->msgv++;

          n32_i2c_sendstart(priv);
        }
      else
        {
          /* Issue a STOP conditions.
           *
           * No additional messages to transmit / receive, so the
           * transfer is indeed complete.  Nothing else to do but
           * issue a STOP and exit.
           */

          i2cinfo("TFC: STOP: dcnt=%i msgc=%i\n",
                  priv->dcnt, priv->msgc);

          n32_i2c_sendstop(priv);

          /* Set signals that will terminate ISR and wake waiting thread */

          priv->dcnt = -1;
          priv->msgc = 0;
        }
    }

  /* Transfer Complete (Reload) State Handler
   *
   * This branch is only triggered when the TFCR event occurs.  This
   * event will only fire when:
   *
   *  CTRL1->TFCIE = 1 (Transfer Complete Interrupts Enabled)
   *  CTRL2->REFILL = 1 (Reload Mode Active)
   *  CTRL2->AUTOSTOP = 0 (Automatic End Mode Disabled, i.e. Software End
   *  Mode)
   *
   * This is similar to the TFC event except that TFCR assumes that
   * additional bytes are available to transfer.  So despite what its name
   * might imply the transfer really isn't complete.
   *
   * There are two reasons REFILL would be enabled:
   *
   *  1) We're trying to send a message with a payload greater than 255
   *     bytes.
   *  2) We're trying to send messages back to back, regardless of their
   *     payload size, to avoid a RESTART (i.e. I2C_M_NOSTART flag is set).
   *
   * These conditions may be true simultaneously, as would be the case if
   * we're sending multiple messages with payloads > 255 bytes.   So we
   * only advance to the next message if we arrive here and dcnt is 0,
   * meaning, we're finished with the last message and ready to move to
   * the next.
   *
   * This logic supports the transfer of bytes limited only by the size of
   * the i2c_msg_s length variable.  The SCL line will be stretched low
   * until BYTECNT is written with a non-zero value, allowing the transfer
   * to continue.
   *
   * RESTARTs are required by the I2CSPEC if the next message transfer
   * direction changes.  The NORESTART flag overrides this behavior, so
   * I2C_M_NOSTART is only valid between messages of the same direction.
   */

  else if ((status & I2C_STSINT_TFCR) != 0)
    {
      /* If no more bytes in the current message to transfer */

      if (priv->dcnt == 0)
        {
          /* Prior message has been sent successfully */

          priv->msgc--;

          /* Advance to the next message in the list */

          priv->msgv++;

          /* Update current message data */

          priv->ptr    = priv->msgv->buffer;
          priv->dcnt   = priv->msgv->length;
          priv->flags  = priv->msgv->flags;

          /* If this is the last message, disable reload so the
           * TFC event fires next time.
           */

          if (priv->msgc == 0)
            {
              i2cinfo("TFCR: DISABLE RELOAD: dcnt = %i msgc = %i\n",
                      priv->dcnt, priv->msgc);

              n32_i2c_disable_reload(priv);
            }

          /* Update BYTECNT with length of current message */

          i2cinfo("TFCR: NEXT MSG dcnt = %i msgc = %i\n",
                  priv->dcnt, priv->msgc);

          n32_i2c_set_bytes_to_transfer(priv, priv->dcnt);
        }
      else
        {
          /* More bytes in the current (greater than 255 byte payload
           * length) message, so set BYTECNT according to the bytes
           * remaining in the message, up to a maximum each cycle of 255.
           */

          if (priv->dcnt > 255)
            {
              i2cinfo("TFCR: ENABLE RELOAD: BYTECNT = 255 dcnt = %i"
                      " msgc = %i\n",
                      priv->dcnt, priv->msgc);

              /* More than 255 bytes to transfer so the REFILL bit is
               * set in order to generate a TFCR event rather than a TFC
               * event when 255 bytes are successfully transferred.
               * This forces us to return here to update BYTECNT and
               * continue until BYTECNT is set to less than 255 bytes,
               * at which point REFILL will be disabled and a TFC
               * event will (eventually) follow to officially terminate
               * the transfer.
               */

              n32_i2c_enable_reload(priv);

              n32_i2c_set_bytes_to_transfer(priv, 255);
            }
          else
            {
              /* Less than 255 bytes left to transfer, which means we'll
               * complete the transfer of all bytes in the current message
               * the next time around.
               *
               * This means we need to disable the REFILL functionality so
               * we receive a TFC event next time which will allow us to
               * either RESTART and continue sending the contents of the
               * next message or send a STOP condition and exit the ISR.
               */

              i2cinfo("TFCR: DISABLE RELOAD: BYTECNT = dcnt = %i"
                      " msgc = %i\n",
                      priv->dcnt, priv->msgc);

              n32_i2c_set_bytes_to_transfer(priv, priv->dcnt);

              n32_i2c_disable_reload(priv);
            }
        }
    }

  /* Empty call handler
   *
   * Case to handle an empty call to the ISR where it has nothing to
   * do and should exit immediately.
   */

  else if (priv->dcnt == -1 && priv->msgc == 0)
    {
      status = n32_i2c_getreg32(priv, N32_I2C_STSINT_OFFSET);
      i2cwarn("WARNING: EMPTY CALL: Stopping ISR: status 0x%08" PRIx32 "\n",
              status);
    }

  /* An interrupt taken for a flag that is already gone again.  The TFC of
   * the message before a RESTART stays pending until the repeated START is
   * on the wire, and that START clears it just as this routine looks.  The
   * transfer is still running and its next event reports again; one that
   * never reports is caught by the wait timeout.
   */

  else if ((status & ~(I2C_STSINT_WRE | I2C_STSINT_BUSY)) == 0)
    {
      i2cwarn("WARNING: no event flag: status 0x%08" PRIx32 "\n", status);
    }

  /* Error handler
   *
   * We get to this branch only if we can't handle the current state.
   *
   * This can happen in interrupt based operation on ABLO & BUSY.
   *
   * This will happen during polled operation when the device is not
   * in one of the supported states when polled.
   */

  else
    {
#ifndef CONFIG_I2C_POLLED
      /* Set condition to terminate ISR and wake waiting thread */

      priv->status |= I2C_INT_BAD_STATE;
      priv->dcnt = -1;
      priv->msgc = 0;
#endif
    }

  /* --------------------- End of I2C protocol handling ------------------ */

  /* Message Handling
   *
   * Transmission of the whole message chain has been completed. We have to
   * terminate the ISR and wake up n32_i2c_process() that is waiting for
   * the ISR cycle to handle the sending/receiving of the messages.
   */

  if (priv->dcnt == -1 && priv->msgc == 0)
    {
      i2cinfo("MSG: Shutting down I2C ISR\n");

      /* Clear pointer to message content to reflect we are done
       * with the current transaction.
       */

      priv->msgv = NULL;

#ifdef CONFIG_I2C_POLLED
      priv->intstate = INTSTATE_DONE;
#else
      /* We will update private status to capture NACK which is used in
       * combination with the astart flag to report the type of NACK received
       * (address vs data) to the upper layers once we exit the ISR.
       *
       * Note: status is captured  prior to clearing interrupts because
       * the NAKF flag will naturally be cleared by that process.
       */

      status = n32_i2c_getreg32(priv, N32_I2C_STSINT_OFFSET);

      /* Clear all interrupts */

      n32_i2c_clearinterrupts(priv);

      /* Was a bad state detected in the processing? */

      if (priv->status & I2C_INT_BAD_STATE)
        {
          /* SW reset device.  Clearing I2CEN releases the lines and
           * resets the state machine; it stays low well beyond the
           * minimum software reset time and is set again by
           * n32_i2c_setclock() or n32_i2c_recover().
           */

          n32_i2c_modifyreg32(priv, N32_I2C_CTRL1_OFFSET,
                              I2C_CTRL1_I2CEN, 0);
        }

      /* Update private status from above, keeping I2C_INT_BAD_STATE so
       * that the waiting thread can report it.
       */

      priv->status = status | (priv->status & I2C_INT_BAD_STATE);

      /* If a thread is waiting then inform it transfer is complete */

      if (priv->intstate == INTSTATE_WAITING)
        {
          nxsem_post(&priv->sem_isr);
          priv->intstate = INTSTATE_DONE;
        }
#endif
    }

  return OK;
}

/****************************************************************************
 * Name: n32_i2c_isr
 *
 * Description:
 *   Common I2C interrupt service routine, attached to both the event and
 *   the error interrupt of an instance.
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int n32_i2c_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct n32_i2c_priv_s *priv = (FAR struct n32_i2c_priv_s *)arg;

  DEBUGASSERT(priv != NULL);
  UNUSED(irq);
  UNUSED(context);
  return n32_i2c_isr_process(priv);
}
#endif

/****************************************************************************
 * Name: n32_i2c_kerclk
 *
 * Description:
 *   Configure the I2C kernel clock of the RCC group of an instance: select
 *   the divided system bus clock as the source and prescale it by 4.
 *
 *   The prescaler is shared by all instances of the group, so it is
 *   programmed only once per group; every instance of a group must use
 *   this source.  From then on the kernel clock is I2C_KERNEL_HZ.
 *
 * Input Parameters:
 *   config - The hardware configuration of the instance.
 *
 ****************************************************************************/

static void n32_i2c_kerclk(FAR const struct n32_i2c_config_s *config)
{
  if (g_i2c_clkcfg[config->group])
    {
      return;
    }

  g_i2c_clkcfg[config->group] = true;

  /* Select the divided system bus clock as the kernel clock source */

  modifyreg32(config->sel_reg, config->sel_mask, config->sel_sysdiv);

  /* Set the prescaler of the group; it acts on that source only */

  modifyreg32(config->div_reg, config->div_mask, config->div_div4);
}

/****************************************************************************
 * Name: n32_i2c_init
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults.
 *
 *   The peripheral clock is already enabled by the board start-up logic
 *   and is left untouched.
 *
 ****************************************************************************/

static int n32_i2c_init(FAR struct n32_i2c_priv_s *priv)
{
  /* Power-up and configure GPIOs */

  /* Reset the peripheral */

  modifyreg32(priv->config->rst_reg, 0, priv->config->rst_bit);
  modifyreg32(priv->config->rst_reg, priv->config->rst_bit, 0);

  /* Select and prescale the kernel clock of the group */

  n32_i2c_kerclk(priv->config);

  /* Configure pins */

  if (n32_configgpio(priv->config->scl_pin) < 0)
    {
      return ERROR;
    }

  if (n32_configgpio(priv->config->sda_pin) < 0)
    {
      n32_unconfiggpio(priv->config->scl_pin);
      return ERROR;
    }

#ifndef CONFIG_I2C_POLLED
  /* Attach error and event interrupts to the ISR */

  irq_attach(priv->config->ev_irq, n32_i2c_isr, priv);
  irq_attach(priv->config->er_irq, n32_i2c_isr, priv);
  up_enable_irq(priv->config->ev_irq);
  up_enable_irq(priv->config->er_irq);
#endif

  /* Force a frequency update */

  priv->frequency = 0;
  n32_i2c_setclock(priv, 100000);

  return OK;
}

/****************************************************************************
 * Name: n32_i2c_deinit
 *
 * Description:
 *   Shutdown the I2C hardware.
 *
 ****************************************************************************/

static int n32_i2c_deinit(FAR struct n32_i2c_priv_s *priv)
{
  /* Disable I2C */

  n32_i2c_putreg32(priv, N32_I2C_CTRL1_OFFSET, 0);

  /* Unconfigure GPIO pins */

  n32_unconfiggpio(priv->config->scl_pin);
  n32_unconfiggpio(priv->config->sda_pin);

#ifndef CONFIG_I2C_POLLED

  /* Disable and detach interrupts */

  up_disable_irq(priv->config->ev_irq);
  up_disable_irq(priv->config->er_irq);
  irq_detach(priv->config->ev_irq);
  irq_detach(priv->config->er_irq);
#endif

  return OK;
}

/****************************************************************************
 * Name: n32_i2c_recover
 *
 * Description:
 *   Clear a wedged bus: disable the peripheral, turn SCL and SDA into
 *   open-drain GPIO outputs, clock SCL until SDA is released, generate a
 *   START and a STOP by hand, then restore the pins, the timing and the
 *   peripheral.
 *
 *   Every wait in this function is bounded.  The caller must hold the bus
 *   mutex.
 *
 * Input Parameters:
 *   priv - The private struct of the I2C driver.
 *
 * Returned Value:
 *   OK on success, -EIO if the bus could not be cleared.
 *
 ****************************************************************************/

static int n32_i2c_recover(FAR struct n32_i2c_priv_s *priv)
{
  uint32_t scl_gpio = priv->config->scl_gpio;
  uint32_t sda_gpio = priv->config->sda_gpio;
  unsigned int clock_count;
  unsigned int stretch_count;
  int ret;

  /* Disable the peripheral: this is a software reset that releases SCL
   * and SDA.
   */

  n32_i2c_modifyreg32(priv, N32_I2C_CTRL1_OFFSET, I2C_CTRL1_I2CEN, 0);

  /* Preload the output latches high, then turn SCL and SDA into
   * open-drain GPIO outputs.  n32_configgpio() does not set the output
   * level, so the latches are set first to keep both lines released.
   */

  n32_gpiowrite(scl_gpio, true);
  n32_gpiowrite(sda_gpio, true);
  n32_configgpio(sda_gpio);
  n32_configgpio(scl_gpio);

  /* Let SDA go high */

  n32_gpiowrite(sda_gpio, true);

  ret = OK;

  /* Clock the bus until any slaves currently driving it let it go. */

  clock_count = 0;
  while (!n32_gpioread(sda_gpio))
    {
      /* Give up if we have tried too hard */

      if (clock_count++ >= 9)
        {
          ret = -EIO;
          goto out;
        }

      /* Sniff to make sure that clock stretching has finished.
       *
       * If the bus never relaxes, the reset has failed.
       */

      stretch_count = 0;
      while (!n32_gpioread(scl_gpio))
        {
          /* Give up if we have tried too hard */

          if (stretch_count++ >= 10)
            {
              ret = -EIO;
              goto out;
            }

          up_udelay(10);
        }

      /* Drive SCL low */

      n32_gpiowrite(scl_gpio, false);
      up_udelay(10);

      /* Drive SCL high again */

      n32_gpiowrite(scl_gpio, true);
      up_udelay(10);
    }

  /* Generate a start followed by a stop to reset slave state machines. */

  n32_gpiowrite(sda_gpio, false);
  up_udelay(10);
  n32_gpiowrite(scl_gpio, false);
  up_udelay(10);
  n32_gpiowrite(scl_gpio, true);
  up_udelay(10);
  n32_gpiowrite(sda_gpio, true);
  up_udelay(10);

out:

  if (ret != OK)
    {
      i2cerr("ERROR: I2C bus recovery failed, SDA is still low\n");
    }

  /* Restore the alternate function pin configuration. */

  n32_configgpio(priv->config->scl_pin);
  n32_configgpio(priv->config->sda_pin);

  /* Clear pending flags, reprogram the cached timing and enable the
   * peripheral.  It has been disabled for far longer than the minimum
   * software reset time.
   */

  n32_i2c_putreg32(priv, N32_I2C_BUSTM_OFFSET,
                   n32_i2c_timing(priv->frequency));
  n32_i2c_modifyreg32(priv, N32_I2C_CTRL2_OFFSET,
                      I2C_CTRL2_START | I2C_CTRL2_STOP |
                      I2C_CTRL2_REFILL, 0);
  n32_i2c_clearinterrupts(priv);
  n32_i2c_modifyreg32(priv, N32_I2C_CTRL1_OFFSET, 0, I2C_CTRL1_I2CEN);

  return ret;
}

/****************************************************************************
 * Name: n32_i2c_process
 *
 * Description:
 *   Common I2C transfer logic.
 *
 *   Initiates a master mode transaction on the I2C bus to transfer the
 *   provided messages to and from the slave devices.  The caller must
 *   hold the bus mutex.
 *
 * Input Parameters:
 *   dev   - The I2C device instance.
 *   msgs  - The messages to transfer.
 *   count - The number of messages.
 *
 * Returned Value:
 *   Zero (OK) on success, a negated errno value on failure.
 *
 ****************************************************************************/

static int n32_i2c_process(FAR struct i2c_master_s *dev,
                           FAR struct i2c_msg_s *msgs, int count)
{
  FAR struct n32_i2c_inst_s *inst = (FAR struct n32_i2c_inst_s *)dev;
  FAR struct n32_i2c_priv_s *priv = inst->priv;
  clock_t start;
  clock_t timeout;
  uint32_t status;
  uint32_t cr1;
  uint32_t cr2;
  int errval = 0;
  int waitrc;
  int i;

  /* Reject the requests this driver cannot handle: the message count
   * must fit the 8-bit message counter, a message must carry at least one
   * byte and 10-bit addressing is not implemented.
   */

  if (count <= 0 || count > 255 || msgs == NULL)
    {
      return -EINVAL;
    }

  for (i = 0; i < count; i++)
    {
      if (msgs[i].buffer == NULL || msgs[i].length <= 0)
        {
          return -EINVAL;
        }

      if ((msgs[i].flags & I2C_M_TEN) != 0)
        {
          return -ENOTSUP;
        }
    }

  /* Wait for any STOP in progress */

  n32_i2c_sem_waitstop(priv);

  /* Clear any pending error interrupts */

  n32_i2c_clearinterrupts(priv);

  /* The user manual does not define what START does while BUSY is set, so
   * wait for a free bus before the first START.  Recover the bus if it
   * does not release.
   */

  if (!n32_i2c_waitnotbusy(priv))
    {
      i2cwarn("WARNING: I2C bus busy, recovering\n");
      n32_i2c_recover(priv);

      if (!n32_i2c_waitnotbusy(priv))
        {
          i2cerr("ERROR: I2C Bus busy\n");
          return -EBUSY;
        }
    }

  /* Old transfers are done */

  priv->msgv = msgs;
  priv->msgc = count;

  /* Set I2C clock frequency; this is a SW reset when the frequency
   * changes.
   */

  n32_i2c_setclock(priv, msgs->frequency);

  /* Trigger start condition, then the process moves into the ISR.  I2C
   * interrupts will be enabled within n32_i2c_sem_waitdone().
   */

  /* A read that ended early can leave a byte in the receive register.  The
   * RDAVL flag cannot be cleared while it is occupied and the byte does
   * not belong to this transfer, so discard it.
   */

  if ((n32_i2c_getstatus(priv) & I2C_STSINT_RDAVL) != 0)
    {
      n32_i2c_getreg32(priv, N32_I2C_RDR_OFFSET);
    }

  priv->status = 0;

#ifndef CONFIG_I2C_POLLED
  /* Enable transmit, receive and NACK interrupts here so when we send the
   * start condition below the ISR will fire if the data was sent and some
   * response from the slave received.  All other interrupts relevant to
   * our needs are enabled in n32_i2c_sem_waitdone() below.
   */

  n32_i2c_enableinterrupts(priv, I2C_CTRL1_TXRX | I2C_CTRL1_NAKIE);
#endif

  /* Trigger START condition generation, which also sends the slave address
   * with read/write flag and the data in the first message
   */

  n32_i2c_sendstart(priv);

  /* Wait for the ISR to tell us that the transfer is complete by attempting
   * to grab the semaphore that is initially locked by the ISR.  If the ISR
   * does not release the lock so we can obtain it here prior to the end
   * of the timeout period waitdone returns error and we report a timeout.
   */

  waitrc = n32_i2c_sem_waitdone(priv);

  cr1 = n32_i2c_getreg32(priv, N32_I2C_CTRL1_OFFSET);
  cr2 = n32_i2c_getreg32(priv, N32_I2C_CTRL2_OFFSET);

  /* Status after a normal / good exit usually has only the WRE bit set,
   * meaning the WDR register is empty, which it naturally is after the
   * last byte was transmitted.  The priv->status field can hold
   * additional information like a NACK, so we use that information.
   */

  status = priv->status;

  if (waitrc < 0)
    {
      /* Connection timed out.  The aborted transfer may have left a slave
       * mid-byte holding SDA low, so clear the bus before the next one.
       */

      errval = ETIMEDOUT;
      i2cerr("ERROR: Waitdone timed out CR1: 0x%08" PRIx32
             " CR2: 0x%08" PRIx32 " status: 0x%08" PRIx32 "\n",
             cr1, cr2, status);
      n32_i2c_recover(priv);
    }
  else
    {
      i2cinfo("Waitdone success: CR1: 0x%08" PRIx32
              " CR2: 0x%08" PRIx32 " status: 0x%08" PRIx32 "\n",
              cr1, cr2, status);
    }

  /* Check for error status conditions */

  if ((status & (I2C_STSINT_BSER |
                 I2C_STSINT_ABLO |
                 I2C_STSINT_OVF |
                 I2C_STSINT_CRCERR |
                 I2C_STSINT_TMOUT |
                 I2C_STSINT_NAKF)) != 0)
    {
      /* one or more errors in the mask are present */

      if (status & I2C_STSINT_BSER)
        {
          /* Bus Error */

          i2cerr("ERROR: I2C Bus Error\n");
          errval = EIO;
        }
      else if (status & I2C_STSINT_ABLO)
        {
          /* Arbitration Lost (master mode) */

          i2cerr("ERROR: I2C Arbitration Lost\n");
          errval = EAGAIN;
        }
      else if (status & I2C_STSINT_OVF)
        {
          /* Overrun/Underrun */

          i2cerr("ERROR: I2C Overrun/Underrun\n");
          errval = EIO;
        }
      else if (status & I2C_STSINT_CRCERR)
        {
          /* PEC Error in reception (SMBus Only) */

          i2cerr("ERROR: I2C PEC Error\n");
          errval = EPROTO;
        }
      else if (status & I2C_STSINT_TMOUT)
        {
          /* Timeout or Tlow Error (SMBus Only) */

          i2cerr("ERROR: I2C Timeout / Tlow Error\n");
          errval = ETIME;
        }
      else if (status & I2C_STSINT_NAKF)
        {
          /* NACK Received, flag as "communication error on send" */

          if (priv->astart == true)
            {
              i2cwarn("WARNING: I2C Address NACK\n");
              errval = EADDRNOTAVAIL;
            }
          else
            {
              i2cwarn("WARNING: I2C Data NACK\n");
              errval = ECOMM;
            }
        }
      else
        {
          /* Unrecognized error */

          i2cerr("ERROR: I2C Unrecognized Error status: 0x%08" PRIx32 "\n",
                 status);
          errval = EINTR;
        }
    }

  /* The state machine ran into a state it cannot handle; the ISR has
   * reset the peripheral.  This is not an error of the bus, but should
   * not happen.
   */

  else if ((status & I2C_INT_BAD_STATE) != 0)
    {
      i2cerr("ERROR: I2C invalid state, status: 0x%08" PRIx32 "\n", status);
      errval = EINTR;
    }

  /* The BUSY flag can remain set if devices on the bus are in an odd
   * state and need to be reset, or if the wait above failed.  It is
   * normally still set while the STOP of the transfer is on the wire, so
   * give the bus about one bit period to release.  A bus that is still
   * busy then is recovered.
   */

  if ((n32_i2c_getstatus(priv) & I2C_STSINT_BUSY) != 0)
    {
      start = clock_systime_ticks();
      timeout = USEC2TICK(USEC_PER_SEC / priv->frequency) + 1;

      while ((n32_i2c_getstatus(priv) & I2C_STSINT_BUSY) != 0)
        {
          if ((clock_systime_ticks() - start) > timeout)
            {
              break;
            }
        }

      if ((n32_i2c_getstatus(priv) & I2C_STSINT_BUSY) != 0)
        {
          i2cerr("ERROR: I2C Bus busy\n");
          n32_i2c_recover(priv);
          if (errval == 0)
            {
              errval = EBUSY;
            }
        }
    }

  return -errval;
}

/****************************************************************************
 * Name: n32_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function.  The bus mutex is held for the whole
 *   multi-message transfer; the framework has no bus lock of its own.
 *
 ****************************************************************************/

static int n32_i2c_transfer(FAR struct i2c_master_s *dev,
                            FAR struct i2c_msg_s *msgs, int count)
{
  FAR struct n32_i2c_priv_s *priv;
  int ret;

  DEBUGASSERT(dev != NULL);

  /* Get I2C private structure */

  priv = ((FAR struct n32_i2c_inst_s *)dev)->priv;

  /* Ensure that address or flags don't change meanwhile */

  ret = nxmutex_lock(&priv->lock);
  if (ret >= 0)
    {
      ret = n32_i2c_process(dev, msgs, count);
      nxmutex_unlock(&priv->lock);
    }

  return ret;
}

/****************************************************************************
 * Name: n32_i2c_reset
 *
 * Description:
 *   Reset an I2C bus.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static int n32_i2c_reset(FAR struct i2c_master_s *dev)
{
  FAR struct n32_i2c_priv_s *priv;
  int ret;

  DEBUGASSERT(dev != NULL);

  /* Get I2C private structure */

  priv = ((FAR struct n32_i2c_inst_s *)dev)->priv;

  /* Our caller must own a ref */

  DEBUGASSERT(priv->refs > 0);

  /* Lock out other clients */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = n32_i2c_recover(priv);

  /* Release the port for reuse by other clients */

  nxmutex_unlock(&priv->lock);
  return ret;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus.
 *
 * Input Parameters:
 *   port - I2C port number, 1 based.
 *
 * Returned Value:
 *   A reference to the I2C interface structure, NULL on failure.
 *
 ****************************************************************************/

FAR struct i2c_master_s *n32_i2cbus_initialize(int port)
{
  FAR struct n32_i2c_priv_s *priv = NULL;  /* private data of device with multiple instances */
  FAR struct n32_i2c_inst_s *inst = NULL;  /* device, single instance */
  int ret = OK;

  /* Get I2C private structure */

  switch (port)
    {
#ifdef CONFIG_N32H7_I2C1
      case 1:
        priv = &n32_i2c1_priv;
        break;
#endif
#ifdef CONFIG_N32H7_I2C2
      case 2:
        priv = &n32_i2c2_priv;
        break;
#endif
#ifdef CONFIG_N32H7_I2C3
      case 3:
        priv = &n32_i2c3_priv;
        break;
#endif
#ifdef CONFIG_N32H7_I2C4
      case 4:
        priv = &n32_i2c4_priv;
        break;
#endif
#ifdef CONFIG_N32H7_I2C5
      case 5:
        priv = &n32_i2c5_priv;
        break;
#endif
#ifdef CONFIG_N32H7_I2C6
      case 6:
        priv = &n32_i2c6_priv;
        break;
#endif
#ifdef CONFIG_N32H7_I2C7
      case 7:
        priv = &n32_i2c7_priv;
        break;
#endif
#ifdef CONFIG_N32H7_I2C8
      case 8:
        priv = &n32_i2c8_priv;
        break;
#endif
      default:
        return NULL;
    }

  /* Allocate instance */

  inst = kmm_malloc(sizeof(struct n32_i2c_inst_s));
  if (inst == NULL)
    {
      return NULL;
    }

  /* Initialize instance */

  inst->ops  = &n32_i2c_ops;
  inst->priv = priv;

  /* Init private data for the first time, increment refs count,
   * power-up hardware and configure GPIOs.
   */

  nxmutex_lock(&priv->lock);
  if (priv->refs == 0)
    {
      ret = n32_i2c_init(priv);
    }

  if (ret == OK)
    {
      priv->refs++;
    }

  nxmutex_unlock(&priv->lock);

  if (ret < 0)
    {
      kmm_free(inst);
      return NULL;
    }

  return (FAR struct i2c_master_s *)inst;
}

/****************************************************************************
 * Name: n32_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus.
 *
 * Input Parameters:
 *   dev - An instance returned by n32_i2cbus_initialize().
 *
 * Returned Value:
 *   OK on success, a negated errno value on failure.
 *
 ****************************************************************************/

int n32_i2cbus_uninitialize(FAR struct i2c_master_s *dev)
{
  FAR struct n32_i2c_priv_s *priv;
  int ret;

  DEBUGASSERT(dev != NULL);

  priv = ((FAR struct n32_i2c_inst_s *)dev)->priv;

  /* Decrement refs and check for underflow */

  if (priv->refs == 0)
    {
      return ERROR;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (--priv->refs)
    {
      nxmutex_unlock(&priv->lock);
      kmm_free(dev);
      return OK;
    }

  /* Disable power and other HW resource (GPIO's) */

  n32_i2c_deinit(priv);
  nxmutex_unlock(&priv->lock);

  kmm_free(dev);
  return OK;
}

#endif /* defined(CONFIG_N32H7_I2C1) || ... || defined(CONFIG_N32H7_I2C8) */
