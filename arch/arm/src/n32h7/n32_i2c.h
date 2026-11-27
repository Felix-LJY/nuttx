/****************************************************************************
 * arch/arm/src/n32h7/n32_i2c.h
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

#ifndef __ARCH_ARM_SRC_N32H7_N32_I2C_H
#define __ARCH_ARM_SRC_N32H7_N32_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: n32_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus.  The bus can be shared: every call returns a
 *   new instance that refers to the same hardware.
 *
 * Input Parameters:
 *   port - I2C port number, 1 based.
 *
 * Returned Value:
 *   A reference to the I2C interface structure, NULL on failure.
 *
 ****************************************************************************/

EXTERN FAR struct i2c_master_s *n32_i2cbus_initialize(int port);

/****************************************************************************
 * Name: n32_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus instance.  The hardware is shut down when the
 *   last instance is released.
 *
 * Input Parameters:
 *   dev - An instance previously returned by n32_i2cbus_initialize().
 *
 * Returned Value:
 *   OK on success, a negated errno value on failure.
 *
 ****************************************************************************/

EXTERN int n32_i2cbus_uninitialize(FAR struct i2c_master_s *dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_N32H7_N32_I2C_H */
