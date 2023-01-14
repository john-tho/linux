/*******************************************************************************
Copyright (C) 2016 Annapurna Labs Ltd.

This file may be licensed under the terms of the Annapurna Labs Commercial
License Agreement.

Alternatively, this file can be distributed under the terms of the GNU General
Public License V2 as published by the Free Software Foundation and can be
found at http://www.gnu.org/licenses/gpl-2.0.html

Alternatively, redistribution and use in source and binary forms, with or
without modification, are permitted provided that the following conditions are
met:

    *     Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

    *     Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in
the documentation and/or other materials provided with the
distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#ifndef __TPM_IF_I2C_H__
#define __TPM_IF_I2C_H__

#include "al_tpm_if.h"
#include "al_hal_i2c.h"

struct al_tpm_if_i2c_priv {
	struct al_i2c_interface		*i2c_if;
	unsigned int			tpm_i2c_addr;
	unsigned int			guard_time;
};

struct al_tpm_if_i2c_params {
	struct al_i2c_interface		*i2c_if;
	unsigned int			tpm_i2c_addr;
	unsigned int			guard_time;
};

/* Struct is used in case tpm HAL service does not
 * manage the i2c interface which is owned by the OS (e.g. u-boot)
 */
struct al_tpm_if_i2c_gen {
	unsigned int			tpm_i2c_addr;
	unsigned int			mux_channel;
	unsigned int			bus_old;
	unsigned int			bus;
	unsigned int			guard_time;
	int (*i2c_read)(uint8_t chip, unsigned int addr, int alen,
					uint8_t *buffer, int len);
	int (*i2c_write)(uint8_t chip, unsigned int addr, int alen,
					uint8_t *buffer, int len);
	unsigned int (*i2c_get_bus_num)(void);
	int (*i2c_set_bus_num)(unsigned int bus);
};

/**
 * TPM I2C interface handle initialization using initialization structure
 *
 * @param	tpm_if
 *		An uninitialized TPM interface handle
 * @param	tpm_if_priv
 *		An uninitialized TPM I2C private structure
 * @param	al_tpm_if_i2c_params
 *		TPM interface handle initialization structure
 */
void al_tpm_if_i2c_handle_init_aux(
	struct al_tpm_if		*tpm_if,
	struct al_tpm_if_i2c_priv	*tpm_if_priv,
	struct al_tpm_if_i2c_params	*al_tpm_if_i2c_params);

/**
 * TPM I2C interface handle initialization
 *
 * @param	tpm_if
 *		An uninitialized TPM interface handle
 * @param	tpm_if_priv
 *		An uninitialized TPM I2C private structure
 * @param	i2c_if
 *		An initialized I2C interface handle
 * @param	tpm_i2c_addr
 *		TPM i2c address
 */
void al_tpm_if_i2c_handle_init(
	struct al_tpm_if		*tpm_if,
	struct al_tpm_if_i2c_priv	*tpm_if_priv,
	struct al_i2c_interface		*i2c_if,
	unsigned int			tpm_i2c_addr);

/**
 * TPM I2C generic interface handle initialization.
 * Used when TPM HAL service does not manage the i2c interface (e.g. u-boot)
 *
 * @param	tpm_if
 *		An uninitialized TPM interface handle
 * @param	tpm_if_i2c_gen
 *		An uninitialized I2C interface structure
 * @param	i2c_params
 *		I2C params and driver callbacks to be used by the TPM service
 */
void al_tpm_if_i2c_handle_init_gen(
	struct al_tpm_if		*tpm_if,
	struct al_tpm_if_i2c_gen	*tpm_if_i2c_gen,
	struct al_tpm_if_i2c_gen	*i2c_params);

#endif

