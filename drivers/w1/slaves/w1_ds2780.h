/*
 * 1-Wire implementation for the ds2780 chip
 * Copyright (C) 2009 Vladimir Ananiev <vovan888@gmail.com>
 *
 * based on ds2760 driver:
 * Copyright (C) 2004-2005, Szabolcs Gyurko <szabolcs.gyurko@tlt.hu>
 *
 * Use consistent with the GNU GPL is permitted,
 * provided that this copyright notice is
 * preserved in its entirety in all copies and derived works.
 *
 */

#ifndef __w1_ds2780_h__
#define __w1_ds2780_h__

/* Net address commands */
#define W1_DS2780_READ_NET_ADDRESS	0x33	
#define W1_DS2780_MATCH NET_ADDRESS	0x55
#define W1_DS2780_SKIP_NET_ADDRESS	0xCC
#define W1_DS2780_SEARCH_NET_ADDRESS	0xF0
#define W1_DS2780_RESUME		0xA5
/* Function commands */
#define W1_DS2780_READ_DATA		0x69
#define W1_DS2780_WRITE_DATA		0x6C
#define W1_DS2780_COPY_DATA		0x48
#define W1_DS2780_RECALL_DATA		0xB8
#define W1_DS2780_LOCK			0x6A

/* Number of valid register addresses */
#define DS2780_DATA_SIZE		0x80
/* Registers */
#define DS2780_STATUS			0x01	/* Status Register */
#define DS2780_RAAC_MSB			0x02	/* Remaining Active Absolute Capacity */
#define DS2780_RAAC_LSB			0x03
#define DS2780_RSAC_MSB			0x04	/* Remaining Standby Absolute Capacity */
#define DS2780_RSAC_LSB			0x05
#define DS2780_RARC			0x06	/* Remaining Active Relative Capacity */
#define DS2780_RSRC			0x07	/* Remaining Standby Relative Capacity */
#define DS2780_IAVG_MSB			0x08	/* Average Current Register */
#define DS2780_IAVG_LSB			0x09
#define DS2780_TEMP_MSB			0x0A	/* Temperature Register */
#define DS2780_TEMP_LSB			0x0B
#define DS2780_VOLT_MSB			0x0C	/* Voltage Register */
#define DS2780_VOLT_LSB			0x0D
#define DS2780_CURRENT_MSB		0x0E	/* Current Register */
#define DS2780_CURRENT_LSB		0x0F
#define DS2780_ACR_MSB			0x10	/* Accumulated Current Register */
#define DS2780_ACR_LSB			0x11
#define DS2780_ACRL_MSB			0x12	/* Low Accumulated Current Register */
#define DS2780_ACRL_LSB			0x13
#define DS2780_AS			0x14	/* Age Scalar */
#define DS2780_SFR			0x15	/* Special Feature Register */
#define DS2780_FULL_MSB			0x16	/* Full Capacity */
#define DS2780_FULL_LSB			0x17
#define DS2780_AE_MSB			0x18	/* Active Empty */
#define DS2780_AE_LSB			0x19
#define DS2780_SE_MSB			0x1A	/* Standby Empty */
#define DS2780_SE_LSB			0x1B

#define DS2780_EEPROM			0x1F	/* EEPROM Register */

#define DS2780_USER_EEPROM_START	0x20	/* User EEPROM, Lockable, Block 0 */
#define DS2780_USER_EEPROM_SIZE		0x10
#define DS2780_PARAM_EEPROM_START	0x60	/* Parameter EEPROM, Lockable, Block 1 */
#define DS2780_PARAM_EEPROM_SIZE	0x20

/* Parameter EEPROM block 1 registers */
#define DS2780_PARAM_CONTROL		0x60	/* Control Register */
#define DS2780_PARAM_AB			0x61	/* Accumulation Bias */
#define DS2780_PARAM_AC_MSB		0x62	/* Aging Capacity MSB */
#define DS2780_PARAM_AC_LSB		0x63	/* Aging Capacity LSB */
#define DS2780_PARAM_VCHG		0x64	/* Charge Voltage */
#define DS2780_PARAM_IMIN		0x65	/* Minimum Charge Current */
#define DS2780_PARAM_VAE		0x66	/* Active Empty Voltage */
#define DS2780_PARAM_IAE		0x67	/* Active Empty Current */
#define DS2780_PARAM_RSNSP		0x69	/* Sense Resistor Prime */
#define DS2780_PARAM_RSGAIN_MSB		0x78	/* Sense Resistor Gain MSB */
#define DS2780_PARAM_RSGAIN_LSB		0x79	/* Sense Resistor Gain LSB */
#define DS2780_PARAM_RSTC		0x7A	/* Sense Resistor Temp. Coeff */
#define DS2780_PARAM_FRSGAIN_MSB	0x7B	/* Factory Gain MSB */
#define DS2780_PARAM_FRSGAIN_LSB	0x7C	/* Factory Gain LSB */

/* Status register bits */
#define DS2780_STATUS_CHGTF		0x80	/* Charge Termination Flag */
#define DS2780_STATUS_AEF		0x40	/* Active Empty Flag */
#define DS2780_STATUS_SEF		0x20	/* Standby Empty Flag */
#define DS2780_STATUS_LEARNF		0x10	/* Learn Flag */
#define DS2780_STATUS_RESERVED3		0x08
#define DS2780_STATUS_UVF		0x04	/* Under-Voltage Flag */
#define DS2780_STATUS_PORF		0x02	/* Power-On Reset Flag */
#define DS2780_STATUS_RESERVED0		0x01

extern int w1_ds2780_read(struct device *dev, char *buf, int addr,
			  size_t count);
extern int w1_ds2780_write(struct device *dev, char *buf, int addr,
			   size_t count);

#endif /* !__w1_ds2780_h__ */
