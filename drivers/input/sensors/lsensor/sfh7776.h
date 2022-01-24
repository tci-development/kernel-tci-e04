/* include/linux/sfh7776.h
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef _LINUX_SFH7776_H_
#define	_LINUX_SFH7776_H_

#include <linux/types.h>
#include <linux/ioctl.h>
#define SFH7776_NAME "sfh7776"

struct sfh7776_platform_data {
	uint16_t	slave_address;
	int		intr;
	uint8_t		mode_control;
	uint8_t		als_ps_control;
	uint8_t		int_setting;
	uint8_t		ps_th_lsb;
	uint8_t		ps_th_msb;
	uint8_t		ps_tl_lsb;
	uint8_t		ps_tl_msb;
	uint8_t		als_vis_th_lsb;
	uint8_t		als_vis_th_msb;
	uint8_t         als_vis_tl_lsb;
	uint8_t         als_vis_tl_msb;
	uint16_t	adc_table[10];
};

#define CHIP_ID		0x9
#define REGISTER_NR	19 /*0x40->0x52*/
/*register map*/
#define SYSTEM_CONTROL	0x40
#define MODE_CONTROL	0x41
#define ALS_PS_CONTROL	0X42
#define ALS_PS_STATUS	0X43
#define PS_DATA_LSB	0x44
#define PS_DATA_MSB	0x45
#define ALS_VIS_DATA_LSB	0x46
#define ALS_VIS_DATA_MSB	0X47
#define ALS_IR_DATA_LSB		0X48
#define ALS_IR_DATA_MSB		0x49
#define INT_SETTING	0x4a
#define PS_TH_LSB	0x4b
#define PS_TH_MSB	0x4c
#define PS_TL_LSB	0x4d
#define PS_TL_MSB	0x4e
#define ALS_VIS_TH_LSB	0x4f
#define ALS_VIS_TH_MSB	0x50
#define ALS_VIS_TL_LSB	0x51
#define ALS_VIS_TL_MSB	0x52

#define IF_PS_INT	(1 << 7)
#define IF_ALS_INT	(1 << 6)
#define ENABLE_PS_INT	(1 << 0)
#define ENABLE_ALS_INT	(1 << 1)

#endif /*_LINUX_SFH7776_H_*/
