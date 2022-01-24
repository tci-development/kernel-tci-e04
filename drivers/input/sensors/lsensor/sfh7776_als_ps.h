/* include/linux/sfh7776_als_ps.h
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

#ifndef __LINUX_SFH7776_ALS_PS_H
#define __LINUX_SFH7776_ALS_PS_H
#include <linux/types.h>
#include <linux/ioctl.h>

#define SFH7776_IOCTL_MAGIC             'c'        
#define PROXIMITY_IOCTL_ENABLE          _IOW(SFH7776_IOCTL_MAGIC, 1, int*)
#define PROXIMITY_IOCTL_GET_ENABLED     _IOR(SFH7776_IOCTL_MAGIC, 2, int*)
#define LIGHTSENSOR_IOCTL_ENABLE        _IOW(SFH7776_IOCTL_MAGIC, 3, int*)
#define LIGHTSENSOR_IOCTL_GET_ENABLED   _IOR(SFH7776_IOCTL_MAGIC, 4, int*)
#endif
