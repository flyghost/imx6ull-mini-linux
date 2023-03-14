/*
 * i2c-boardinfo.c - collect pre-declarations of I2C devices
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/export.h>
#include <linux/rwsem.h>

#include "i2c-core.h"


/* These symbols are exported ONLY FOR the i2c core.
 * No other users will be supported.
 */
DECLARE_RWSEM(__i2c_board_lock);
EXPORT_SYMBOL_GPL(__i2c_board_lock);

LIST_HEAD(__i2c_board_list);
EXPORT_SYMBOL_GPL(__i2c_board_list);

// __i2c_first_dynamic_bus_num是一个全局变量，用来表示第一个动态分配的I2C总线编号。它的初始值是0，
// 但是如果在machine_init()函数中调用了i2c_register_board_info()函数，它会被更新为busnum + 1
// 其中busnum是传递给i2c_register_board_info()函数的参数。这样可以避免和静态分配的I2C总线编号冲突。
// 例如：已经初始化了一个i2c0，下一个调用分配的最大总线标号就是1
// 动态创建的I2C总线通常是由某些设备驱动程序或用户空间程序创建的，而静态创建的I2C总线通常是由平台代码或设备树定义的。
int __i2c_first_dynamic_bus_num;
EXPORT_SYMBOL_GPL(__i2c_first_dynamic_bus_num);


/**
 * i2c_register_board_info - statically declare I2C devices
 * @busnum: identifies the bus to which these devices belong
 * @info: vector of i2c device descriptors
 * @len: how many descriptors in the vector; may be zero to reserve
 *	the specified bus number.
 *
 * Systems using the Linux I2C driver stack can declare tables of board info
 * while they initialize.  This should be done in board-specific init code
 * near arch_initcall() time, or equivalent, before any I2C adapter driver is
 * registered.  For example, mainboard init code could define several devices,
 * as could the init code for each daughtercard in a board stack.
 *
 * The I2C devices will be created later, after the adapter for the relevant
 * bus has been registered.  After that moment, standard driver model tools
 * are used to bind "new style" I2C drivers to the devices.  The bus number
 * for any device declared using this routine is not available for dynamic
 * allocation.
 *
 * The board info passed can safely be __initdata, but be careful of embedded
 * pointers (for platform_data, functions, etc) since that won't be copied.
 */
int __init
i2c_register_board_info(int busnum,
	struct i2c_board_info const *info, unsigned len)
{
	int status;

	down_write(&__i2c_board_lock);

	/* dynamic bus numbers will be assigned after the last static one */
	if (busnum >= __i2c_first_dynamic_bus_num)
		__i2c_first_dynamic_bus_num = busnum + 1;

	for (status = 0; len; len--, info++) {
		struct i2c_devinfo	*devinfo;

		devinfo = kzalloc(sizeof(*devinfo), GFP_KERNEL);
		if (!devinfo) {
			pr_debug("i2c-core: can't register boardinfo!\n");
			status = -ENOMEM;
			break;
		}

		devinfo->busnum = busnum;
		devinfo->board_info = *info;
		list_add_tail(&devinfo->list, &__i2c_board_list);
	}

	up_write(&__i2c_board_lock);

	return status;
}
