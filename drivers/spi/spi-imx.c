/*
 * Copyright 2004-2007, 2016 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2008 Juergen Beisert
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation
 * 51 Franklin Street, Fifth Floor
 * Boston, MA  02110-1301, USA.
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <linux/platform_data/dma-imx.h>
#include <linux/platform_data/spi-imx.h>

#define DRIVER_NAME "spi_imx"

#define MXC_CSPIRXDATA		0x00
#define MXC_CSPITXDATA		0x04
#define MXC_CSPICTRL		0x08
#define MXC_CSPIINT		0x0c
#define MXC_RESET		0x1c

/* generic defines to abstract from the different register layouts */
#define MXC_INT_RR	(1 << 0) /* Receive data ready interrupt */
#define MXC_INT_TE	(1 << 1) /* Transmit FIFO empty interrupt */
#define MXC_INT_TCEN    (1 << 7)   /* Transfer complete */

/* The maximum  bytes that a sdma BD can transfer.*/
#define MAX_SDMA_BD_BYTES  (1 << 15)
/* 3 Sec for 1MB or less than 1MB, else change with the transfer length */
#define IMX_DEFAULT_DMA_TIMEOUT (msecs_to_jiffies(3000))
#define IMX_DMA_TIMEOUT(len) ((len < 0x100000) ? IMX_DEFAULT_DMA_TIMEOUT : \
			     len * DIV_ROUND_UP(IMX_DEFAULT_DMA_TIMEOUT, \
			     0x100000))
struct spi_imx_config {
	unsigned int speed_hz;  // 配置速度（以 Hz 为单位）
	unsigned int bpw;       // 每字位数
	unsigned int mode;      // 模式
	u8 cs;                  // 片选
};

enum spi_imx_devtype {
	IMX1_CSPI,
	IMX21_CSPI,
	IMX27_CSPI,
	IMX31_CSPI,
	IMX35_CSPI,	/* CSPI on all i.mx except above */
	IMX51_ECSPI,	/* ECSPI on i.mx51 and later */
	IMX6UL_ECSPI,
};

struct spi_imx_data;

struct spi_imx_devtype_data {
	void (*intctrl)(struct spi_imx_data *, int);			// 控制SPI中断的使能或者禁止
	int (*config)(struct spi_imx_data *, struct spi_imx_config *);	// 配置SPI时钟、数据位、极性等参数
	void (*trigger)(struct spi_imx_data *);				// 触发SPI传输
	int (*rx_available)(struct spi_imx_data *);			// 判断SPI的接收缓冲区是否有可用的数据
	void (*reset)(struct spi_imx_data *);				// 复位SPI的寄存器和状态
	enum spi_imx_devtype devtype;					// 不同的SPI设备类型
};

struct spi_imx_data {
	struct spi_bitbang bitbang;         // 用于执行位操作

	struct completion xfer_done;        // 用于指示传输何时完成
	void __iomem *base;                 // 寄存器基地址
	struct clk *clk_per;                // 外设时钟
	struct clk *clk_ipg;                // IPG 时钟
	unsigned long spi_clk;              // 存储 SPI 时钟速率

	unsigned int count;                 // 存储传输的字节数
	void (*tx)(struct spi_imx_data *);  // 执行发送操作
	void (*rx)(struct spi_imx_data *);  // 执行接收操作
	void *rx_buf;                       // 接收缓冲区
	const void *tx_buf;                 // 发送缓冲区
	unsigned int txfifo; /* number of words pushed in tx FIFO */    // 存储已经发送到fifo中的字节数

	/* DMA */
	unsigned int dma_is_inited;         // 指示 DMA 是否已初始化
	unsigned int dma_finished;          // 指示 DMA 传输是否已完成
	bool usedma;                        // 指示是否使用 DMA 进行传输
	u32 rx_wml;                         // 存储接收水位线
	u32 tx_wml;                         // 存储发送水位线
	u32 rxt_wml;                        // 存储接收/发送触发器水位线
	struct completion dma_rx_completion;    // 指示 DMA 接收操作何时完成
	struct completion dma_tx_completion;    // 指示 DMA 发送操作何时完成
	struct dma_slave_config rx_config;      // 用于配置 DMA 接收操作
	struct dma_slave_config tx_config;      // 用于配置 DMA 发送操作

	const struct spi_imx_devtype_data *devtype_data;    // 用于存储设备类型相关的数据
	int chipselect[0];                                  // 存储芯片选择信息
};

static inline int is_imx27_cspi(struct spi_imx_data *d)
{
	return d->devtype_data->devtype == IMX27_CSPI;
}

static inline int is_imx35_cspi(struct spi_imx_data *d)
{
	return d->devtype_data->devtype == IMX35_CSPI;
}

/**
 * @brief 根据不同的设备类型，返回FIFO大小
 * 
 * @param d 
 * @return unsigned 
 */
static inline unsigned spi_imx_get_fifosize(struct spi_imx_data *d)
{
	return (d->devtype_data->devtype == IMX51_ECSPI
		|| d->devtype_data->devtype == IMX6UL_ECSPI) ? 64 : 8;
}

/**
 * @brief 生成一个函数定义，用来从SPI的接收寄存器读取数据，并存储到SPI的接收缓冲区中
 * 
 * 从 spi_imx->base + MXC_CSPIRXDATA 这个地址读取一个无符号整数，赋值给 val 变量。
 * 这个地址是 SPI 的接收寄存器的地址，其中存储了最近一次 SPI 传输的接收数据。
 * 
 * 如果 spi_imx->rx_buf 不为空，表示有有效的接收缓冲区，那么就把 val 变量转换成指定的类型，并赋值给 spi_imx->rx_buf 指向的内存位置。
 * 然后把 spi_imx->rx_buf 指针向后移动相应的字节数，以便下次写入数据
 * 
 */
#define MXC_SPI_BUF_RX(type)						\
static void spi_imx_buf_rx_##type(struct spi_imx_data *spi_imx)		\
{									\
	unsigned int val = readl(spi_imx->base + MXC_CSPIRXDATA);	\
									\
	if (spi_imx->rx_buf) {						\
		*(type *)spi_imx->rx_buf = val;				\
		spi_imx->rx_buf += sizeof(type);			\
	}								\
}

/**
 * @brief 生成一个函数定义，用来从 SPI 的发送缓冲区读取数据，并写入到 SPI 的发送寄存器中
 * 
 * 如果 spi_imx->tx_buf 不为空，表示有有效的发送缓冲区，那么就把 spi_imx->tx_buf 指向的内存位置转换成指定的类型，并赋值给 val 变量。
 * 然后把 spi_imx->tx_buf 指针向后移动相应的字节数，以便下次读取数据
 * 
 * 把 spi_imx->count 变量减去相应的字节数，表示已经发送了一部分数据
 * 
 * 把 val 变量写入到 spi_imx->base + MXC_CSPITXDATA 这个地址中。这个地址是 SPI 的发送寄存器的地址，其中存储了要进行下一次 SPI 传输的发送数据
 * 
 */
#define MXC_SPI_BUF_TX(type)						\
static void spi_imx_buf_tx_##type(struct spi_imx_data *spi_imx)		\
{									\
	type val = 0;							\
									\
	if (spi_imx->tx_buf) {						\
		val = *(type *)spi_imx->tx_buf;				\
		spi_imx->tx_buf += sizeof(type);			\
	}								\
									\
	spi_imx->count -= sizeof(type);					\
									\
	writel(val, spi_imx->base + MXC_CSPITXDATA);			\
}

MXC_SPI_BUF_RX(u8)
MXC_SPI_BUF_TX(u8)
MXC_SPI_BUF_RX(u16)
MXC_SPI_BUF_TX(u16)
MXC_SPI_BUF_RX(u32)
MXC_SPI_BUF_TX(u32)

/* First entry is reserved, second entry is valid only if SDHC_SPIEN is set
 * (which is currently not the case in this driver)
 */
// SPI 的时钟分频系数，用来控制 SPI 的时钟速度
static int mxc_clkdivs[] = {0, 3, 4, 6, 8, 12, 16, 24, 32, 48, 64, 96, 128, 192,
	256, 384, 512, 768, 1024};

/* MX21, MX27 */
/**
 * @brief 根据 SPI 的目标时钟速度 fspi 和主机的时钟速度 fin，从 mxc_clkdivs 数组中选择一个合适的分频系数，并返回其下标
 * 
 * @param fin 			主机的时钟速度
 * @param fspi 			目标时钟速度
 * @param max			mxc_clkdivs 数组的最大长度，用来限制搜索范围
 * @return unsigned int 	mxc_clkdivs的下标
 */
static unsigned int spi_imx_clkdiv_1(unsigned int fin,
		unsigned int fspi, unsigned int max)
{
	int i;

	/**
	 * @brief 从 i 等于 2 开始，循环遍历 mxc_clkdivs 数组中的元素，直到 i 等于 max 或者找到一个满足条件的元素为止。
	 * 
	 * 条件是 fspi 乘以 mxc_clkdivs[i] 大于等于 fin，表示该分频系数可以使 SPI 的时钟速度不超过目标时钟速度
	 * 
	 */
	for (i = 2; i < max; i++)
		if (fspi * mxc_clkdivs[i] >= fin)
			return i;

	return max;
}

/* MX1, MX31, MX35, MX51 CSPI */
/**
 * @brief 根据 SPI 的目标时钟速度 fspi 和主机的时钟速度 fin，从 4 到 512 之间的一些数值中选择一个合适的分频系数，并返回其对应的位移量
 * 
 * @param fin 			主机的时钟速度
 * @param fspi 			目标时钟速度
 * @return unsigned int 
 */
static unsigned int spi_imx_clkdiv_2(unsigned int fin,
		unsigned int fspi)
{
	int i, div = 4;

	for (i = 0; i < 7; i++) {
		if (fspi * div >= fin)
			return i;
		div <<= 1;
	}

	return 7;
}

/**
 * @brief 判断是否可以使用 DMA（直接内存访问）来进行 SPI 的传输
 * 
 * @param master 	SPI 的主控制器
 * @param spi 		SPI 的设备
 * @param transfer 	SPI 的传输
 * @return true 
 * @return false 
 */
static bool spi_imx_can_dma(struct spi_master *master, struct spi_device *spi,
			 struct spi_transfer *transfer)
{
	// 获取 SPI 的驱动数据结构
	struct spi_imx_data *spi_imx = spi_master_get_devdata(master);

	// 如果 transfer 指针不为空，表示有有效的传输，
	// 并且 spi_imx->dma_is_inited 为真，表示 DMA 已经初始化，
	// 并且 transfer->len 大于 spi_imx_get_fifosize(spi_imx) 的返回值，表示传输的长度大于 SPI 的 FIFO 大小，
	// 那么返回真值，表示可以使用 DMA
	if (transfer && spi_imx->dma_is_inited &&
		(transfer->len > spi_imx_get_fifosize(spi_imx)))
		return true;
	return false;
}

#define MX51_ECSPI_CTRL		0x08				// ECSPI 控制寄存器的地址
#define MX51_ECSPI_CTRL_ENABLE		(1 <<  0)		// ECSPI 控制寄存器的使能位。当该位为 1 时，表示启动 ECSPI 控制器
#define MX51_ECSPI_CTRL_XCH		(1 <<  2)		// ECSPI 控制寄存器的交换位。当该位为 1 时，表示开始一次 SPI 的传输
#define MX51_ECSPI_CTRL_SMC		(1 << 3)		// ECSPI 控制寄存器的单主控模式位。当该位为 1 时，表示只有一个主控设备连接到 ECSPI 总线上
#define MX51_ECSPI_CTRL_MODE_MASK	(0xf << 4)		// ECSPI 控制寄存器的模式掩码。这些位用来设置 SPI 的工作模式，包括时钟极性、时钟相位和数据有效边沿等
#define MX51_ECSPI_CTRL_POSTDIV_OFFSET	8			// ECSPI 控制寄存器的后分频系数偏移量，为第 8 到第 11 位。这些位用来设置 SPI 的后分频系数，用来进一步降低 SPI 的时钟速度
#define MX51_ECSPI_CTRL_PREDIV_OFFSET	12			// ECSPI 控制寄存器的预分频系数偏移量，为第 12 到第 15 位。这些位用来设置 SPI 的预分频系数，用来降低 SPI 的时钟速度
#define MX51_ECSPI_CTRL_CS(cs)		((cs) << 18)		// 表示 SPI 的从设备编号。这个宏的功能是把 cs 的值左移 18 位，得到 ECSPI 控制寄存器的片选位域。
								// 这些位用来选择 SPI 的从设备，最多可以支持四个从设备
#define MX51_ECSPI_CTRL_BL_OFFSET	20			// ECSPI 控制寄存器的位长度偏移量，为第 20 到第 23 位。
								// 这些位用来设置 SPI 的每个字节的位长度，即每次传输的数据位数。最小的位长度为 1，最大的位长度为 32

/**
 * @brief 设置ECSPI控制器的CONFIG寄存器，这个寄存器有32位，每4位对应一个从机设备。cs是一个参数，表示从机设备的编号，从0到3。
 * 
 */
#define MX51_ECSPI_CONFIG	0x0c				// ECSPI 配置寄存器的地址
#define MX51_ECSPI_CONFIG_SCLKPHA(cs)	(1 << ((cs) +  0))	// ECSPI 配置寄存器的时钟相位位，占用第 0 到第 3 位。
								// 这些位用来设置 SPI 的时钟相位，即数据采样发生在时钟的哪个边沿。每个从设备可以有不同的时钟相位设置
#define MX51_ECSPI_CONFIG_SCLKPOL(cs)	(1 << ((cs) +  4))	// ECSPI 配置寄存器的时钟极性位，占用第 4 到第 7 位。
								// 这些位用来设置 SPI 的时钟极性，即空闲状态下时钟信号是高电平还是低电平。每个从设备可以有不同的时钟极性设置
#define MX51_ECSPI_CONFIG_SBBCTRL(cs)	(1 << ((cs) +  8))	// 是否启用SPI时钟的突发传输模式
#define MX51_ECSPI_CONFIG_SSBPOL(cs)	(1 << ((cs) + 12))	// SPI片选信号的极性
#define MX51_ECSPI_CONFIG_SCLKCTL(cs)	(1 << ((cs) + 20))	// SPI时钟信号的极性和相位

/**
 * @brief 用来控制中断的使能和状态
 * 
 */
#define MX51_ECSPI_INT		0x10				// INT寄存器地址
#define MX51_ECSPI_INT_TEEN		(1 <<  0)		// 发送缓冲区空中断使能
#define MX51_ECSPI_INT_RREN		(1 <<  3)		// 接收缓冲区满中断使能
#define MX51_ECSPI_INT_TCEN             (1 << 7)		// 传输完成中断使能

/**
 * @brief ECSPI控制器的DMA寄存器
 * 
 */
#define MX51_ECSPI_DMA      0x14				// DMA寄存器地址
#define MX51_ECSPI_DMA_TX_WML_OFFSET	0			// 发送缓冲区水位线的偏移量
#define MX51_ECSPI_DMA_TX_WML_MASK	0x3F
#define MX51_ECSPI_DMA_RX_WML_OFFSET	16			// 接收缓冲区水位线的偏移量
#define MX51_ECSPI_DMA_RX_WML_MASK	(0x3F << 16)
#define MX51_ECSPI_DMA_RXT_WML_OFFSET	24			// 接收阈值水位线的偏移量
#define MX51_ECSPI_DMA_RXT_WML_MASK	(0x3F << 24)

#define MX51_ECSPI_DMA_TEDEN_OFFSET	7			// 发送缓冲区空DMA使能
#define MX51_ECSPI_DMA_RXDEN_OFFSET	23			// 接收缓冲区满DMA使能
#define MX51_ECSPI_DMA_RXTDEN_OFFSET	31			// 接收阈值DMA使能

/**
 * @brief STAT寄存器
*/
#define MX51_ECSPI_STAT		0x18				// STAT寄存器的地址
#define MX51_ECSPI_STAT_RR		(1 <<  3)		// 表示接收缓冲区满中断状态，如果为1，表示接收缓冲区已满；如果为0，表示接收缓冲区未满

/* MX51 eCSPI */
/**
 * @brief 计算ECSPI控制器的时钟分频系数
 * 
 * @param fin 		输入时钟频率
 * @param fspi 		SPI时钟频率
 * @param fres 		用来返回实际的SPI时钟频率
 * @return unsigned int 时钟分频系数
 */
static unsigned int mx51_ecspi_clkdiv(unsigned int fin, unsigned int fspi,
				      unsigned int *fres)
{
	/*
	 * there are two 4-bit dividers, the pre-divider divides by
	 * $pre, the post-divider by 2^$post
	 */
	unsigned int pre, post;

	// 判断fspi是否大于fin，如果是，就返回0，表示无法设置
	if (unlikely(fspi > fin))
		return 0;

	// 计算post值，它表示后分频器的指数。后分频器是用来将输入时钟除以2^post得到中间时钟。post值要尽量小，使得中间时钟不小于fspi
	post = fls(fin) - fls(fspi);
	if (fin > fspi << post)
		post++;

	/* now we have: (fin <= fspi << post) with post being minimal */

	post = max(4U, post) - 4;
	if (unlikely(post > 0xf)) {
		pr_err("%s: cannot set clock freq: %u (base freq: %u)\n",
				__func__, fspi, fin);
		return 0xff;
	}

	pre = DIV_ROUND_UP(fin, fspi << post) - 1;

	pr_debug("%s: fin: %u, fspi: %u, post: %u, pre: %u\n",
			__func__, fin, fspi, post, pre);

	/* Resulting frequency for the SCLK line. */
	*fres = (fin / (pre + 1)) >> post;

	return (pre << MX51_ECSPI_CTRL_PREDIV_OFFSET) |
		(post << MX51_ECSPI_CTRL_POSTDIV_OFFSET);
}

static void __maybe_unused mx51_ecspi_intctrl(struct spi_imx_data *spi_imx, int enable)
{
	unsigned val = 0;

	if (enable & MXC_INT_TE)
		val |= MX51_ECSPI_INT_TEEN;

	if (enable & MXC_INT_RR)
		val |= MX51_ECSPI_INT_RREN;

	if (enable & MXC_INT_TCEN)
		val |= MX51_ECSPI_INT_TCEN;

	writel(val, spi_imx->base + MX51_ECSPI_INT);
}

/**
 * @brief 触发SPI传输
 * 
 * @param spi_imx 
 */
static void __maybe_unused mx51_ecspi_trigger(struct spi_imx_data *spi_imx)
{
	u32 reg = readl(spi_imx->base + MX51_ECSPI_CTRL);
	/*
	 * To workaround ERR008517, SDMA script need use XCH instead of SMC
	 * just like PIO mode and it fix on i.mx6ul
	 */
	if (!spi_imx->usedma)
		reg |= MX51_ECSPI_CTRL_XCH;
	else if (!spi_imx->dma_finished &&
		 spi_imx->devtype_data->devtype == IMX6UL_ECSPI)
		reg |= MX51_ECSPI_CTRL_SMC;
	else
		reg &= ~MX51_ECSPI_CTRL_SMC;
	writel(reg, spi_imx->base + MX51_ECSPI_CTRL);
}

static int __maybe_unused mx51_ecspi_config(struct spi_imx_data *spi_imx,
		struct spi_imx_config *config)
{
	u32 ctrl = MX51_ECSPI_CTRL_ENABLE, cfg = 0, dma = 0;
	u32 clk = config->speed_hz, delay;

	/*
	 * The hardware seems to have a race condition when changing modes. The
	 * current assumption is that the selection of the channel arrives
	 * earlier in the hardware than the mode bits when they are written at
	 * the same time.
	 * So set master mode for all channels as we do not support slave mode.
	 */
	ctrl |= MX51_ECSPI_CTRL_MODE_MASK;

	/* set clock speed */
	ctrl |= mx51_ecspi_clkdiv(spi_imx->spi_clk, config->speed_hz, &clk);

	/* set chip select to use */
	ctrl |= MX51_ECSPI_CTRL_CS(config->cs);

	ctrl |= (config->bpw - 1) << MX51_ECSPI_CTRL_BL_OFFSET;

	cfg |= MX51_ECSPI_CONFIG_SBBCTRL(config->cs);

	if (config->mode & SPI_CPHA)
		cfg |= MX51_ECSPI_CONFIG_SCLKPHA(config->cs);

	if (config->mode & SPI_CPOL) {
		cfg |= MX51_ECSPI_CONFIG_SCLKPOL(config->cs);
		cfg |= MX51_ECSPI_CONFIG_SCLKCTL(config->cs);
	}
	if (config->mode & SPI_CS_HIGH)
		cfg |= MX51_ECSPI_CONFIG_SSBPOL(config->cs);

	writel(ctrl, spi_imx->base + MX51_ECSPI_CTRL);
	writel(cfg, spi_imx->base + MX51_ECSPI_CONFIG);

	/*
	 * Wait until the changes in the configuration register CONFIGREG
	 * propagate into the hardware. It takes exactly one tick of the
	 * SCLK clock, but we will wait two SCLK clock just to be sure. The
	 * effect of the delay it takes for the hardware to apply changes
	 * is noticable if the SCLK clock run very slow. In such a case, if
	 * the polarity of SCLK should be inverted, the GPIO ChipSelect might
	 * be asserted before the SCLK polarity changes, which would disrupt
	 * the SPI communication as the device on the other end would consider
	 * the change of SCLK polarity as a clock tick already.
	 */
	delay = (2 * 1000000) / clk;
	if (likely(delay < 10))	/* SCLK is faster than 100 kHz */
		udelay(delay);
	else			/* SCLK is _very_ slow */
		usleep_range(delay, delay + 10);

	/*
	 * Configure the DMA register: setup the watermark
	 * and enable DMA request.
	 */
	if (spi_imx->dma_is_inited) {
		if (spi_imx->devtype_data->devtype != IMX6UL_ECSPI)
			spi_imx->tx_wml = 1;
		dma = (spi_imx->rx_wml - 1) << MX51_ECSPI_DMA_RX_WML_OFFSET
		      | (spi_imx->tx_wml - 1) << MX51_ECSPI_DMA_TX_WML_OFFSET
		      | (1 << MX51_ECSPI_DMA_TEDEN_OFFSET)
		      | (1 << MX51_ECSPI_DMA_RXDEN_OFFSET);
		writel(dma, spi_imx->base + MX51_ECSPI_DMA);
	}

	return 0;
}

static int __maybe_unused mx51_ecspi_rx_available(struct spi_imx_data *spi_imx)
{
	return readl(spi_imx->base + MX51_ECSPI_STAT) & MX51_ECSPI_STAT_RR;
}

static void __maybe_unused mx51_ecspi_reset(struct spi_imx_data *spi_imx)
{
	/* drain receive buffer */
	while (mx51_ecspi_rx_available(spi_imx))
		readl(spi_imx->base + MXC_CSPIRXDATA);
}

#define MX31_INTREG_TEEN	(1 << 0)
#define MX31_INTREG_RREN	(1 << 3)

#define MX31_CSPICTRL_ENABLE	(1 << 0)
#define MX31_CSPICTRL_MASTER	(1 << 1)
#define MX31_CSPICTRL_XCH	(1 << 2)
#define MX31_CSPICTRL_POL	(1 << 4)
#define MX31_CSPICTRL_PHA	(1 << 5)
#define MX31_CSPICTRL_SSCTL	(1 << 6)
#define MX31_CSPICTRL_SSPOL	(1 << 7)
#define MX31_CSPICTRL_BC_SHIFT	8
#define MX35_CSPICTRL_BL_SHIFT	20
#define MX31_CSPICTRL_CS_SHIFT	24
#define MX35_CSPICTRL_CS_SHIFT	12
#define MX31_CSPICTRL_DR_SHIFT	16

#define MX31_CSPISTATUS		0x14
#define MX31_STATUS_RR		(1 << 3)

/* These functions also work for the i.MX35, but be aware that
 * the i.MX35 has a slightly different register layout for bits
 * we do not use here.
 */
static void __maybe_unused mx31_intctrl(struct spi_imx_data *spi_imx, int enable)
{
	unsigned int val = 0;

	if (enable & MXC_INT_TE)
		val |= MX31_INTREG_TEEN;
	if (enable & MXC_INT_RR)
		val |= MX31_INTREG_RREN;

	writel(val, spi_imx->base + MXC_CSPIINT);
}

static void __maybe_unused mx31_trigger(struct spi_imx_data *spi_imx)
{
	unsigned int reg;

	reg = readl(spi_imx->base + MXC_CSPICTRL);
	reg |= MX31_CSPICTRL_XCH;
	writel(reg, spi_imx->base + MXC_CSPICTRL);
}

static int __maybe_unused mx31_config(struct spi_imx_data *spi_imx,
		struct spi_imx_config *config)
{
	unsigned int reg = MX31_CSPICTRL_ENABLE | MX31_CSPICTRL_MASTER;
	int cs = spi_imx->chipselect[config->cs];

	reg |= spi_imx_clkdiv_2(spi_imx->spi_clk, config->speed_hz) <<
		MX31_CSPICTRL_DR_SHIFT;

	if (is_imx35_cspi(spi_imx)) {
		reg |= (config->bpw - 1) << MX35_CSPICTRL_BL_SHIFT;
		reg |= MX31_CSPICTRL_SSCTL;
	} else {
		reg |= (config->bpw - 1) << MX31_CSPICTRL_BC_SHIFT;
	}

	if (config->mode & SPI_CPHA)
		reg |= MX31_CSPICTRL_PHA;
	if (config->mode & SPI_CPOL)
		reg |= MX31_CSPICTRL_POL;
	if (config->mode & SPI_CS_HIGH)
		reg |= MX31_CSPICTRL_SSPOL;
	if (cs < 0)
		reg |= (cs + 32) <<
			(is_imx35_cspi(spi_imx) ? MX35_CSPICTRL_CS_SHIFT :
						  MX31_CSPICTRL_CS_SHIFT);

	writel(reg, spi_imx->base + MXC_CSPICTRL);

	return 0;
}

static int __maybe_unused mx31_rx_available(struct spi_imx_data *spi_imx)
{
	return readl(spi_imx->base + MX31_CSPISTATUS) & MX31_STATUS_RR;
}

static void __maybe_unused mx31_reset(struct spi_imx_data *spi_imx)
{
	/* drain receive buffer */
	while (readl(spi_imx->base + MX31_CSPISTATUS) & MX31_STATUS_RR)
		readl(spi_imx->base + MXC_CSPIRXDATA);
}

#define MX21_INTREG_RR		(1 << 4)
#define MX21_INTREG_TEEN	(1 << 9)
#define MX21_INTREG_RREN	(1 << 13)

#define MX21_CSPICTRL_POL	(1 << 5)
#define MX21_CSPICTRL_PHA	(1 << 6)
#define MX21_CSPICTRL_SSPOL	(1 << 8)
#define MX21_CSPICTRL_XCH	(1 << 9)
#define MX21_CSPICTRL_ENABLE	(1 << 10)
#define MX21_CSPICTRL_MASTER	(1 << 11)
#define MX21_CSPICTRL_DR_SHIFT	14
#define MX21_CSPICTRL_CS_SHIFT	19

static void __maybe_unused mx21_intctrl(struct spi_imx_data *spi_imx, int enable)
{
	unsigned int val = 0;

	if (enable & MXC_INT_TE)
		val |= MX21_INTREG_TEEN;
	if (enable & MXC_INT_RR)
		val |= MX21_INTREG_RREN;

	writel(val, spi_imx->base + MXC_CSPIINT);
}

static void __maybe_unused mx21_trigger(struct spi_imx_data *spi_imx)
{
	unsigned int reg;

	reg = readl(spi_imx->base + MXC_CSPICTRL);
	reg |= MX21_CSPICTRL_XCH;
	writel(reg, spi_imx->base + MXC_CSPICTRL);
}

static int __maybe_unused mx21_config(struct spi_imx_data *spi_imx,
		struct spi_imx_config *config)
{
	unsigned int reg = MX21_CSPICTRL_ENABLE | MX21_CSPICTRL_MASTER;
	int cs = spi_imx->chipselect[config->cs];
	unsigned int max = is_imx27_cspi(spi_imx) ? 16 : 18;

	reg |= spi_imx_clkdiv_1(spi_imx->spi_clk, config->speed_hz, max) <<
		MX21_CSPICTRL_DR_SHIFT;
	reg |= config->bpw - 1;

	if (config->mode & SPI_CPHA)
		reg |= MX21_CSPICTRL_PHA;
	if (config->mode & SPI_CPOL)
		reg |= MX21_CSPICTRL_POL;
	if (config->mode & SPI_CS_HIGH)
		reg |= MX21_CSPICTRL_SSPOL;
	if (cs < 0)
		reg |= (cs + 32) << MX21_CSPICTRL_CS_SHIFT;

	writel(reg, spi_imx->base + MXC_CSPICTRL);

	return 0;
}

static int __maybe_unused mx21_rx_available(struct spi_imx_data *spi_imx)
{
	return readl(spi_imx->base + MXC_CSPIINT) & MX21_INTREG_RR;
}

static void __maybe_unused mx21_reset(struct spi_imx_data *spi_imx)
{
	writel(1, spi_imx->base + MXC_RESET);
}

#define MX1_INTREG_RR		(1 << 3)
#define MX1_INTREG_TEEN		(1 << 8)
#define MX1_INTREG_RREN		(1 << 11)

#define MX1_CSPICTRL_POL	(1 << 4)
#define MX1_CSPICTRL_PHA	(1 << 5)
#define MX1_CSPICTRL_XCH	(1 << 8)
#define MX1_CSPICTRL_ENABLE	(1 << 9)
#define MX1_CSPICTRL_MASTER	(1 << 10)
#define MX1_CSPICTRL_DR_SHIFT	13

static void __maybe_unused mx1_intctrl(struct spi_imx_data *spi_imx, int enable)
{
	unsigned int val = 0;

	if (enable & MXC_INT_TE)
		val |= MX1_INTREG_TEEN;
	if (enable & MXC_INT_RR)
		val |= MX1_INTREG_RREN;

	writel(val, spi_imx->base + MXC_CSPIINT);
}

static void __maybe_unused mx1_trigger(struct spi_imx_data *spi_imx)
{
	unsigned int reg;

	reg = readl(spi_imx->base + MXC_CSPICTRL);
	reg |= MX1_CSPICTRL_XCH;
	writel(reg, spi_imx->base + MXC_CSPICTRL);
}

static int __maybe_unused mx1_config(struct spi_imx_data *spi_imx,
		struct spi_imx_config *config)
{
	unsigned int reg = MX1_CSPICTRL_ENABLE | MX1_CSPICTRL_MASTER;

	reg |= spi_imx_clkdiv_2(spi_imx->spi_clk, config->speed_hz) <<
		MX1_CSPICTRL_DR_SHIFT;
	reg |= config->bpw - 1;

	if (config->mode & SPI_CPHA)
		reg |= MX1_CSPICTRL_PHA;
	if (config->mode & SPI_CPOL)
		reg |= MX1_CSPICTRL_POL;

	writel(reg, spi_imx->base + MXC_CSPICTRL);

	return 0;
}

static int __maybe_unused mx1_rx_available(struct spi_imx_data *spi_imx)
{
	return readl(spi_imx->base + MXC_CSPIINT) & MX1_INTREG_RR;
}

static void __maybe_unused mx1_reset(struct spi_imx_data *spi_imx)
{
	writel(1, spi_imx->base + MXC_RESET);
}

static struct spi_imx_devtype_data imx1_cspi_devtype_data = {
	.intctrl = mx1_intctrl,
	.config = mx1_config,
	.trigger = mx1_trigger,
	.rx_available = mx1_rx_available,
	.reset = mx1_reset,
	.devtype = IMX1_CSPI,
};

static struct spi_imx_devtype_data imx21_cspi_devtype_data = {
	.intctrl = mx21_intctrl,
	.config = mx21_config,
	.trigger = mx21_trigger,
	.rx_available = mx21_rx_available,
	.reset = mx21_reset,
	.devtype = IMX21_CSPI,
};

static struct spi_imx_devtype_data imx27_cspi_devtype_data = {
	/* i.mx27 cspi shares the functions with i.mx21 one */
	.intctrl = mx21_intctrl,
	.config = mx21_config,
	.trigger = mx21_trigger,
	.rx_available = mx21_rx_available,
	.reset = mx21_reset,
	.devtype = IMX27_CSPI,
};

static struct spi_imx_devtype_data imx31_cspi_devtype_data = {
	.intctrl = mx31_intctrl,
	.config = mx31_config,
	.trigger = mx31_trigger,
	.rx_available = mx31_rx_available,
	.reset = mx31_reset,
	.devtype = IMX31_CSPI,
};

static struct spi_imx_devtype_data imx35_cspi_devtype_data = {
	/* i.mx35 and later cspi shares the functions with i.mx31 one */
	.intctrl = mx31_intctrl,
	.config = mx31_config,
	.trigger = mx31_trigger,
	.rx_available = mx31_rx_available,
	.reset = mx31_reset,
	.devtype = IMX35_CSPI,
};

static struct spi_imx_devtype_data imx51_ecspi_devtype_data = {
	.intctrl = mx51_ecspi_intctrl,
	.config = mx51_ecspi_config,
	.trigger = mx51_ecspi_trigger,
	.rx_available = mx51_ecspi_rx_available,
	.reset = mx51_ecspi_reset,
	.devtype = IMX51_ECSPI,
};

static struct spi_imx_devtype_data imx6ul_ecspi_devtype_data = {
	.intctrl = mx51_ecspi_intctrl,
	.config = mx51_ecspi_config,
	.trigger = mx51_ecspi_trigger,
	.rx_available = mx51_ecspi_rx_available,
	.reset = mx51_ecspi_reset,
	.devtype = IMX6UL_ECSPI,
};

static struct platform_device_id spi_imx_devtype[] = {
	{
		.name = "imx1-cspi",
		.driver_data = (kernel_ulong_t) &imx1_cspi_devtype_data,
	}, {
		.name = "imx21-cspi",
		.driver_data = (kernel_ulong_t) &imx21_cspi_devtype_data,
	}, {
		.name = "imx27-cspi",
		.driver_data = (kernel_ulong_t) &imx27_cspi_devtype_data,
	}, {
		.name = "imx31-cspi",
		.driver_data = (kernel_ulong_t) &imx31_cspi_devtype_data,
	}, {
		.name = "imx35-cspi",
		.driver_data = (kernel_ulong_t) &imx35_cspi_devtype_data,
	}, {
		.name = "imx51-ecspi",
		.driver_data = (kernel_ulong_t) &imx51_ecspi_devtype_data,
	}, {
		.name = "imx6ul-ecspi",
		.driver_data = (kernel_ulong_t) &imx6ul_ecspi_devtype_data,
	}, {
		/* sentinel */
	}
};

static const struct of_device_id spi_imx_dt_ids[] = {
	{ .compatible = "fsl,imx1-cspi", .data = &imx1_cspi_devtype_data, },
	{ .compatible = "fsl,imx21-cspi", .data = &imx21_cspi_devtype_data, },
	{ .compatible = "fsl,imx27-cspi", .data = &imx27_cspi_devtype_data, },
	{ .compatible = "fsl,imx31-cspi", .data = &imx31_cspi_devtype_data, },
	{ .compatible = "fsl,imx35-cspi", .data = &imx35_cspi_devtype_data, },
	{ .compatible = "fsl,imx51-ecspi", .data = &imx51_ecspi_devtype_data, },
	{ .compatible = "fsl,imx6ul-ecspi", .data = &imx6ul_ecspi_devtype_data, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, spi_imx_dt_ids);

static void spi_imx_chipselect(struct spi_device *spi, int is_active)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);
	int gpio = spi_imx->chipselect[spi->chip_select];
	int active = is_active != BITBANG_CS_INACTIVE;
	int dev_is_lowactive = !(spi->mode & SPI_CS_HIGH);

	if (!gpio_is_valid(gpio))
		return;

	gpio_set_value(gpio, dev_is_lowactive ^ active);
}

static void spi_imx_push(struct spi_imx_data *spi_imx)
{
	while (spi_imx->txfifo < spi_imx_get_fifosize(spi_imx)) {
		if (!spi_imx->count)
			break;
		spi_imx->tx(spi_imx);
		spi_imx->txfifo++;
	}

	spi_imx->devtype_data->trigger(spi_imx);
}

static irqreturn_t spi_imx_isr(int irq, void *dev_id)
{
	struct spi_imx_data *spi_imx = dev_id;

	while (spi_imx->devtype_data->rx_available(spi_imx)) {
		spi_imx->rx(spi_imx);
		spi_imx->txfifo--;
	}

	if (spi_imx->count) {
		spi_imx_push(spi_imx);
		return IRQ_HANDLED;
	}

	if (spi_imx->txfifo) {
		/* No data left to push, but still waiting for rx data,
		 * enable receive data available interrupt.
		 */
		spi_imx->devtype_data->intctrl(
				spi_imx, MXC_INT_RR);
		return IRQ_HANDLED;
	}

	spi_imx->devtype_data->intctrl(spi_imx, 0);
	complete(&spi_imx->xfer_done);

	return IRQ_HANDLED;
}

static int spi_imx_setupxfer(struct spi_device *spi,
				 struct spi_transfer *t)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);
	struct spi_imx_config config;
	int ret;

	config.bpw = t ? t->bits_per_word : spi->bits_per_word;
	config.speed_hz  = t ? t->speed_hz : spi->max_speed_hz;
	config.mode = spi->mode;
	config.cs = spi->chip_select;

	if (!config.speed_hz)
		config.speed_hz = spi->max_speed_hz;
	if (!config.bpw)
		config.bpw = spi->bits_per_word;

	/* Initialize the functions for transfer */
	if (config.bpw <= 8) {
		spi_imx->rx = spi_imx_buf_rx_u8;
		spi_imx->tx = spi_imx_buf_tx_u8;
		spi_imx->tx_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		spi_imx->rx_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	} else if (config.bpw <= 16) {
		spi_imx->rx = spi_imx_buf_rx_u16;
		spi_imx->tx = spi_imx_buf_tx_u16;
		spi_imx->tx_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		spi_imx->rx_config.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	} else {
		spi_imx->rx = spi_imx_buf_rx_u32;
		spi_imx->tx = spi_imx_buf_tx_u32;
		spi_imx->tx_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		spi_imx->rx_config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	}

	if (spi_imx->bitbang.master->can_dma &&
	    spi_imx_can_dma(spi_imx->bitbang.master, spi, t)) {
		ret = dmaengine_slave_config(spi_imx->bitbang.master->dma_tx,
						&spi_imx->tx_config);
		if (ret) {
			dev_err(&spi->dev, "error in TX dma configuration.\n");
			return ret;
		}

		ret = dmaengine_slave_config(spi_imx->bitbang.master->dma_rx,
						&spi_imx->rx_config);
		if (ret) {
			dev_err(&spi->dev, "error in RX dma configuration.\n");
			return ret;
		}
	}

	spi_imx->devtype_data->config(spi_imx, &config);

	return 0;
}

static void spi_imx_sdma_exit(struct spi_imx_data *spi_imx)
{
	struct spi_master *master = spi_imx->bitbang.master;

	if (master->dma_rx) {
		dma_release_channel(master->dma_rx);
		master->dma_rx = NULL;
	}

	if (master->dma_tx) {
		dma_release_channel(master->dma_tx);
		master->dma_tx = NULL;
	}

	spi_imx->dma_is_inited = 0;
}

static int spi_imx_sdma_init(struct device *dev, struct spi_imx_data *spi_imx,
			     struct spi_master *master,
			     const struct resource *res)
{
	int ret;

	/* Prepare for TX DMA: */
	master->dma_tx = dma_request_slave_channel(dev, "tx");
	if (!master->dma_tx) {
		dev_err(dev, "cannot get the TX DMA channel!\n");
		ret = -EINVAL;
		goto err;
	}

	spi_imx->tx_config.direction = DMA_MEM_TO_DEV;
	spi_imx->tx_config.dst_addr = res->start + MXC_CSPITXDATA;
	spi_imx->tx_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	spi_imx->tx_config.dst_maxburst = spi_imx_get_fifosize(spi_imx) / 4;
	ret = dmaengine_slave_config(master->dma_tx, &spi_imx->tx_config);
	if (ret) {
		dev_err(dev, "error in TX dma configuration.\n");
		goto err;
	}

	/* Prepare for RX : */
	master->dma_rx = dma_request_slave_channel(dev, "rx");
	if (!master->dma_rx) {
		dev_dbg(dev, "cannot get the DMA channel.\n");
		ret = -EINVAL;
		goto err;
	}

	spi_imx->rx_config.direction = DMA_DEV_TO_MEM;
	spi_imx->rx_config.src_addr = res->start + MXC_CSPIRXDATA;
	spi_imx->rx_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	spi_imx->rx_config.src_maxburst = spi_imx_get_fifosize(spi_imx) / 2;
	ret = dmaengine_slave_config(master->dma_rx, &spi_imx->rx_config);
	if (ret) {
		dev_err(dev, "error in RX dma configuration.\n");
		goto err;
	}

	init_completion(&spi_imx->dma_rx_completion);
	init_completion(&spi_imx->dma_tx_completion);
	master->can_dma = spi_imx_can_dma;
	master->max_dma_len = MAX_SDMA_BD_BYTES;
	spi_imx->bitbang.master->flags = SPI_MASTER_MUST_RX |
					 SPI_MASTER_MUST_TX;
	spi_imx->rx_wml = spi_imx->rx_config.src_maxburst;
	spi_imx->tx_wml = spi_imx->tx_config.dst_maxburst;
	spi_imx->dma_is_inited = 1;

	return 0;
err:
	spi_imx_sdma_exit(spi_imx);
	return ret;
}

static void spi_imx_dma_rx_callback(void *cookie)
{
	struct spi_imx_data *spi_imx = (struct spi_imx_data *)cookie;

	complete(&spi_imx->dma_rx_completion);
}

static void spi_imx_dma_tx_callback(void *cookie)
{
	struct spi_imx_data *spi_imx = (struct spi_imx_data *)cookie;

	complete(&spi_imx->dma_tx_completion);
}

static void spi_imx_tail_pio_set(struct spi_imx_data *spi_imx, int left)
{

	switch (spi_imx->rx_config.src_addr_width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		spi_imx->rx = spi_imx_buf_rx_u8;
		break;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		spi_imx->rx = spi_imx_buf_rx_u16;
		break;
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		spi_imx->rx = spi_imx_buf_rx_u32;
		break;
	default:
		spi_imx->rx = spi_imx_buf_rx_u8;
		break;
	}

	spi_imx->txfifo = left / spi_imx->tx_config.dst_addr_width;
}

static int spi_imx_dma_transfer(struct spi_imx_data *spi_imx,
				struct spi_transfer *transfer)
{
	struct dma_async_tx_descriptor *desc_tx = NULL, *desc_rx = NULL;
	int ret;
	int left = 0;
	struct spi_master *master = spi_imx->bitbang.master;
	struct sg_table *tx = &transfer->tx_sg, *rx = &transfer->rx_sg;

	if (tx) {
		desc_tx = dmaengine_prep_slave_sg(master->dma_tx,
					tx->sgl, tx->nents, DMA_MEM_TO_DEV,
					DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		if (!desc_tx)
			goto tx_nodma;

		desc_tx->callback = spi_imx_dma_tx_callback;
		desc_tx->callback_param = (void *)spi_imx;
		dmaengine_submit(desc_tx);
	}

	if (rx) {
		struct scatterlist *sgl_last = &rx->sgl[rx->nents - 1];
		unsigned int	orig_length = sgl_last->length;
		int	wml_mask = ~(spi_imx->rx_wml - 1);
		/*
		 * Adjust the transfer lenth of the last scattlist if there are
		 * some tail data, use PIO read to get the tail data since DMA
		 * sometimes miss the last tail interrupt.
		 */
		left = transfer->len % spi_imx->rx_wml;
		if (left)
			sgl_last->length = orig_length & wml_mask;

		desc_rx = dmaengine_prep_slave_sg(master->dma_rx,
					rx->sgl, rx->nents, DMA_DEV_TO_MEM,
					DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		if (!desc_rx)
			goto rx_nodma;

		desc_rx->callback = spi_imx_dma_rx_callback;
		desc_rx->callback_param = (void *)spi_imx;
		dmaengine_submit(desc_rx);
	}

	reinit_completion(&spi_imx->dma_rx_completion);
	reinit_completion(&spi_imx->dma_tx_completion);

	/* Trigger the cspi module. */
	spi_imx->dma_finished = 0;
	spi_imx->devtype_data->trigger(spi_imx);

	dma_async_issue_pending(master->dma_tx);
	dma_async_issue_pending(master->dma_rx);
	/* Wait SDMA to finish the data transfer.*/
	ret = wait_for_completion_timeout(&spi_imx->dma_tx_completion,
					  IMX_DMA_TIMEOUT(transfer->len));
	if (!ret) {
		pr_warn("%s %s: I/O Error in DMA TX:%x\n",
			dev_driver_string(&master->dev),
			dev_name(&master->dev), transfer->len);
		dmaengine_terminate_all(master->dma_tx);
	} else {
		ret = wait_for_completion_timeout(&spi_imx->dma_rx_completion,
				IMX_DMA_TIMEOUT(transfer->len));
		if (!ret) {
			pr_warn("%s %s: I/O Error in DMA RX:%x\n",
				dev_driver_string(&master->dev),
				dev_name(&master->dev), transfer->len);
			spi_imx->devtype_data->reset(spi_imx);
			dmaengine_terminate_all(master->dma_rx);
		} else if (left) {
			/* read the tail data by PIO */
			dma_sync_sg_for_cpu(master->dma_rx->device->dev,
					    &rx->sgl[rx->nents - 1], 1,
					    DMA_FROM_DEVICE);
			spi_imx->rx_buf = transfer->rx_buf
						+ (transfer->len - left);
			spi_imx_tail_pio_set(spi_imx, left);
			reinit_completion(&spi_imx->xfer_done);

			spi_imx->devtype_data->intctrl(spi_imx, MXC_INT_TCEN);

			ret = wait_for_completion_timeout(&spi_imx->xfer_done,
						IMX_DMA_TIMEOUT(transfer->len));
			if (!ret) {
				pr_warn("%s %s: I/O Error in RX tail\n",
					dev_driver_string(&master->dev),
					dev_name(&master->dev));
			}
		}
	}

	spi_imx->dma_finished = 1;
	if (spi_imx->devtype_data->devtype == IMX6UL_ECSPI)
		spi_imx->devtype_data->trigger(spi_imx);

	if (!ret)
		ret = -ETIMEDOUT;
	else if (ret > 0)
		ret = transfer->len;

	return ret;

rx_nodma:
	dmaengine_terminate_all(master->dma_tx);
tx_nodma:
	pr_warn_once("%s %s: DMA not available, falling back to PIO\n",
		     dev_driver_string(&master->dev),
		     dev_name(&master->dev));
	return -EAGAIN;
}

static int spi_imx_pio_transfer(struct spi_device *spi,
				struct spi_transfer *transfer)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);

	spi_imx->tx_buf = transfer->tx_buf;
	spi_imx->rx_buf = transfer->rx_buf;
	spi_imx->count = transfer->len;
	spi_imx->txfifo = 0;

	reinit_completion(&spi_imx->xfer_done);

	spi_imx_push(spi_imx);

	spi_imx->devtype_data->intctrl(spi_imx, MXC_INT_TE);

	wait_for_completion(&spi_imx->xfer_done);

	return transfer->len;
}

static int spi_imx_transfer(struct spi_device *spi,
				struct spi_transfer *transfer)
{
	int ret;
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);

	if (spi_imx->bitbang.master->can_dma &&
	    spi_imx_can_dma(spi_imx->bitbang.master, spi, transfer)) {
		spi_imx->usedma = true;
		ret = spi_imx_dma_transfer(spi_imx, transfer);
		spi_imx->usedma = false; /* clear the dma flag */
		if (ret != -EAGAIN)
			return ret;
	}
	spi_imx->usedma = false;

	return spi_imx_pio_transfer(spi, transfer);
}

static int spi_imx_setup(struct spi_device *spi)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);
	int gpio = spi_imx->chipselect[spi->chip_select];

	dev_dbg(&spi->dev, "%s: mode %d, %u bpw, %d hz\n", __func__,
		 spi->mode, spi->bits_per_word, spi->max_speed_hz);

	if (gpio_is_valid(gpio))
		gpio_direction_output(gpio, spi->mode & SPI_CS_HIGH ? 0 : 1);

	spi_imx_chipselect(spi, BITBANG_CS_INACTIVE);

	return 0;
}

static void spi_imx_cleanup(struct spi_device *spi)
{
}

static int
spi_imx_prepare_message(struct spi_master *master, struct spi_message *msg)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(master);
	int ret;

	ret = clk_prepare_enable(spi_imx->clk_per);
	if (ret)
		return ret;

	ret = clk_prepare_enable(spi_imx->clk_ipg);
	if (ret) {
		clk_disable_unprepare(spi_imx->clk_per);
		return ret;
	}

	return 0;
}

static int
spi_imx_unprepare_message(struct spi_master *master, struct spi_message *msg)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(master);

	clk_disable_unprepare(spi_imx->clk_ipg);
	clk_disable_unprepare(spi_imx->clk_per);
	return 0;
}

static int spi_imx_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *of_id =
			of_match_device(spi_imx_dt_ids, &pdev->dev);
	struct spi_imx_master *mxc_platform_info =
			dev_get_platdata(&pdev->dev);
	struct spi_master *master;
	struct spi_imx_data *spi_imx;
	struct resource *res;
	int i, ret, num_cs, irq;

	if (!np && !mxc_platform_info) {
		dev_err(&pdev->dev, "can't get the platform data\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "fsl,spi-num-chipselects", &num_cs);
	if (ret < 0) {
		if (mxc_platform_info)
			num_cs = mxc_platform_info->num_chipselect;
		else
			return ret;
	}

	master = spi_alloc_master(&pdev->dev,
			sizeof(struct spi_imx_data) + sizeof(int) * num_cs);
	if (!master)
		return -ENOMEM;

	platform_set_drvdata(pdev, master);

	master->bits_per_word_mask = SPI_BPW_RANGE_MASK(1, 32);
	master->bus_num = pdev->id;
	master->num_chipselect = num_cs;

	spi_imx = spi_master_get_devdata(master);
	spi_imx->bitbang.master = master;

	for (i = 0; i < master->num_chipselect; i++) {
		int cs_gpio = of_get_named_gpio(np, "cs-gpios", i);
		if (!gpio_is_valid(cs_gpio) && mxc_platform_info)
			cs_gpio = mxc_platform_info->chipselect[i];

		spi_imx->chipselect[i] = cs_gpio;
		if (!gpio_is_valid(cs_gpio))
			continue;

		ret = devm_gpio_request(&pdev->dev, spi_imx->chipselect[i],
					DRIVER_NAME);
		if (ret) {
			dev_err(&pdev->dev, "can't get cs gpios\n");
			goto out_master_put;
		}
	}

	spi_imx->bitbang.chipselect = spi_imx_chipselect;
	spi_imx->bitbang.setup_transfer = spi_imx_setupxfer;
	spi_imx->bitbang.txrx_bufs = spi_imx_transfer;
	spi_imx->bitbang.master->setup = spi_imx_setup;
	spi_imx->bitbang.master->cleanup = spi_imx_cleanup;
	spi_imx->bitbang.master->prepare_message = spi_imx_prepare_message;
	spi_imx->bitbang.master->unprepare_message = spi_imx_unprepare_message;
	spi_imx->bitbang.master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;

	init_completion(&spi_imx->xfer_done);

	spi_imx->devtype_data = of_id ? of_id->data :
		(struct spi_imx_devtype_data *) pdev->id_entry->driver_data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	spi_imx->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(spi_imx->base)) {
		ret = PTR_ERR(spi_imx->base);
		goto out_master_put;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = irq;
		goto out_master_put;
	}

	ret = devm_request_irq(&pdev->dev, irq, spi_imx_isr, 0,
			       dev_name(&pdev->dev), spi_imx);
	if (ret) {
		dev_err(&pdev->dev, "can't get irq%d: %d\n", irq, ret);
		goto out_master_put;
	}

	spi_imx->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(spi_imx->clk_ipg)) {
		ret = PTR_ERR(spi_imx->clk_ipg);
		goto out_master_put;
	}

	spi_imx->clk_per = devm_clk_get(&pdev->dev, "per");
	if (IS_ERR(spi_imx->clk_per)) {
		ret = PTR_ERR(spi_imx->clk_per);
		goto out_master_put;
	}

	ret = clk_prepare_enable(spi_imx->clk_per);
	if (ret)
		goto out_master_put;

	ret = clk_prepare_enable(spi_imx->clk_ipg);
	if (ret)
		goto out_put_per;

	spi_imx->spi_clk = clk_get_rate(spi_imx->clk_per);
	/*
	 * Only validated on i.mx6 now, can remove the constrain if validated on
	 * other chips.
	 */
	if ((spi_imx->devtype_data == &imx51_ecspi_devtype_data
	    || spi_imx->devtype_data == &imx6ul_ecspi_devtype_data)
	    && spi_imx_sdma_init(&pdev->dev, spi_imx, master, res))
		dev_err(&pdev->dev, "dma setup error,use pio instead\n");

	spi_imx->devtype_data->reset(spi_imx);

	spi_imx->devtype_data->intctrl(spi_imx, 0);

	master->dev.of_node = pdev->dev.of_node;
	ret = spi_bitbang_start(&spi_imx->bitbang);
	if (ret) {
		dev_err(&pdev->dev, "bitbang start failed with %d\n", ret);
		goto out_clk_put;
	}

	dev_info(&pdev->dev, "probed\n");

	clk_disable_unprepare(spi_imx->clk_ipg);
	clk_disable_unprepare(spi_imx->clk_per);
	return ret;

out_clk_put:
	clk_disable_unprepare(spi_imx->clk_ipg);
out_put_per:
	clk_disable_unprepare(spi_imx->clk_per);
out_master_put:
	spi_master_put(master);

	return ret;
}

static int spi_imx_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct spi_imx_data *spi_imx = spi_master_get_devdata(master);

	spi_bitbang_stop(&spi_imx->bitbang);

	writel(0, spi_imx->base + MXC_CSPICTRL);
	clk_unprepare(spi_imx->clk_ipg);
	clk_unprepare(spi_imx->clk_per);
	spi_imx_sdma_exit(spi_imx);
	spi_master_put(master);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int spi_imx_suspend(struct device *dev)
{
	pinctrl_pm_select_sleep_state(dev);
	return 0;
}

static int spi_imx_resume(struct device *dev)
{
	pinctrl_pm_select_default_state(dev);
	return 0;
}

static SIMPLE_DEV_PM_OPS(imx_spi_pm, spi_imx_suspend, spi_imx_resume);
#define IMX_SPI_PM       (&imx_spi_pm)
#else
#define IMX_SPI_PM       NULL
#endif

static struct platform_driver spi_imx_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .of_match_table = spi_imx_dt_ids,
		   .pm = IMX_SPI_PM,
	},
	.id_table = spi_imx_devtype,
	.probe = spi_imx_probe,
	.remove = spi_imx_remove,
};
module_platform_driver(spi_imx_driver);

MODULE_DESCRIPTION("SPI Master Controller driver");
MODULE_AUTHOR("Sascha Hauer, Pengutronix");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
