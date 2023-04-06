/*
 * Synopsys DesignWare Multimedia Card Interface driver
 *  (Based on NXP driver for lpc 31xx)
 *
 * Copyright (C) 2009 NXP Semiconductors
 * Copyright (C) 2009, 2010 Imagination Technologies Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _DW_MMC_H_
#define _DW_MMC_H_

#define DW_MMC_240A		0x240a

#define SDMMC_CTRL		0x000
#define SDMMC_PWREN		0x004
#define SDMMC_CLKDIV		0x008
#define SDMMC_CLKSRC		0x00c
#define SDMMC_CLKENA		0x010
#define SDMMC_TMOUT		0x014
#define SDMMC_CTYPE		0x018
#define SDMMC_BLKSIZ		0x01c
#define SDMMC_BYTCNT		0x020
#define SDMMC_INTMASK		0x024
#define SDMMC_CMDARG		0x028
#define SDMMC_CMD		0x02c
#define SDMMC_RESP0		0x030
#define SDMMC_RESP1		0x034
#define SDMMC_RESP2		0x038
#define SDMMC_RESP3		0x03c
#define SDMMC_MINTSTS		0x040
#define SDMMC_RINTSTS		0x044
#define SDMMC_STATUS		0x048
#define SDMMC_FIFOTH		0x04c
#define SDMMC_CDETECT		0x050
#define SDMMC_WRTPRT		0x054
#define SDMMC_GPIO		0x058
#define SDMMC_TCBCNT		0x05c
#define SDMMC_TBBCNT		0x060
#define SDMMC_DEBNCE		0x064
#define SDMMC_USRID		0x068
#define SDMMC_VERID		0x06c
#define SDMMC_HCON		0x070
#define SDMMC_UHS_REG		0x074
#define SDMMC_BMOD		0x080
#define SDMMC_PLDMND		0x084
#define SDMMC_DBADDR		0x088
#define SDMMC_IDSTS		0x08c
#define SDMMC_IDINTEN		0x090
#define SDMMC_DSCADDR		0x094
#define SDMMC_BUFADDR		0x098
#define SDMMC_CDTHRCTL		0x100
#define SDMMC_DATA(x)		(x)
/*
* Registers to support idmac 64-bit address mode
*/
#define SDMMC_DBADDRL		0x088
#define SDMMC_DBADDRU		0x08c
#define SDMMC_IDSTS64		0x090
#define SDMMC_IDINTEN64		0x094
#define SDMMC_DSCADDRL		0x098
#define SDMMC_DSCADDRU		0x09c
#define SDMMC_BUFADDRL		0x0A0
#define SDMMC_BUFADDRU		0x0A4

/*
 * Data offset is difference according to Version
 * Lower than 2.40a : data register offest is 0x100
 */
#define DATA_OFFSET		0x100
#define DATA_240A_OFFSET	0x200

/* shift bit field */
#define _SBF(f, v)		((v) << (f))

/* Control register defines */
#define SDMMC_CTRL_USE_IDMAC		BIT(25)
#define SDMMC_CTRL_CEATA_INT_EN		BIT(11)
#define SDMMC_CTRL_SEND_AS_CCSD		BIT(10)
#define SDMMC_CTRL_SEND_CCSD		BIT(9)
#define SDMMC_CTRL_ABRT_READ_DATA	BIT(8)
#define SDMMC_CTRL_SEND_IRQ_RESP	BIT(7)
#define SDMMC_CTRL_READ_WAIT		BIT(6)
#define SDMMC_CTRL_DMA_ENABLE		BIT(5)
#define SDMMC_CTRL_INT_ENABLE		BIT(4)
#define SDMMC_CTRL_DMA_RESET		BIT(2)
#define SDMMC_CTRL_FIFO_RESET		BIT(1)
#define SDMMC_CTRL_RESET		BIT(0)
/* Clock Enable register defines */
#define SDMMC_CLKEN_LOW_PWR		BIT(16)
#define SDMMC_CLKEN_ENABLE		BIT(0)
/* time-out register defines */
#define SDMMC_TMOUT_DATA(n)		_SBF(8, (n))
#define SDMMC_TMOUT_DATA_MSK		0xFFFFFF00
#define SDMMC_TMOUT_RESP(n)		((n) & 0xFF)
#define SDMMC_TMOUT_RESP_MSK		0xFF
/* card-type register defines */
#define SDMMC_CTYPE_8BIT		BIT(16)
#define SDMMC_CTYPE_4BIT		BIT(0)
#define SDMMC_CTYPE_1BIT		0
/* Interrupt status & mask register defines */
#define SDMMC_INT_SDIO(n)		BIT(16 + (n))
#define SDMMC_INT_EBE			BIT(15)
#define SDMMC_INT_ACD			BIT(14)
#define SDMMC_INT_SBE			BIT(13)
#define SDMMC_INT_HLE			BIT(12)
#define SDMMC_INT_FRUN			BIT(11)
#define SDMMC_INT_HTO			BIT(10)
#define SDMMC_INT_VOLT_SWITCH		BIT(10) /* overloads bit 10! */
#define SDMMC_INT_DRTO			BIT(9)
#define SDMMC_INT_RTO			BIT(8)
#define SDMMC_INT_DCRC			BIT(7)
#define SDMMC_INT_RCRC			BIT(6)
#define SDMMC_INT_RXDR			BIT(5)
#define SDMMC_INT_TXDR			BIT(4)
#define SDMMC_INT_DATA_OVER		BIT(3)
#define SDMMC_INT_CMD_DONE		BIT(2)
#define SDMMC_INT_RESP_ERR		BIT(1)
#define SDMMC_INT_CD			BIT(0)
#define SDMMC_INT_ERROR			0xbfc2
/* Command register defines */
#define SDMMC_CMD_START			BIT(31)		// 开始命令,一旦命令被CIU取走,该位清零
#define SDMMC_CMD_USE_HOLD_REG	BIT(29)			// 发送给CARD的命令和数据是否经过HOLD寄存器
#define SDMMC_CMD_VOLT_SWITCH		BIT(28)		// 开启电压切换(必须也只能通过CMD11设置)
#define SDMMC_CMD_CCS_EXP		BIT(23)
#define SDMMC_CMD_CEATA_RD		BIT(22)
#define SDMMC_CMD_UPD_CLK		BIT(21)		// 0: 正常命令序列     1: 不发送命令, 只给卡的时钟域更新时钟寄存器的值(包括CLKDIIV, CLRSRC, CLKENA)
#define SDMMC_CMD_INIT			BIT(15)		// 在发送命令前是否发送初始化序列,这样才可以在发送命令前初始化时钟(上电后,在给卡发送任意的命令前,必须发送80个时钟来初始化.所以在发送第一条命令时必须设置该bit位)
#define SDMMC_CMD_STOP			BIT(14)		// 停止当前的传输
#define SDMMC_CMD_PRV_DAT_WAIT		BIT(13)		// 是否等待之前的数据发送结束再发送当前数据    1: 等待  0: 不等待
#define SDMMC_CMD_SEND_STOP		BIT(12)		// 传输结束后是否发送结束信号     1: 发送   0: 不发送
#define SDMMC_CMD_STRM_MODE		BIT(11)		// 传输模式:   1: 块数据传输命令     0: 流数据传输命令
#define SDMMC_CMD_DAT_WR		BIT(10)		// 传输方向:   1: 写入卡     0: 从卡读
#define SDMMC_CMD_DAT_EXP		BIT(9)		// 1: 有数据需要传输       0: 没有数据需要传输
#define SDMMC_CMD_RESP_CRC		BIT(8)		// 1: 需要检查响应的CRC    0: 不需要检查响应的CRC(有一些响应不返回有效的CRC位,软件需要关闭这些命令的CRC检查)
#define SDMMC_CMD_RESP_LONG		BIT(7)		// 1: 期望从卡收到长响应   0: 期望从卡收到短响应
#define SDMMC_CMD_RESP_EXP		BIT(6)		// 需要卡响应
#define SDMMC_CMD_INDX(n)		((n) & 0x1F)
/* Status register defines */
#define SDMMC_GET_FCNT(x)		(((x)>>17) & 0x1FFF)
#define SDMMC_STATUS_DMA_REQ		BIT(31)
#define SDMMC_STATUS_BUSY		BIT(9)
/* FIFOTH register defines */
#define SDMMC_SET_FIFOTH(m, r, t)	(((m) & 0x7) << 28 | \
					 ((r) & 0xFFF) << 16 | \
					 ((t) & 0xFFF))
/* Internal DMAC interrupt defines */
#define SDMMC_IDMAC_INT_AI		BIT(9)
#define SDMMC_IDMAC_INT_NI		BIT(8)
#define SDMMC_IDMAC_INT_CES		BIT(5)
#define SDMMC_IDMAC_INT_DU		BIT(4)
#define SDMMC_IDMAC_INT_FBE		BIT(2)
#define SDMMC_IDMAC_INT_RI		BIT(1)
#define SDMMC_IDMAC_INT_TI		BIT(0)
/* Internal DMAC bus mode bits */
#define SDMMC_IDMAC_ENABLE		BIT(7)
#define SDMMC_IDMAC_FB			BIT(1)
#define SDMMC_IDMAC_SWRESET		BIT(0)
/* Version ID register define */
#define SDMMC_GET_VERID(x)		((x) & 0xFFFF)
/* Card read threshold */
#define SDMMC_SET_RD_THLD(v, x)		(((v) & 0x1FFF) << 16 | (x))
#define SDMMC_UHS_18V			BIT(0)
/* All ctrl reset bits */
#define SDMMC_CTRL_ALL_RESET_FLAGS \
	(SDMMC_CTRL_RESET | SDMMC_CTRL_FIFO_RESET | SDMMC_CTRL_DMA_RESET)

/* FIFO register access macros. These should not change the data endian-ness
 * as they are written to memory to be dealt with by the upper layers */
#define mci_fifo_readw(__reg)	__raw_readw(__reg)
#define mci_fifo_readl(__reg)	__raw_readl(__reg)
#define mci_fifo_readq(__reg)	__raw_readq(__reg)

#define mci_fifo_writew(__value, __reg)	__raw_writew(__reg, __value)
#define mci_fifo_writel(__value, __reg)	__raw_writel(__reg, __value)
#define mci_fifo_writeq(__value, __reg)	__raw_writeq(__reg, __value)

/* Register access macros */
#define mci_readl(dev, reg)			\
	readl_relaxed((dev)->regs + SDMMC_##reg)
#define mci_writel(dev, reg, value)			\
	writel_relaxed((value), (dev)->regs + SDMMC_##reg)

/* 16-bit FIFO access macros */
#define mci_readw(dev, reg)			\
	readw_relaxed((dev)->regs + SDMMC_##reg)
#define mci_writew(dev, reg, value)			\
	writew_relaxed((value), (dev)->regs + SDMMC_##reg)

/* 64-bit FIFO access macros */
#ifdef readq
#define mci_readq(dev, reg)			\
	readq_relaxed((dev)->regs + SDMMC_##reg)
#define mci_writeq(dev, reg, value)			\
	writeq_relaxed((value), (dev)->regs + SDMMC_##reg)
#else
/*
 * Dummy readq implementation for architectures that don't define it.
 *
 * We would assume that none of these architectures would configure
 * the IP block with a 64bit FIFO width, so this code will never be
 * executed on those machines. Defining these macros here keeps the
 * rest of the code free from ifdefs.
 */
#define mci_readq(dev, reg)			\
	(*(volatile u64 __force *)((dev)->regs + SDMMC_##reg))
#define mci_writeq(dev, reg, value)			\
	(*(volatile u64 __force *)((dev)->regs + SDMMC_##reg) = (value))

#define __raw_writeq(__value, __reg) \
	(*(volatile u64 __force *)(__reg) = (__value))
#define __raw_readq(__reg) (*(volatile u64 __force *)(__reg))
#endif

extern int dw_mci_probe(struct dw_mci *host);
extern void dw_mci_remove(struct dw_mci *host);
#ifdef CONFIG_PM_SLEEP
extern int dw_mci_suspend(struct dw_mci *host);
extern int dw_mci_resume(struct dw_mci *host);
#endif

/**
 * struct dw_mci_slot - MMC slot state
 * @mmc: The mmc_host representing this slot.
 * @host: The MMC controller this slot is using.
 * @quirks: Slot-level quirks (DW_MCI_SLOT_QUIRK_XXX)
 * @ctype: Card type for this slot.
 * @mrq: mmc_request currently being processed or waiting to be
 *	processed, or NULL when the slot is idle.
 * @queue_node: List node for placing this node in the @queue list of
 *	&struct dw_mci.
 * @clock: Clock rate configured by set_ios(). Protected by host->lock.
 * @__clk_old: The last updated clock with reflecting clock divider.
 *	Keeping track of this helps us to avoid spamming the console
 *	with CONFIG_MMC_CLKGATE.
 * @flags: Random state bits associated with the slot.
 * @id: Number of this slot.
 * @sdio_id: Number of this slot in the SDIO interrupt registers.
 */
struct dw_mci_slot {
	struct mmc_host		*mmc;
	struct dw_mci		*host;

	int			quirks;

	u32			ctype;

	struct mmc_request	*mrq;
	struct list_head	queue_node;

	unsigned int		clock;
	unsigned int		__clk_old;

	unsigned long		flags;
#define DW_MMC_CARD_PRESENT	0
#define DW_MMC_CARD_NEED_INIT	1
#define DW_MMC_CARD_NO_LOW_PWR	2
	int			id;
	int			sdio_id;
};

/**
 * dw_mci driver data - dw-mshc implementation specific driver data.
 * @caps: mmc subsystem specified capabilities of the controller(s).
 * @init: early implementation specific initialization.
 * @setup_clock: implementation specific clock configuration.
 * @prepare_command: handle CMD register extensions.
 * @set_ios: handle bus specific extensions.
 * @parse_dt: parse implementation specific device tree properties.
 * @execute_tuning: implementation specific tuning procedure.
 *
 * Provide controller implementation specific extensions. The usage of this
 * data structure is fully optional and usage of each member in this structure
 * is optional as well.
 */
struct dw_mci_drv_data {
	unsigned long	*caps;
	int		(*init)(struct dw_mci *host);
	int		(*setup_clock)(struct dw_mci *host);
	void		(*prepare_command)(struct dw_mci *host, u32 *cmdr);
	void		(*set_ios)(struct dw_mci *host, struct mmc_ios *ios);
	int		(*parse_dt)(struct dw_mci *host);
	int		(*execute_tuning)(struct dw_mci_slot *slot);
	int		(*prepare_hs400_tuning)(struct dw_mci *host,
						struct mmc_ios *ios);
};
#endif /* _DW_MMC_H_ */
