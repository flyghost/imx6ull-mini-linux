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

#ifndef LINUX_MMC_DW_MMC_H
#define LINUX_MMC_DW_MMC_H

#include <linux/scatterlist.h>
#include <linux/mmc/core.h>

#define MAX_MCI_SLOTS	2

enum dw_mci_state {
	STATE_IDLE = 0,				// IDLE状态
	STATE_SENDING_CMD,			// 正在发送CMD状态
	STATE_SENDING_DATA,			// 正在发送DATA
	STATE_DATA_BUSY,			// BUSY
	STATE_SENDING_STOP,			// 正在发送STOP命令
	STATE_DATA_ERROR,			// 发生DATA错误
	STATE_SENDING_CMD11,			// 正在发送CMD11
	STATE_WAITING_CMD11_DONE,		// 等待CMD11结束
};

enum {
	EVENT_CMD_COMPLETE = 0,
	EVENT_XFER_COMPLETE,
	EVENT_DATA_COMPLETE,
	EVENT_DATA_ERROR,
	EVENT_XFER_ERROR
};

struct mmc_data;

/**
 * struct dw_mci - MMC controller state shared between all slots
 * @lock: Spinlock protecting the queue and associated data.
 * @regs: Pointer to MMIO registers.
 * @fifo_reg: Pointer to MMIO registers for data FIFO
 * @sg: Scatterlist entry currently being processed by PIO code, if any.
 * @sg_miter: PIO mapping scatterlist iterator.
 * @cur_slot: The slot which is currently using the controller.
 * @mrq: The request currently being processed on @cur_slot,
 *	or NULL if the controller is idle.
 * @cmd: The command currently being sent to the card, or NULL.
 * @data: The data currently being transferred, or NULL if no data
 *	transfer is in progress.
 * @use_dma: Whether DMA channel is initialized or not.
 * @using_dma: Whether DMA is in use for the current transfer.
 * @dma_64bit_address: Whether DMA supports 64-bit address mode or not.
 * @sg_dma: Bus address of DMA buffer.
 * @sg_cpu: Virtual address of DMA buffer.
 * @dma_ops: Pointer to platform-specific DMA callbacks.
 * @cmd_status: Snapshot of SR taken upon completion of the current
 *	command. Only valid when EVENT_CMD_COMPLETE is pending.
 * @data_status: Snapshot of SR taken upon completion of the current
 *	data transfer. Only valid when EVENT_DATA_COMPLETE or
 *	EVENT_DATA_ERROR is pending.
 * @stop_cmdr: Value to be loaded into CMDR when the stop command is
 *	to be sent.
 * @dir_status: Direction of current transfer.
 * @tasklet: Tasklet running the request state machine.
 * @card_tasklet: Tasklet handling card detect.
 * @pending_events: Bitmask of events flagged by the interrupt handler
 *	to be processed by the tasklet.
 * @completed_events: Bitmask of events which the state machine has
 *	processed.
 * @state: Tasklet state.
 * @queue: List of slots waiting for access to the controller.
 * @bus_hz: The rate of @mck in Hz. This forms the basis for MMC bus
 *	rate and timeout calculations.
 * @current_speed: Configured rate of the controller.
 * @num_slots: Number of slots available.
 * @verid: Denote Version ID.
 * @dev: Device associated with the MMC controller.
 * @pdata: Platform data associated with the MMC controller.
 * @drv_data: Driver specific data for identified variant of the controller
 * @priv: Implementation defined private data.
 * @biu_clk: Pointer to bus interface unit clock instance.
 * @ciu_clk: Pointer to card interface unit clock instance.
 * @slot: Slots sharing this MMC controller.
 * @fifo_depth: depth of FIFO.
 * @data_shift: log2 of FIFO item size.
 * @part_buf_start: Start index in part_buf.
 * @part_buf_count: Bytes of partial data in part_buf.
 * @part_buf: Simple buffer for partial fifo reads/writes.
 * @push_data: Pointer to FIFO push function.
 * @pull_data: Pointer to FIFO pull function.
 * @quirks: Set of quirks that apply to specific versions of the IP.
 * @irq_flags: The flags to be passed to request_irq.
 * @irq: The irq value to be passed to request_irq.
 * @sdio_id0: Number of slot0 in the SDIO interrupt registers.
 *
 * Locking
 * =======
 *
 * @lock is a softirq-safe spinlock protecting @queue as well as
 * @cur_slot, @mrq and @state. These must always be updated
 * at the same time while holding @lock.
 *
 * @irq_lock is an irq-safe spinlock protecting the INTMASK register
 * to allow the interrupt handler to modify it directly.  Held for only long
 * enough to read-modify-write INTMASK and no other locks are grabbed when
 * holding this one.
 *
 * The @mrq field of struct dw_mci_slot is also protected by @lock,
 * and must always be written at the same time as the slot is added to
 * @queue.
 *
 * @pending_events and @completed_events are accessed using atomic bit
 * operations, so they don't need any locking.
 *
 * None of the fields touched by the interrupt handler need any
 * locking. However, ordering is important: Before EVENT_DATA_ERROR or
 * EVENT_DATA_COMPLETE is set in @pending_events, all data-related
 * interrupts must be disabled and @data_status updated with a
 * snapshot of SR. Similarly, before EVENT_CMD_COMPLETE is set, the
 * CMDRDY interrupt must be disabled and @cmd_status updated with a
 * snapshot of SR, and before EVENT_XFER_COMPLETE can be set, the
 * bytes_xfered field of @data must be written. This is ensured by
 * using barriers.
 */
struct dw_mci {
	spinlock_t		lock;			// 保护队列和相关数据的自旋锁
	spinlock_t		irq_lock;		// 保护 INTMASK 设置的自旋锁
	void __iomem		*regs;			// 指向 MMIO 寄存器的指针
	void __iomem		*fifo_reg;		// 指向数据 FIFO 的 MMIO 寄存器的指针

	struct scatterlist	*sg;			// 当前由 PIO 代码处理的散列表项（如果有）
	struct sg_mapping_iter	sg_miter;		// PIO 映射散列表迭代器

	struct dw_mci_slot	*cur_slot;
	struct mmc_request	*mrq;			// 当前正在 slot 上处理的请求，如果控制器处于空闲状态，则为 NULL
	struct mmc_command	*cmd;			// 当前正在发送给卡片的命令，或者为 NULL
	struct mmc_data		*data;			// 当前正在传输的数据，如果没有数据传输正在进行，则为 NULL
	struct mmc_command	stop_abort;		// 当前准备用于停止传输的命令
	unsigned int		prev_blksz;		// 前一个传输块大小记录
	unsigned char		timing;			// 当前 ios 时序记录

	/* DMA interface members*/
	int			use_dma;		// 当前传输使用的 DMA 通道，零表示 PIO 模式
	int			using_dma;		// 当前传输是否使用 DMA
	int			dma_64bit_address;	// DMA 是否支持 64 位地址模式

	dma_addr_t		sg_dma;			// DMA 缓冲区的总线地址
	void			*sg_cpu;		// DMA 缓冲区的虚拟地址
	const struct dw_mci_dma_ops	*dma_ops;	// 指向平台特定 DMA 回调函数的指针
#ifdef CONFIG_MMC_DW_IDMAC
	unsigned int		ring_size;
#else
	struct dw_mci_dma_data	*dma_data;
#endif
	u32			cmd_status;		// 当前命令完成时 SR 的快照。仅在 EVENT_CMD_COMPLETE 挂起时有效
	u32			data_status;		// 当前数据传输完成时 SR 的快照。仅在 EVENT_DATA_COMPLETE 或 EVENT_DATA_ERROR 挂起时有效
	u32			stop_cmdr;		// 当停止命令要发送时要加载到 CMDR 中的值
	u32			dir_status;		// 当前传输的方向
	struct tasklet_struct	tasklet;		// cmd断结束后(包含cmd11超时中断), 执行的底半部处理程序(顶半步处理程序为中断处理程序)
	unsigned long		pending_events;
	unsigned long		completed_events;
	enum dw_mci_state	state;
	struct list_head	queue;

	u32			bus_hz;
	u32			current_speed;
	u32			num_slots;
	u32			fifoth_val;
	u16			verid;
	struct device		*dev;
	struct dw_mci_board	*pdata;
	const struct dw_mci_drv_data	*drv_data;
	void			*priv;
	struct clk		*biu_clk;
	struct clk		*ciu_clk;
	struct dw_mci_slot	*slot[MAX_MCI_SLOTS];

	/* FIFO push and pull */
	int			fifo_depth;
	int			data_shift;
	u8			part_buf_start;
	u8			part_buf_count;
	union {
		u16		part_buf16;
		u32		part_buf32;
		u64		part_buf;
	};
	void (*push_data)(struct dw_mci *host, void *buf, int cnt);
	void (*pull_data)(struct dw_mci *host, void *buf, int cnt);

	/* Workaround flags */
	u32			quirks;

	bool			vqmmc_enabled;
	unsigned long		irq_flags; /* IRQ flags */
	int			irq;

	int			sdio_id0;

	struct timer_list       cmd11_timer;
};

/* DMA ops for Internal/External DMAC interface */
struct dw_mci_dma_ops {
	/* DMA Ops */
	int (*init)(struct dw_mci *host);
	void (*start)(struct dw_mci *host, unsigned int sg_len);
	void (*complete)(struct dw_mci *host);
	void (*stop)(struct dw_mci *host);
	void (*cleanup)(struct dw_mci *host);
	void (*exit)(struct dw_mci *host);
};

/* IP Quirks/flags. */
/* DTO fix for command transmission with IDMAC configured */
#define DW_MCI_QUIRK_IDMAC_DTO			BIT(0)
/* delay needed between retries on some 2.11a implementations */
#define DW_MCI_QUIRK_RETRY_DELAY		BIT(1)
/* High Speed Capable - Supports HS cards (up to 50MHz) */
#define DW_MCI_QUIRK_HIGHSPEED			BIT(2)
/* Unreliable card detection */
#define DW_MCI_QUIRK_BROKEN_CARD_DETECTION	BIT(3)
/* No write protect */
#define DW_MCI_QUIRK_NO_WRITE_PROTECT		BIT(4)

/* Slot level quirks */
/* This slot has no write protect */
#define DW_MCI_SLOT_QUIRK_NO_WRITE_PROTECT	BIT(0)

struct dma_pdata;

struct block_settings {
	unsigned short	max_segs;	/* see blk_queue_max_segments */
	unsigned int	max_blk_size;	/* maximum size of one mmc block */
	unsigned int	max_blk_count;	/* maximum number of blocks in one req*/
	unsigned int	max_req_size;	/* maximum number of bytes in one req*/
	unsigned int	max_seg_size;	/* see blk_queue_max_segment_size */
};

/* Board platform data */
struct dw_mci_board {
	u32 num_slots;								// 插槽数量

	u32 quirks; /* Workaround / Quirk flags */				// 控制器需要的工作区或变通方法的标志
	unsigned int bus_hz; /* Clock speed at the cclk_in pad */		// 总线频率

	u32 caps;	/* Capabilities */					// 控制器能力
	u32 caps2;	/* More capabilities */					// 控制器更多能力
	u32 pm_caps;	/* PM capabilities */					// 控制器电源管理能力
	/*
	 * Override fifo depth. If 0, autodetect it from the FIFOTH register,
	 * but note that this may not be reliable after a bootloader has used
	 * it.
	 */
	unsigned int fifo_depth;						// FIFO深度, 如果为0, 则从FIFOTH寄存器自动检测, 但注意在引导加载程序使用后可能不可靠

	/* delay in mS before detecting cards after interrupt */
	u32 detect_delay_ms;							// 在中断后检测卡片前的延迟（以毫秒为单位）

	struct dw_mci_dma_ops *dma_ops;						// 指向平台特定 DMA 回调函数的指针
	struct dma_pdata *data;							// dma 数据
	struct block_settings *blk_settings;					// 块设备结构体
};

#endif /* LINUX_MMC_DW_MMC_H */
