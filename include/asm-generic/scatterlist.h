#ifndef __ASM_GENERIC_SCATTERLIST_H
#define __ASM_GENERIC_SCATTERLIST_H

#include <linux/types.h>

/**
 * @brief 在 scatterlist 链表中，每个 scatterlist 结构体都可以包含多个内存页。
 * 
 */
struct scatterlist {
#ifdef CONFIG_DEBUG_SG
	unsigned long	sg_magic;
#endif
	/**
	 * @brief 指示该内存块的下一个scatterlist
	 * 
	 * bit0 和 bit1 有特殊用途(可参考后面的介绍),因此要求 page 最低 4 字节对齐
	 * 
	 * bit0=1, 表示该scatterlist不是一个有效的内存卡,而是一个chain(铰链), 指向另一个scatterlist数组
	 * bit1=1, 表示该scatterlist是scatterlist数组中最后一个有效内存块(后面就忽略不计了)
	 * 
	 * 通过这种机制, 可以将不同的scatterlist数组链在一起, 因为scatterlist也称作chain scatterlist
	 */
	unsigned long	page_link;
	unsigned int	offset;			// 指示该内存块在页面中的偏移(起始位置)
	unsigned int	length;			// 该内存块的长度
	dma_addr_t	dma_address;		// 该内存块实际的起始地址(PA,相比 page 更接近我们人类的语言)
#ifdef CONFIG_NEED_SG_DMA_LENGTH
	unsigned int	dma_length;		// 相应的长度信息
#endif
};

/*
 * These macros should be used after a dma_map_sg call has been done
 * to get bus addresses of each of the SG entries and their lengths.
 * You should only work with the number of sg entries pci_map_sg
 * returns, or alternatively stop on the first sg_dma_len(sg) which
 * is 0.
 */
#define sg_dma_address(sg)	((sg)->dma_address)		// 获取一个scatterlist的物理地址

#ifdef CONFIG_NEED_SG_DMA_LENGTH
#define sg_dma_len(sg)		((sg)->dma_length)		// 获取一个scatterlist的长度
#else
#define sg_dma_len(sg)		((sg)->length)
#endif

#endif /* __ASM_GENERIC_SCATTERLIST_H */
