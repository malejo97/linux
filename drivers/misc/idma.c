#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of_platform.h>
#include <linux/bitfield.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/types.h>
#include <linux/pci_regs.h>
#include <linux/iommu.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/param.h>
#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/timekeeping.h>

MODULE_DESCRIPTION("Simple iDMA driver for the PULP iDMA module");
MODULE_AUTHOR("Manuel Rodríguez <manuel.cederog@gmail.com>");
MODULE_ALIAS("idma");
MODULE_LICENSE("GPL v2");

static int use_unmapped_addr = 0;
module_param(use_unmapped_addr, int, 0644);
MODULE_PARM_DESC(use_unmapped_addr, "Flag to use unmapped addresses to trigger IOMMU page fault");

static int idma_n = 0;

static int transfer_number = 0;
static u64 ktime_cnt = 0;

/**
 * 	Use DMA consistent mappings instead of streaming mappings.
 * 	Streaming DMA mappings are created on every transfer. Thus, when using an IOMMU, internal caches 
 * 		are flushed to invalidate all entries associated with the transfer.
 * 	Consistent DMA mappings are created in the driver initialization, 
 * 		and maintained during the lifetime of the driver.
 * 		Internal IOMMU caches are not flushed unless explicitely requested.
 */
#define CONSISTENT_MAPPING 	(1)

/**
 * 	Number of mappings (PTs) per device
 */
#define N_MAPPINGS			(1)

/**
 *	Include code to measure transfer times using iDMA interrupts
 */
#define ENABLE_IRQ			(0)

// Source Address
#define IDMA_SRC_ADDR_REG_OFFSET 0x0

// Destination Address
#define IDMA_DST_ADDR_REG_OFFSET 0x8

// Number of bytes
#define IDMA_NUM_BYTES_REG_OFFSET 0x10

// Configuration Register for DMA settings
#define IDMA_CONF_REG_OFFSET 0x18

#define IDMA_CONF_DECOUPLE_BIT 0
#define IDMA_CONF_DEBURST_BIT 1
#define IDMA_CONF_SERIALIZE_BIT 2

// DMA Status
#define IDMA_STATUS_REG_OFFSET 0x20

#define IDMA_STATUS_BUSY_BIT 0

// Next ID, launches transfer, returns 0 if transfer not set up properly.
#define IDMA_NEXT_ID_REG_OFFSET 0x28

// Get ID of finished transactions.
#define IDMA_DONE_REG_OFFSET 0x30

// Interrupt Pending Status Register
#define IDMA_IPSR_REG_OFFSET 	0x38
#define IDMA_IPSR_RIP			(1ULL << 0)
#define IDMA_IPSR_WIP			(1ULL << 1)

struct idma_device {
	struct miscdevice miscdev;
	void __iomem *reg;
	void *rd_vptr[N_MAPPINGS], *wr_vptr[N_MAPPINGS];
	dma_addr_t rd_dmaptr[N_MAPPINGS], wr_dmaptr[N_MAPPINGS];
	struct mutex lock;
	refcount_t ref;
	u64 kt_st;
	u64 kt_rd;
	u64 kt_wr;	
};

static int idma_open(struct inode *inode, struct file *fp)
{
	#if (ENABLE_IRQ == 1)
	transfer_number = 0;
	ktime_cnt = 0;
	#endif

	return 0;
}

static int idma_release(struct inode *inode, struct file *fp)
{
	#if (ENABLE_IRQ == 1)
	pr_info("Time avg: %llu ns\n\n", ktime_cnt/transfer_number);
	#endif
	
	return 0;
}

/**
 *        |----------|
 *        ^          v
 * 		|----------|----------|
 * 		|   4kiB   |   4kiB   |
 * 		|----------|----------|
 * 
 * 	Copies 'count' bytes from the first page of user memory pointed by 'buf' to the 
 * 	base address of the next (contiguous) page of user memory.
 * 
 * 	NOTES: 
 * 	-	'buf' must be aligned to 4kiB and point to two contiguous pages of user memory
 * 	-	'count' must not be greater than 4kiB.
 * 
 */
static ssize_t idma_write(struct file *fp, const char __user *buf, size_t count, loff_t *ppos)
{
	int ret;
	struct page *page_ro[1];
	struct page *page_wr[1];
	struct idma_device *idma_dev = fp->private_data;
	u64 rand_r = 0, idx_r = 0;
	u64 rand_w = 0, idx_w = 0;

	u64 src  = (u64)buf;					// base address of the user page where src data is
	u64 dst  = (u64)(PAGE_ALIGN(src + 1));	// base address of the user page where data is to be placed
	u64 cnt = count;						// number of bytes to be transferred
	u64 conf = 0;							// iDMA configuration
	u64 transfer_id;						// ID of the transfer

	// Check whether the input address is aligned
	if (offset_in_page(buf) != 0)
	{
		pr_err("Source buffer is not aligned!\n");
		return -EINVAL;
	}

	// Page crossing not supported
	if (cnt > 4096)
		return -EINVAL;

	// Generate random index
	get_random_bytes(&rand_r, sizeof(rand_r));
	idx_r = rand_r % N_MAPPINGS;
	get_random_bytes(&rand_w, sizeof(rand_w));
	idx_w = rand_w % N_MAPPINGS;

	// Use streaming DMA mappings
	if(!CONSISTENT_MAPPING)
	{
		// pin user memory page where the src data resides
		ret = pin_user_pages_fast(src, 1, 0, page_ro);
		if (ret != 1) {
			pr_err("Failure locking RO pages.\n");
			return -ENOMEM;
		}
		// pin user memory page where data will be written
		ret = pin_user_pages_fast(dst, 1, FOLL_WRITE, page_wr);
		if (ret != 1) {
			pr_err("Failure locking WR pages.\n");
			return -ENOMEM;
		}

		// Configure PTs in the IOMMU (streaming mappings)
		// Read address
		idma_dev->rd_dmaptr[idx_r] = dma_map_page(idma_dev->miscdev.parent, page_ro[0],
					offset_in_page(src), cnt, DMA_TO_DEVICE);
		ret = dma_mapping_error(idma_dev->miscdev.parent, idma_dev->rd_dmaptr[idx_r]);

		if (ret) {
			pr_err("Failure mapping src page.\n");
			return -ENOMEM;
		}

		// Write address
		idma_dev->wr_dmaptr[idx_w] = dma_map_page(idma_dev->miscdev.parent, page_wr[0],
					offset_in_page(dst), cnt, DMA_FROM_DEVICE);
		ret = dma_mapping_error(idma_dev->miscdev.parent, idma_dev->wr_dmaptr[idx_w]);
		if (ret) {
			pr_err("Failure mapping dest page.\n");
			return -ENOMEM;
		}
	}
	
	// Use consistent DMA mappings
	else
	{
		// Copy user data to src address
		copy_from_user(idma_dev->rd_vptr[idx_r], (void*)buf, SZ_4K);
	}

	// Configure iDMA
	mutex_lock(&idma_dev->lock);
	if (use_unmapped_addr)
	{
		pr_info("Using src DMA Addr -> %llx\n", src);
		pr_info("Using dst DMA Addr -> %llx\n", dst);
		writeq(src,  idma_dev->reg + IDMA_SRC_ADDR_REG_OFFSET);
		writeq(dst,  idma_dev->reg + IDMA_DST_ADDR_REG_OFFSET);
	}
	else
	{
		pr_info("Using src DMA Addr -> %llx\n", idma_dev->rd_dmaptr[idx_r]);
		pr_info("Using dst DMA Addr -> %llx\n", idma_dev->wr_dmaptr[idx_w]);
		writeq(idma_dev->rd_dmaptr[idx_r],  idma_dev->reg + IDMA_SRC_ADDR_REG_OFFSET);
		writeq(idma_dev->wr_dmaptr[idx_w],  idma_dev->reg + IDMA_DST_ADDR_REG_OFFSET);
	}

	writeq(cnt,  		idma_dev->reg + IDMA_NUM_BYTES_REG_OFFSET);
	writeq(conf, 		idma_dev->reg + IDMA_CONF_REG_OFFSET);
	
	// Get transfer ID and trigger transfer
	#if (ENABLE_IRQ == 1)
	idma_dev->kt_st		= ktime_get_ns();
	#endif
	transfer_id = readq(idma_dev->reg + IDMA_NEXT_ID_REG_OFFSET);
	if (!transfer_id)
	{
		pr_err("Failure configuring iDMA module.\n");
		return -EINVAL;
	}

	// Wait for transfer to be done
	do {
		cpu_relax();
	} while (readq(idma_dev->reg + IDMA_DONE_REG_OFFSET) != transfer_id);
	mutex_unlock(&idma_dev->lock);

	// Calculate latency
	#if (ENABLE_IRQ == 1)
	transfer_number++;
	ktime_cnt += (idma_dev->kt_wr - idma_dev->kt_st);
	// pr_info("T[%d]: %llu ns\n\n", transfer_number, ktime_cnt);
	// pr_info("Time avg: %llu ns\n\n", ktime_cnt/transfer_number);
	#endif

	if (!CONSISTENT_MAPPING)
	{
		// Unmap pages
		dma_unmap_page(idma_dev->miscdev.parent, idma_dev->rd_dmaptr[idx_r], cnt, DMA_TO_DEVICE);
		dma_unmap_page(idma_dev->miscdev.parent, idma_dev->wr_dmaptr[idx_w], cnt, DMA_FROM_DEVICE);
		unpin_user_pages_dirty_lock(page_ro, 1, false);
		unpin_user_pages_dirty_lock(page_wr, 1, true);
	}

	else
	{
		// Copy back data to user page
		copy_to_user((void*)PAGE_ALIGN((u64)buf + 1), idma_dev->wr_vptr[idx_w], SZ_4K);
		// Clear destination buffer
		memset(idma_dev->wr_vptr[idx_w], 0, SZ_4K);
	}

	return 0;
}

/**
 *        |----------|
 *        v          ^
 * 		|----------|----------|
 * 		|   4kiB   |   4kiB   |
 * 		|----------|----------|
 * 
 * 	Copies 'count' bytes from the second page of user memory to the 
 * 	base address of the previous (contiguous) page of user memory, pointed by 'buf'.
 * 
 * 	NOTES: 
 * 	-	'buf' must be aligned to 4kiB and point to two contiguous pages of user memory
 * 	-	'count' must not be greater than 4kiB.
 * 
 */
static ssize_t idma_read(struct file *fp, char __user *buf, size_t count, loff_t *ppos)
{
	int ret;
	struct page *page_ro[1];
	struct page *page_wr[1];
	struct idma_device *idma_dev = fp->private_data;

	u64 dst  = (u64)buf;					// base address of the user page where data is to be placed
	u64 src  = (u64)(PAGE_ALIGN(dst + 1));	// base address of the user page where src data is
	u64 cnt  = count;						// number of bytes to be transferred
	u64 conf = 0;							// iDMA configuration
	u64 transfer_id;
	u64 rand_r = 0, idx_r = 0;
	u64 rand_w = 0, idx_w = 0;

	// Check whether the input address is aligned
	if (offset_in_page(buf) != 0)
	{
		pr_err("Source buffer is not aligned!\n");
		return -EINVAL;
	}

	// Page crossing not supported
	if (cnt > 4096)
		return -EINVAL;

	// Generate random index
	get_random_bytes(&rand_r, sizeof(rand_r));
	idx_r = rand_r % N_MAPPINGS;
	get_random_bytes(&rand_w, sizeof(rand_w));
	idx_w = rand_w % N_MAPPINGS;

	// Use streaming DMA mappings
	if(!CONSISTENT_MAPPING)
	{
		// pin user memory page where the src data resides
		ret = pin_user_pages_fast(src, 1, 0, page_ro);
		if (ret != 1) {
			pr_err("Failure locking RO pages.\n");
			return -ENOMEM;
		}
		// pin user memory page where data will be written
		ret = pin_user_pages_fast(dst, 1, FOLL_WRITE, page_wr);
		if (ret != 1) {
			pr_err("Failure locking WR pages.\n");
			return -ENOMEM;
		}

		// Configure PTs in the IOMMU (streaming mappings)
		// Read address
		idma_dev->rd_dmaptr[idx_r] = dma_map_page(idma_dev->miscdev.parent, page_ro[0],
					offset_in_page(src), cnt, DMA_TO_DEVICE);
		ret = dma_mapping_error(idma_dev->miscdev.parent, idma_dev->rd_dmaptr[idx_r]);

		if (ret) {
			pr_err("Failure mapping src page.\n");
			return -ENOMEM;
		}

		// Write address
		idma_dev->wr_dmaptr[idx_w] = dma_map_page(idma_dev->miscdev.parent, page_wr[0],
					offset_in_page(dst), cnt, DMA_FROM_DEVICE);
		ret = dma_mapping_error(idma_dev->miscdev.parent, idma_dev->wr_dmaptr[idx_w]);
		if (ret) {
			pr_err("Failure mapping dest page.\n");
			return -ENOMEM;
		}
	}
	
	// Use consistent DMA mappings
	else
	{
		// Copy user data to src address
		copy_from_user(idma_dev->rd_vptr[idx_r], (void*)buf, SZ_4K);
	}

	// Configure iDMA
	mutex_lock(&idma_dev->lock);
	if (use_unmapped_addr)
	{
		pr_info("Using src DMA Addr -> %llx\n", src);
		pr_info("Using dst DMA Addr -> %llx\n", dst);
		writeq(src,  idma_dev->reg + IDMA_SRC_ADDR_REG_OFFSET);
		writeq(dst,  idma_dev->reg + IDMA_DST_ADDR_REG_OFFSET);
	}
	else
	{
		pr_info("Using src DMA Addr -> %llx\n", idma_dev->rd_dmaptr[idx_r]);
		pr_info("Using dst DMA Addr -> %llx\n", idma_dev->wr_dmaptr[idx_w]);
		writeq(idma_dev->rd_dmaptr[idx_r],  idma_dev->reg + IDMA_SRC_ADDR_REG_OFFSET);
		writeq(idma_dev->wr_dmaptr[idx_w],  idma_dev->reg + IDMA_DST_ADDR_REG_OFFSET);
	}

	writeq(cnt,  		idma_dev->reg + IDMA_NUM_BYTES_REG_OFFSET);
	writeq(conf, 		idma_dev->reg + IDMA_CONF_REG_OFFSET);
	
	// Get transfer ID and trigger transfer
	#if (ENABLE_IRQ == 1)
	idma_dev->kt_st		= ktime_get_ns();
	#endif
	transfer_id = readq(idma_dev->reg + IDMA_NEXT_ID_REG_OFFSET);
	if (!transfer_id)
	{
		pr_err("Failure configuring iDMA module.\n");
		return -EINVAL;
	}

	// Wait for transfer to be done
	do {
		cpu_relax();
	} while (readq(idma_dev->reg + IDMA_DONE_REG_OFFSET) != transfer_id);
	mutex_unlock(&idma_dev->lock);

	// Calculate latency
	#if (ENABLE_IRQ == 1)
	transfer_number++;
	ktime_cnt += (idma_dev->kt_wr - idma_dev->kt_st);
	// pr_info("T: %llu ns\n\n", ktime_cnt);
	// pr_info("Time avg: %llu ns\n\n", ktime_cnt/transfer_number);
	#endif

	if (!CONSISTENT_MAPPING)
	{
		// Unmap pages
		dma_unmap_page(idma_dev->miscdev.parent, idma_dev->rd_dmaptr[idx_r], cnt, DMA_TO_DEVICE);
		dma_unmap_page(idma_dev->miscdev.parent, idma_dev->wr_dmaptr[idx_w], cnt, DMA_FROM_DEVICE);
		unpin_user_pages_dirty_lock(page_ro, 1, false);
		unpin_user_pages_dirty_lock(page_wr, 1, true);
	}

	else
	{
		// Copy back data to user page
		copy_to_user((void*)PAGE_ALIGN((u64)buf + 1), idma_dev->wr_vptr[idx_w], SZ_4K);
		// Clear destination buffer
		memset(idma_dev->wr_vptr[idx_w], 0, SZ_4K);
	}

	return 0;
}

static const struct file_operations idma_fops = {
	.owner		= THIS_MODULE,
	.open		= idma_open,
	.release	= idma_release,
	.read		= idma_read,
	.write		= idma_write,
};

#if (ENABLE_IRQ == 1)

static irqreturn_t idma_rd_irq_check(int irq, void *data)
{
	struct idma_device *idma_dev = (struct idma_device *)data;
	u64 ipsr = readq(idma_dev->reg + IDMA_IPSR_REG_OFFSET);

	if (ipsr & IDMA_IPSR_RIP)
	{
		// Capture read stamp
		idma_dev->kt_rd		= ktime_get_ns();
		return IRQ_WAKE_THREAD;
	}

	return IRQ_NONE;
}

static irqreturn_t idma_rd_process(int irq, void *data)
{
	// A read transaction has finished. Capture read time
	struct idma_device *idma_dev = (struct idma_device *)data;
	u64 ipsr = readq(idma_dev->reg + IDMA_IPSR_REG_OFFSET);
	
	// Clear interrupt
	writeq(ipsr & IDMA_IPSR_RIP, idma_dev->reg + IDMA_IPSR_REG_OFFSET);

	return IRQ_HANDLED;
}

static irqreturn_t idma_wr_irq_check(int irq, void *data)
{
	struct idma_device *idma_dev = (struct idma_device *)data;
	u64 ipsr = readq(idma_dev->reg + IDMA_IPSR_REG_OFFSET);

	if (ipsr & IDMA_IPSR_WIP)
	{
		idma_dev->kt_wr		= ktime_get_ns();
		return IRQ_WAKE_THREAD;
	}
	return IRQ_NONE;
}

static irqreturn_t idma_wr_process(int irq, void *data)
{
	// Write transaction has finished. Capture write time
	struct idma_device *idma_dev = (struct idma_device *)data;
	u64 ipsr = readq(idma_dev->reg + IDMA_IPSR_REG_OFFSET);

	// Clear interrupt
	writeq(ipsr & IDMA_IPSR_WIP, idma_dev->reg + IDMA_IPSR_REG_OFFSET);

	return IRQ_HANDLED;
}

#endif

static int idma_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;			// parent device same as platform parent
	struct idma_device *idma_dev = NULL;		// idma representation
	struct resource *res = NULL;
	int ret = 0;
	int irq = 0;
	char idma_name[10];

	// Allocate memory for the idma device
	idma_dev = devm_kzalloc(dev, sizeof(*idma_dev), GFP_KERNEL);
	if (!idma_dev)
		return -ENOMEM;

	// Set name of iDMA instance
	sprintf(idma_name,"idma%d",idma_n);

	// Configure idma dev representation
	idma_dev->miscdev.fops = &idma_fops;
	idma_dev->miscdev.parent = dev;
	idma_dev->miscdev.minor = MISC_DYNAMIC_MINOR;
	idma_dev->miscdev.name = idma_name;
	refcount_set(&idma_dev->ref, 1);
	mutex_init(&idma_dev->lock);

	// Map regmap to kernel memory
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "iDMA: Could not find resource for register region\n");
		return -EINVAL;
	}

	idma_dev->reg = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(idma_dev->reg)) {
		ret = dev_err_probe(dev, PTR_ERR(idma_dev->reg), "iDMA: Could not map register region\n");
		goto fail;
	};

	if (dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64))) {
		dev_err(dev, "iDMA: No suitable DMA available\n");
		goto fail;
	}

	// Register misc device
	ret = misc_register(&idma_dev->miscdev);
	if (ret < 0) {
		dev_err(dev, "iDMA: Could not register misc device\n");
		goto fail;
	}

	// We need to save the idma_dev ptr somewhere
	dev_set_drvdata(&pdev->dev, idma_dev);

	#if (ENABLE_IRQ == 1)

	/* Parse IRQ assignment */
	// RD
	// irq = platform_get_irq_byname_optional(pdev, "rd");
	// if (irq > 0)
	// 	dev_info(dev, "Read completion IRQ: %d\n", irq);

	// else {
	// 	dev_err(dev, "no IRQ provided for read completions\n");
	// 	goto fail;
	// }

	// if (request_threaded_irq(irq, idma_rd_irq_check, idma_rd_process, IRQF_ONESHOT | IRQF_SHARED,
	// 			 dev_name(dev), idma_dev)) {
	// 	dev_err(dev, "fail to request irq %d for iDMA read completion\n", irq);
	// 	goto fail;
	// }

	// WR
	irq = platform_get_irq_byname_optional(pdev, "wr");
	if (irq > 0)
	{
		dev_info(dev, "Write Completion IRQ: %d\n", irq);

		if (request_threaded_irq(irq, idma_wr_irq_check, idma_wr_process, IRQF_ONESHOT | IRQF_SHARED,
				 dev_name(dev), idma_dev)) 
		{
			dev_err(dev, "fail to request irq %d for iDMA write completions\n", irq);
			goto fail;
		}
	}

	else {
		dev_err(dev, "no IRQ provided for the write completions\n");
		goto fail;
	}

	#endif

	if (CONSISTENT_MAPPING)
	{
		for (int i = 0; i < N_MAPPINGS; i++)
		{
			// Allocate a page to read from
			idma_dev->rd_vptr[i] = dma_alloc_coherent(dev, SZ_4K, &(idma_dev->rd_dmaptr[i]), GFP_KERNEL);
			// Allocate a page to write to
			idma_dev->wr_vptr[i] = dma_alloc_coherent(dev, SZ_4K, &(idma_dev->wr_dmaptr[i]), GFP_KERNEL);

			if (!(idma_dev->rd_vptr[i]) | !(idma_dev->wr_vptr[i]))
				return -ENOMEM;
		}
	}

	dev_info(dev, "Registering iDMA%d device\n", idma_n);
	idma_n++;

	return 0;

 fail:
	/* Node idma_dev->reg will be unmapped devres_release_all so we don't unmap it here */
 	if (idma_dev)
 		kfree(idma_dev);
	return ret;
};

// A platform device is usually hardwired and is never removed
static int idma_remove(struct platform_device *pdev)
{
	struct idma_device *idma_dev = NULL;

	idma_dev = (struct idma_device*)dev_get_drvdata(&pdev->dev);
	idma_n--;

	if (CONSISTENT_MAPPING)
	{
		for (int i = 0; i < N_MAPPINGS; i++)
		{
			// Free allocated read page
			dma_free_coherent(&pdev->dev, SZ_4K, idma_dev->rd_vptr[i], idma_dev->rd_dmaptr[i]);
			// Free allocated write page
			dma_free_coherent(&pdev->dev, SZ_4K, idma_dev->wr_vptr[i], idma_dev->wr_dmaptr[i]);
		}
	}

	return 0;
};

static void idma_shutdown(struct platform_device *pdev)
{
	return;
};


static const struct of_device_id idma_of_match[] = {
	{ .compatible = "pulp,idma", },
	{ },
};
MODULE_DEVICE_TABLE(of, idma_of_match);


static struct platform_driver idma_driver = {
	.driver	= {
		.name			= "zdl,idma",
		.of_match_table		= idma_of_match,
		.suppress_bind_attrs	= true,
	},
	.probe	= idma_probe,
	.remove	= idma_remove,
	.shutdown = idma_shutdown,
};
module_driver(idma_driver, platform_driver_register,
	      platform_driver_unregister);