/*****************************************************************************
* Copyright 2011 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL").
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/

#define DEBUG

#include <linux/platform_device.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/dmaengine.h>
#include <linux/dmapool.h>
#include <linux/clk.h>

#include <sound/core.h>
#include <sound/dmaengine_pcm.h>
#include <linux/dma-mapping.h>

#include "bcm2835-pwm-aud.h"

MODULE_AUTHOR("Alexis Murzeau <amubtdx@gmail.com>");
MODULE_DESCRIPTION("snd-bcm2835-pwm");
MODULE_LICENSE("GPL");

#define PWM_CHANNEL_NUM	2
#define PWM_FIFO_SIZE 16 /* 16 * 32bits ints */

/* Registers */
#define PWM_REG_CONTROL		0x00
#define PWM_REG_STATUS		0x04
#define PWM_REG_DMAC		0x08
#define PWM_REG_RANGE(channel)	(((channel) * 0x10) + 0x10) /* period */
#define PWM_REG_DATA(channel)	(((channel) * 0x10) + 0x14) /* duty cycle */
#define PWM_REG_FIFO		0x18

/* Control flags */
#define PWM_CTRL_ENABLE	(1 << 0)
#define PWM_CTRL_MODE	(1 << 1)
#define PWM_CTRL_REPEAT	(1 << 2)
#define PWM_CTRL_SILENCE	(1 << 3)
#define PWM_CTRL_POLARITY	(1 << 4)
#define PWM_CTRL_USE_FIFO	(1 << 5)
#define PWM_CTRL_CLEAR_FIFO	(1 << 6)
#define PWM_CTRL_MS_TRANSMISSION	(1 << 7)
#define PWM_CTRL_FLAGS(channel, flags)	((flags) << ((channel) * 8))

/* Status flags */
#define PWM_STATUS_FIFO_FULL (1 << 0)
#define PWM_STATUS_FIFO_EMPTY (1 << 1)
#define PWM_STATUS_FIFO_WRITE_ERR (1 << 2)
#define PWM_STATUS_FIFO_READ_ERR (1 << 3)
#define PWM_STATUS_FIFO_GAP_OCC(channel) (1 << (4 + (channel)))
#define PWM_STATUS_FIFO_BUS_ERR (1 << 8)
#define PWM_STATUS_FIFO_STATE(channel) (1 << (9 + (channel)))

#define PWM_STATUS_CLEAR_ALL 0x0FFC

/* DMA Control */
#define PWM_DMAC_DREQ(dreq_threshold) ((dreq_threshold) << 0)
#define PWM_DMAC_PANIC(panic_threshold) ((panic_threshold) << 8)
#define PWM_DMAC_ENABLE (1 << 31)

/* Threshold validation */
#define PWM_DMAC_THR_VALIDATE(dreq) (((dreq) & ~0xFF) == 0)

/* Valid only for channels 0 - 14, 15 has its own base address */
#define BCM2835_DMA_CHAN(n)	((n) << 8) /* Base address */
#define BCM2835_DMA_CHANIO(base, n) ((base) + BCM2835_DMA_CHAN(n))


struct pwm_sample {
	u32 left;
	u32 right;
};

struct audio_sample {
	s16 left;
	s16 right;
};

static int bcm2835_pwm_aud_pointer_internal(struct bcm2835_pwm_aud_t* chip);

static void pwm_reg_write(struct bcm2835_pwm_aud_t* chip, int value, int reg) {
	writel(value, chip->pwm_reg_base + reg);
}

static int pwm_reg_read(struct bcm2835_pwm_aud_t* chip, int reg) {
	return readl(chip->pwm_reg_base + reg);
}

static void initialize_dma_buffer(struct pwm_sample* output, int sample_count) {
	int i;
	
	for(i = 0; i < sample_count; i++) {
		output[i].left = PWM_RANGE/2;
		output[i].right = PWM_RANGE/2;
	}
}

static void dma_callback(void *arg) {
	struct bcm2835_pwm_aud_t* chip = (struct bcm2835_pwm_aud_t*) arg;
	period_callback_t callback;
	void* callback_arg;

	if(atomic_read(&chip->running)) {
		callback = READ_ONCE(chip->callback);
		callback_arg = READ_ONCE(chip->callback_arg);
		if(callback)
			(*callback)(callback_arg);
	}
}

static int configure_dma(struct bcm2835_pwm_aud_t* chip) {
	int ret;
	struct dma_slave_config config;
	int dma_buffer_size = chip->dma_buffer_sample_count * sizeof(struct pwm_sample);
	int dma_period_size = chip->dma_period_sample_count * sizeof(struct pwm_sample);

	chip->dma_virt_src = NULL;
	chip->tx = NULL;
	
	dev_dbg(&chip->pdev->dev, "Allocating %d bytes for DMA source\n", dma_buffer_size);
	chip->dma_virt_src = dma_alloc_wc(&chip->pdev->dev, dma_buffer_size, &chip->dma_bus_src, GFP_KERNEL);
	if (unlikely(!chip->dma_virt_src)) {
		ret = -ENOMEM;
		dev_err(&chip->pdev->dev, "Failed to allocate dma source memory with size %d\n", dma_buffer_size);
		goto err;
	}
	
	dev_dbg(&chip->pdev->dev, "DMA source buffer virtual address: %p, physical address: %x, bus address: %x\n",
			chip->dma_virt_src, virt_to_phys(chip->dma_virt_src), chip->dma_bus_src);

	initialize_dma_buffer(chip->dma_virt_src, chip->dma_buffer_sample_count);
	chip->pos = 0;

	memset(&config, 0, sizeof(config));
	config.direction = DMA_MEM_TO_DEV;
	config.dst_addr = chip->pwm_bus_base + PWM_REG_FIFO;
	config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	config.src_maxburst = PWM_FIFO_SIZE / 2;
	config.dst_maxburst = PWM_FIFO_SIZE / 2;
	config.device_fc = true;
	config.slave_id = 0;

	dev_dbg(&chip->pdev->dev, "Configuring DMA\n");
	ret = dmaengine_slave_config(chip->dma_channel, &config);
	if(ret) {
		dev_err(&chip->pdev->dev, "Failed to configure dma: %d\n", ret);
		goto err;
	}
	
	dev_dbg(&chip->pdev->dev, "Preparing DMA from buffer at bus address %x to peripheral address %x\n",
			chip->dma_bus_src, config.dst_addr);
	
	chip->tx = dmaengine_prep_dma_cyclic(chip->dma_channel,
							  chip->dma_bus_src,
							  dma_buffer_size,
							  dma_period_size,
							  DMA_MEM_TO_DEV,
							  DMA_CTRL_ACK | DMA_PREP_INTERRUPT);

	if(IS_ERR(chip->tx)) {
		ret = PTR_ERR(chip->tx);
		chip->tx = NULL;
		dev_err(&chip->pdev->dev, "Failed to prepare dma: %d\n", ret);
		goto err;
	}

	chip->tx->callback = dma_callback;
	chip->tx->callback_param = chip;

	ret = dmaengine_submit(chip->tx);
	if(ret < 0) {
		dev_err(&chip->pdev->dev, "Failed to submit dma: %d\n", ret);
		goto err;
	}

	dma_async_issue_pending(chip->dma_channel);

	return 0;
	
err:
	if(chip->tx) {
		dmaengine_terminate_sync(chip->dma_channel);
		chip->tx = NULL;
	}
	
	if(chip->dma_virt_src) {
		dma_free_wc(&chip->pdev->dev, dma_buffer_size, chip->dma_virt_src, chip->dma_bus_src);
		chip->dma_virt_src = NULL;
	}
	
	return ret;
}

int bcm2835_pwm_aud_init(struct bcm2835_pwm_aud_t* chip, struct platform_device *pdev)
{
	int ret = 0;
	const __be32* pwm_addr;
	
	if(chip == NULL)
		return -EINVAL;
	
	memset(chip, 0, sizeof(*chip));
	chip->pdev = pdev;
	atomic_set(&chip->volume, PWM_RANGE);
	
	chip->pwm_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!chip->pwm_resource) {
		dev_err(&pdev->dev, "no resource, invalid device tree\n");
		ret = -ENODEV;
		goto err;
	}
	chip->pwm_reg_base = devm_ioremap_resource(&pdev->dev, chip->pwm_resource);
	if(IS_ERR(chip->pwm_reg_base)) {
		ret = PTR_ERR(chip->pwm_reg_base);
		chip->pwm_reg_base = NULL;
		dev_err(&pdev->dev, "failed to ioremap pwm base register address %x\n", chip->pwm_resource->start);
		goto err;
	}
	pwm_addr = of_get_address(pdev->dev.of_node, 0, NULL, NULL);
	if (!pwm_addr) {
		ret = -ENODEV;
		dev_err(&pdev->dev, "could not get DMA-register address\n");
		goto err;
	}
	chip->pwm_bus_base = be32_to_cpup(pwm_addr);
	
	dev_dbg(&pdev->dev, "PWM bus address: %x, virtual address: %p\n", chip->pwm_bus_base, chip->pwm_reg_base);
	
	dev_dbg(&pdev->dev, "Retrieving clock\n");
	chip->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(chip->clk)) {
		dev_err(&pdev->dev, "clock not found: %ld\n", PTR_ERR(chip->clk));
		ret = PTR_ERR(chip->clk);
		goto err;
	}
	
	clk_set_rate(chip->clk, PWM_CLK_RATE);

	dev_dbg(&pdev->dev, "Enabling clock\n");
	ret = clk_prepare_enable(chip->clk);
	if (ret) {
		dev_err(&pdev->dev, "Enabling clock error %d\n", ret);
		goto err;
	}
	chip->clk_enabled = true;
	
	dev_dbg(&pdev->dev, "Requesting DMA channel\n");
	chip->dma_channel = dma_request_chan(&pdev->dev, "pwm");
	if (IS_ERR(chip->dma_channel)) {
		ret = PTR_ERR(chip->dma_channel);
		dev_err(&pdev->dev, "DMA request error %d\n", ret);
		goto err;
	}
	dev_dbg(&pdev->dev, "using DMA channel %d\n", chip->dma_channel->chan_id);
	
	return 0;
	
err:
	bcm2835_pwm_aud_free(chip);
	return ret;
}

void bcm2835_pwm_aud_free(struct bcm2835_pwm_aud_t* chip)
{
	if(!chip)
		return;

	dev_dbg(&chip->pdev->dev, "free\n");

	if(chip->dma_channel) {
		bcm2835_pwm_aud_unconfigure(chip);
		
		dev_dbg(&chip->pdev->dev, "releasing dma channel\n");
		dma_release_channel(chip->dma_channel);
		chip->dma_channel = NULL;
	}
	
	if(chip->clk) {
		if(chip->clk_enabled) {
			dev_dbg(&chip->pdev->dev, "disabling clk\n");
			clk_disable_unprepare(chip->clk);
			chip->clk_enabled = false;
		}
		
		dev_dbg(&chip->pdev->dev, "releasing clk\n");
		clk_put(chip->clk);
		chip->clk = NULL;
	}
}

int bcm2835_pwm_aud_configure(struct bcm2835_pwm_aud_t* chip,
							  int dma_period_sample_count,
							  int dma_period_count,
							  period_callback_t callback,
							  void* callback_arg)
{
	int ret = 0;
	
	const int flags = PWM_CTRL_USE_FIFO/* | PWM_CTRL_REPEAT*/;
	const int control = PWM_CTRL_FLAGS(0, flags) |
						PWM_CTRL_FLAGS(1, flags);

	const int status = PWM_STATUS_CLEAR_ALL;
	
	const int dmac = PWM_DMAC_DREQ(14) | PWM_DMAC_PANIC(7);
	
	const int range = PWM_RANGE;
	
	chip->dma_period_sample_count = dma_period_sample_count;
	chip->dma_sample_size = sizeof(struct pwm_sample);
	chip->dma_buffer_sample_count = chip->dma_period_sample_count * dma_period_count;
	chip->callback = callback;
	chip->callback_arg = callback_arg;
	
	bcm2835_pwm_aud_unconfigure(chip);
	
	dev_dbg(&chip->pdev->dev, "configuring pwm, period sample count: %d, period count: %d\n", dma_period_sample_count, dma_period_count);
	
	ret = configure_dma(chip);
	if(ret) {
		bcm2835_pwm_aud_unconfigure(chip);
		return ret;
	}
	
	pwm_reg_write(chip, control, PWM_REG_CONTROL);
	pwm_reg_write(chip, status, PWM_REG_STATUS);
	pwm_reg_write(chip, dmac, PWM_REG_DMAC);
	pwm_reg_write(chip, range, PWM_REG_RANGE(0));
	pwm_reg_write(chip, range, PWM_REG_RANGE(1));

	return 0;
}

int bcm2835_pwm_aud_unconfigure(struct bcm2835_pwm_aud_t* chip)
{
	int ret;
	
	if(!chip || !chip->dma_channel)
		return 0;
	
	if(chip->tx) {
			bcm2835_pwm_aud_pause(chip, false);
			dev_dbg(&chip->pdev->dev, "terminating dma\n");
			dmaengine_pause(chip->dma_channel);
			ret = dmaengine_terminate_sync(chip->dma_channel);
			if(ret)
				dev_err(&chip->pdev->dev, "failed to terminate dma: %d\n", ret);
			chip->tx = NULL;
	}
	if(chip->dma_virt_src) {
		dev_dbg(&chip->pdev->dev, "releasing dma memory\n");
		dma_free_wc(&chip->pdev->dev, chip->dma_buffer_sample_count * sizeof(struct pwm_sample), chip->dma_virt_src, chip->dma_bus_src);
		chip->dma_virt_src = NULL;
		chip->dma_bus_src = 0;
	}
	
	/* PWM must be disabled after having stopped dma
	 * else dma won't flush itself and be left in a bad state
	 * (next dma on this channel will stop because NEXT_CB is 0x0)
	 */
	bcm2835_pwm_aud_enable(chip, 0);
	
	return 0;
}

int bcm2835_pwm_aud_enable(struct bcm2835_pwm_aud_t* chip, int enable) {
	const int control = pwm_reg_read(chip, PWM_REG_CONTROL);
	
	if(enable) {
		pwm_reg_write(chip, control | PWM_CTRL_CLEAR_FIFO, PWM_REG_CONTROL);
		pwm_reg_write(chip, control | PWM_CTRL_FLAGS(0, PWM_CTRL_ENABLE) | PWM_CTRL_FLAGS(1, PWM_CTRL_ENABLE), PWM_REG_CONTROL);
	} else {
		bcm2835_pwm_aud_pause(chip, true);
		pwm_reg_write(chip, control & (~(PWM_CTRL_FLAGS(0, PWM_CTRL_ENABLE) | PWM_CTRL_FLAGS(1, PWM_CTRL_ENABLE))), PWM_REG_CONTROL);
	}
	
	return 0;
}

int bcm2835_pwm_aud_pause(struct bcm2835_pwm_aud_t* chip, int pause) {
	const int dmac = pwm_reg_read(chip, PWM_REG_DMAC);
	const int control = pwm_reg_read(chip, PWM_REG_CONTROL);
	const int repeat_flags = PWM_CTRL_FLAGS(0, PWM_CTRL_REPEAT) | PWM_CTRL_FLAGS(1, PWM_CTRL_REPEAT);
	
	/* Enable repeat to avoid a big pop when output goes to DC 0V. */
	
	if(pause) {
		pwm_reg_write(chip, control | repeat_flags, PWM_REG_CONTROL);
		pwm_reg_write(chip, dmac & (~PWM_DMAC_ENABLE), PWM_REG_DMAC);
	} else {
		pwm_reg_write(chip, dmac | PWM_DMAC_ENABLE, PWM_REG_DMAC);
		pwm_reg_write(chip, (control & (~repeat_flags)), PWM_REG_CONTROL);
	}
	
	return 0;
}

void bcm2835_pwm_aud_reset_pos(struct bcm2835_pwm_aud_t* chip) {
	chip->pos = bcm2835_pwm_aud_pointer_internal(chip);
}

int bcm2835_pwm_aud_write(struct bcm2835_pwm_aud_t* chip, void* data, int size_sample) {
	struct audio_sample* input = (struct audio_sample*) data;
	struct pwm_sample* output = ((struct pwm_sample*) (chip->dma_virt_src));
	int volume = atomic_read(&chip->volume);
	
	int i;
	int dma_pos;
	dma_pos = chip->pos;
	
	for(i = 0; i < size_sample; i++) {
		output[dma_pos].left = (unsigned int)(input[i].left * volume + (PWM_RANGE * 65536 / 2)) / 65536;
		output[dma_pos].right = (unsigned int)(input[i].right * volume + (PWM_RANGE * 65536 / 2)) / 65536;
		dma_pos++;
		if(dma_pos >= chip->dma_buffer_sample_count)
			dma_pos -= chip->dma_buffer_sample_count;
	}
	chip->pos = dma_pos;

	return 0;
}

static int bcm2835_pwm_aud_pointer_internal(struct bcm2835_pwm_aud_t* chip) {
	struct dma_tx_state state;
	enum dma_status status;
	int dma_pos = 0;
	
	if(!chip->dma_channel || !chip->tx) {
		if(!chip->dma_channel)
			dev_err(&chip->pdev->dev, "pointer called without dma channel\n");
		else
			dev_err(&chip->pdev->dev, "pointer called without tx\n");
		return -ENOENT;
	}
	
	status = dmaengine_tx_status(chip->dma_channel, chip->tx->cookie, &state);
	
	switch(status) {
		case DMA_COMPLETE:
			dma_pos = chip->dma_buffer_sample_count;
			break;
		case DMA_IN_PROGRESS:
		case DMA_PAUSED:
			dma_pos = chip->dma_buffer_sample_count - state.residue / sizeof(struct pwm_sample);
			break;
		case DMA_ERROR:
			dev_err(&chip->pdev->dev, "dma error detected, no pointer\n");
			break;
		default:
			dev_err(&chip->pdev->dev, "unhandled dma status %d\n", status);
			break;
	}
	
	if(dma_pos == chip->dma_buffer_sample_count)
		return 0;
	
	return dma_pos;
}

int bcm2835_pwm_aud_pointer(struct bcm2835_pwm_aud_t* chip) {
	int pos = bcm2835_pwm_aud_pointer_internal(chip);

	return pos;
}
