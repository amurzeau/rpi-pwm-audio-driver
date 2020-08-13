#ifndef BCM2835_PWM_AUD_H
#define BCM2835_PWM_AUD_H

#include <linux/compiler.h> /* __iomem */
#include <linux/dmaengine.h>
#include <linux/atomic.h>
#include <linux/mutex.h>
#include "filter.h"

#define PWM_CLK_RATE 25000001
#define PWM_RANGE 125

struct platform_device;
struct clk;
struct resource;

typedef void (*period_callback_t)(void* arg);

struct bcm2835_pwm_aud_t {
	struct platform_device *pdev;
	struct clk* clk;
	int clk_enabled;
	struct resource *pwm_resource;
	void __iomem * pwm_reg_base;
	void __iomem * dma3_cs_reg;
	
	struct dma_chan* dma_channel;
	int dma_period_sample_count;
	int dma_buffer_sample_count;
	int dma_sample_size;
	int range;
	
	atomic_t running; // concurrent
	atomic_t volume; // concurrent
	int paused;
	
	dma_addr_t pwm_bus_base;
	dma_addr_t dma_bus_src;
	void* dma_virt_src;
	struct dma_async_tx_descriptor *tx;
	
	period_callback_t callback;
	void* callback_arg;
	
	// resampling 48khz to 400khz
	bool enable_resampling;
	struct filter_fir_t filter;
	int pos;
	int pos_offset;
	int paused_pos;
};

int bcm2835_pwm_aud_init(struct bcm2835_pwm_aud_t* chip, struct platform_device* pdev);
void bcm2835_pwm_aud_free(struct bcm2835_pwm_aud_t* chip);
int bcm2835_pwm_aud_configure(struct bcm2835_pwm_aud_t* chip, int dma_period_sample_count, int dma_period_count, period_callback_t callback, void* callback_arg, bool enable_resampling);
int bcm2835_pwm_aud_unconfigure(struct bcm2835_pwm_aud_t* chip);
int bcm2835_pwm_aud_enable(struct bcm2835_pwm_aud_t* chip, int enable);
int bcm2835_pwm_aud_pause(struct bcm2835_pwm_aud_t* chip, int pause);
void bcm2835_pwm_aud_reset_pos(struct bcm2835_pwm_aud_t* chip);
int bcm2835_pwm_aud_write(struct bcm2835_pwm_aud_t* chip, void* data, int size_sample);
int bcm2835_pwm_aud_pointer(struct bcm2835_pwm_aud_t* chip);

#endif
