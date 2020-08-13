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
#include "bcm2835-pwm-aud.h"

#include <sound/core.h>
#include <sound/asound.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/pcm-indirect.h>
#include <linux/mutex.h>
#include <linux/device.h>

#include <linux/slab.h>

MODULE_AUTHOR("Alexis Murzeau <amubtdx@gmail.com>");
MODULE_DESCRIPTION("snd-bcm2835-pwm");
MODULE_LICENSE("GPL");

#define audio_error(_chip, fmt, arg...) dev_err(&((_chip)->chip.pdev->dev), "%s:%d " fmt, __func__, __LINE__, ## arg)
#define audio_info(_chip, fmt, arg...) dev_dbg(&((_chip)->chip.pdev->dev), "%s:%d " fmt , __func__, __LINE__, ## arg)

#define CHANNEL_NUM	2
#define MAX_SUBSTREAMS 1
#define SAMPLING_RATE (PWM_CLK_RATE/PWM_RANGE)

static int snd_bcm2835_pwm_pcm_ack(struct snd_pcm_substream *substream);
static snd_pcm_uframes_t snd_bcm2835_pwm_pcm_pointer(struct snd_pcm_substream *substream);

struct bcm2835_alsa_chip;

typedef struct bcm2835_alsa_stream {
	struct bcm2835_alsa_chip* chip;
	struct snd_pcm_substream *substream;
	struct snd_pcm_indirect pcm_indirect;

	struct semaphore buffers_update_sem;
	struct semaphore control_sem;
	spinlock_t lock;
	volatile uint32_t control;
	volatile uint32_t status;
	
	int pos;

	bool configured;
	bool use_resampling;
} bcm2835_alsa_stream_t;

typedef struct bcm2835_alsa_chip {
	struct bcm2835_pwm_aud_t chip;
	struct snd_card *card;
	struct snd_pcm *pcm;
	bcm2835_alsa_stream_t *alsa_stream;
	
	bool opened;
	
	struct mutex audio_mutex;
} bcm2835_alsa_chip_t;

static struct bcm2835_alsa_chip * g_chip;

static const struct snd_pcm_hardware bcm2835_pwm_pcm_hardware = {
	.info		= SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U32_LE,
	.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_KNOT,
	.rate_min = 48000,
	.rate_max = SAMPLING_RATE,
	.channels_min = 2,
	.channels_max = 2,
	.period_bytes_min	= 128,
	.period_bytes_max	= 32768,
	.periods_min		= 2,
	.periods_max		= 128,
	.buffer_bytes_max	= 1024 * 1024,
	.fifo_size		= 32,
};

/* The actual rates supported by the card. */
static unsigned int samplerates[] = {
	48000, SAMPLING_RATE,
};
static struct snd_pcm_hw_constraint_list constraints_rates = {
	.count = ARRAY_SIZE(samplerates), 
	.list = samplerates,
	.mask = 0,
};



static void snd_bcm2835_playback_free(struct snd_pcm_runtime *runtime)
{
	bcm2835_alsa_stream_t *alsa_stream = (bcm2835_alsa_stream_t *) runtime->private_data;
	audio_info(alsa_stream->chip, "Freeing up alsa stream here ..\n");
	
	if (alsa_stream) {
		if(alsa_stream->chip->alsa_stream == alsa_stream)
			alsa_stream->chip->alsa_stream = NULL;
		kfree(alsa_stream);
		runtime->private_data = NULL;
	}
}

static void dma_callback(void *arg) {
	bcm2835_alsa_stream_t *alsa_stream = (bcm2835_alsa_stream_t *) arg;
	//static ktime_t previous;

	if (alsa_stream->substream) {
		//ktime_t start, end;
		//int period_size = alsa_stream->substream->runtime->period_size;
		//int buffer_size = alsa_stream->substream->runtime->buffer_size;
		
		//start = ktime_get();
		
		snd_pcm_period_elapsed(alsa_stream->substream);

		//write_data(alsa_stream, period_size, buffer_size);
		//end = ktime_get();
		//trace_printk("dma_callback %lld ns out, %lld ns in, pos %d\n", ktime_to_ns(ktime_sub(start, previous)), ktime_to_ns(ktime_sub(end, start)), bcm2835_pwm_aud_pointer(&alsa_stream->chip->chip));
		//previous = start;
		//audio_info(alsa_stream->chip, "dma_callback\n");
	}
}

/* open callback */
static int snd_bcm2835_playback_open(
		struct snd_pcm_substream *substream)
{
	bcm2835_alsa_chip_t *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	bcm2835_alsa_stream_t *alsa_stream;
	int err = 0;
	
	if(chip == NULL) {
		printk("Bad chip: %p %p, %p, %d, %.32s\n", chip, substream->private_data, substream->pcm->private_data, substream->ref_count, substream->name);
		chip = substream->private_data = substream->pcm->private_data;
	}

	audio_info(chip, " .. IN (%d)\n", substream->number);

	if(mutex_lock_interruptible(&chip->audio_mutex)) {
		audio_error(chip, "Interrupted whilst waiting for lock\n");
		return -EINTR;
	}
	audio_info(chip, "Alsa open (%d)\n", substream->number);

	if (chip->opened) {
		err = -EBUSY;
		goto out;
	}

	alsa_stream = kzalloc(sizeof(bcm2835_alsa_stream_t), GFP_KERNEL);
	if (alsa_stream == NULL) {
		err = -ENOMEM;
		goto out;
	}

	/* Initialise alsa_stream */
	alsa_stream->chip = chip;
	alsa_stream->substream = substream;

	sema_init(&alsa_stream->buffers_update_sem, 0);
	sema_init(&alsa_stream->control_sem, 0);
	spin_lock_init(&alsa_stream->lock);

	runtime->private_data = alsa_stream;
	runtime->private_free = snd_bcm2835_playback_free;
	runtime->hw = bcm2835_pwm_pcm_hardware;
	
	snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_SIZE, 3);
	snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_BUFFER_SIZE, 3);
	snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE, &constraints_rates);

	chip->alsa_stream = alsa_stream;

	chip->opened = true;

out:
	mutex_unlock(&chip->audio_mutex);

	audio_info(chip, " .. OUT =%d\n", err);

	return err;
}

/* close callback */
static int snd_bcm2835_playback_close(struct snd_pcm_substream *substream)
{
	bcm2835_alsa_chip_t *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime;
	bcm2835_alsa_stream_t *alsa_stream;

	audio_info(chip, " .. IN\n");

	if(mutex_lock_interruptible(&chip->audio_mutex))
	{
		audio_error(chip, "Interrupted whilst waiting for lock\n");
		return -EINTR;
	}
	runtime = substream->runtime;
	alsa_stream = runtime->private_data;

	audio_info(chip, "Alsa close\n");

	/*
	 * Call stop if it's still running. This happens when app
	 * is force killed and we don't get a stop trigger.
	 */
	if (alsa_stream->configured) {
		int err;
		atomic_set(&chip->chip.running, 0);
		snd_pcm_set_runtime_buffer(substream, NULL);
		err = bcm2835_pwm_aud_unconfigure(&alsa_stream->chip->chip);
		alsa_stream->configured = false;
		if (err != 0)
			audio_error(chip, " Failed to STOP alsa device\n");
	}

	if (alsa_stream->chip)
		alsa_stream->chip->alsa_stream = NULL;
	/*
	 * Do not free up alsa_stream here, it will be freed up by
	 * runtime->private_free callback we registered in *_open above
	 */

	chip->opened = false;

	mutex_unlock(&chip->audio_mutex);
	audio_info(chip, " .. OUT\n");

	return 0;
}

/* hw_params callback */
static int snd_bcm2835_pwm_pcm_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	bcm2835_alsa_chip_t *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	bcm2835_alsa_stream_t *alsa_stream = runtime->private_data;
	int error = 0;

	audio_info(chip, " .. IN\n");
	if (mutex_lock_interruptible(&chip->audio_mutex)) {
		error = -EINTR;
		goto err;
	}
	
	if(params_buffer_size(params) != params_period_size(params) * params_periods(params)) {
		audio_error(chip, "Buffer size %d is not multiple of period size %d\n", params_buffer_size(params), params_period_size(params));
		error = -EINVAL;
		goto err;
	}
	
	if(params_rate(params) == SAMPLING_RATE && params_format(params) != SNDRV_PCM_FORMAT_U32_LE) {
		audio_error(chip, "When using 400kHz rate, format must be U32_LE but is %d\n", params_format(params));
		error = -EINVAL;
		goto err;
	} else if(params_rate(params) != SAMPLING_RATE && params_format(params) != SNDRV_PCM_FORMAT_S16_LE) {
		audio_error(chip, "When using 48kHz rate, format must be S16_LE but is %d\n", params_format(params));
		error = -EINVAL;
		goto err;
	}
	
	alsa_stream->use_resampling = params_rate(params) != SAMPLING_RATE;
	
	error = bcm2835_pwm_aud_configure(&alsa_stream->chip->chip,
								   params_period_size(params),
								   params_periods(params),
								   dma_callback,
								   alsa_stream,
								   alsa_stream->use_resampling
 									);
	if(error) {
		audio_error(chip, "Failed to configure: %d\n", error);
		goto err;
	}
	
	memset(&alsa_stream->pcm_indirect, 0, sizeof(alsa_stream->pcm_indirect));
	
	if(alsa_stream->use_resampling) {
		error = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
		if (error < 0) {
			audio_error
				(chip, " pcm_lib_malloc failed to allocate pages for buffers\n");
			goto err;
		}
	
		memset(runtime->dma_area, 0, runtime->dma_bytes);

		
		alsa_stream->pcm_indirect.sw_buffer_size = runtime->dma_bytes;
		alsa_stream->pcm_indirect.hw_buffer_size = params_period_bytes(params) * params_periods(params);
		
		audio_info(chip, "pcm indirect sw size: %u, hw size: %u\n", alsa_stream->pcm_indirect.sw_buffer_size, alsa_stream->pcm_indirect.hw_buffer_size);
	} else {
		substream->dma_buffer.addr = alsa_stream->chip->chip.dma_bus_src;
		substream->dma_buffer.area = alsa_stream->chip->chip.dma_virt_src;
		substream->dma_buffer.bytes = alsa_stream->chip->chip.dma_buffer_sample_count * alsa_stream->chip->chip.dma_sample_size;
		substream->dma_buffer.dev.type = SNDRV_DMA_TYPE_CONTINUOUS;
		substream->dma_buffer.dev.dev = NULL;
		snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
		
		audio_info(chip, "pcm direct dma size: %u\n", substream->dma_buffer.bytes);
	}
	
	
	if(runtime->dma_bytes != params_period_size(params) * params_periods(params) * 8) {
		audio_error
		    (chip, "bad dma area size: %d, expected %d\n", runtime->dma_bytes, params_period_size(params) * params_periods(params) * 8);
	}
		
		
	
	bcm2835_pwm_aud_enable(&chip->chip, true);
	
	alsa_stream->configured = true;
	
err:
	mutex_unlock(&chip->audio_mutex);
	audio_info(chip, " .. OUT\n");

	return error;
}

/* hw_free callback */
static int snd_bcm2835_pwm_pcm_hw_free(struct snd_pcm_substream *substream)
{
	bcm2835_alsa_chip_t *chip = snd_pcm_substream_chip(substream);
	audio_info(chip, " .. IN\n");
	atomic_set(&chip->chip.running, 0);
	audio_info(chip, " .. OUT\n");
	return snd_pcm_lib_free_pages(substream);
}

static void dump_pcm_indirect(bcm2835_alsa_chip_t *chip, struct snd_pcm_indirect *rec) {
	audio_info(chip, "PCM indirect:\n");
	audio_info(chip, "  hw_buffer_size = %u\n", rec->hw_buffer_size);
	audio_info(chip, "  hw_queue_size = %u\n", rec->hw_queue_size);
	audio_info(chip, "  hw_data = %u\n", rec->hw_data);
	audio_info(chip, "  hw_io = %u\n", rec->hw_io);
	audio_info(chip, "  hw_ready = %d\n", rec->hw_ready);
	audio_info(chip, "  sw_buffer_size = %u\n", rec->sw_buffer_size);
	audio_info(chip, "  sw_data = %u\n", rec->sw_data);
	audio_info(chip, "  sw_io = %u\n", rec->sw_io);
	audio_info(chip, "  sw_ready = %d\n", rec->sw_ready);
	audio_info(chip, "  appl_ptr = %lu\n", rec->appl_ptr);
	audio_info(chip, "  runtime appl_ptr = %lu\n", chip->alsa_stream->substream->runtime->control->appl_ptr);
	audio_info(chip, "  runtime hw_ptr = %lu\n", chip->alsa_stream->substream->runtime->status->hw_ptr);
}

/* prepare callback */
static int snd_bcm2835_pwm_pcm_prepare(struct snd_pcm_substream *substream)
{
	bcm2835_alsa_chip_t *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	bcm2835_alsa_stream_t *alsa_stream = runtime->private_data;

	//audio_info(chip, " .. IN\n");

	if (mutex_lock_interruptible(&chip->audio_mutex))
		return -EINTR;
	
	alsa_stream->pos = 0;
	
	if(alsa_stream->use_resampling) {
		snd_bcm2835_pwm_pcm_pointer(substream);
		alsa_stream->pcm_indirect.appl_ptr = 0;
		alsa_stream->pcm_indirect.sw_data = 0;
		alsa_stream->pcm_indirect.sw_io = 0;
		alsa_stream->pcm_indirect.sw_ready = 0;
		alsa_stream->pcm_indirect.hw_data = alsa_stream->pcm_indirect.hw_io;
		dump_pcm_indirect(chip, &alsa_stream->pcm_indirect);
		bcm2835_pwm_aud_reset_pos(&chip->chip);
	}

	/* in preparation of the stream, set the controls (volume level) of the stream */
	//bcm2835_audio_set_ctls(alsa_stream->chip);

	mutex_unlock(&chip->audio_mutex);
	//audio_info(chip, " .. OUT\n");
	return 0;
}

/* trigger callback */
static int snd_bcm2835_pwm_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	bcm2835_alsa_chip_t *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	bcm2835_alsa_stream_t *alsa_stream = runtime->private_data;
	int err = 0;

	audio_info(chip, " .. IN (%d), %d\n", cmd, bcm2835_pwm_aud_pointer(&alsa_stream->chip->chip));

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		atomic_set(&chip->chip.running, 1);
		//alsa_stream->pcm_indirect.hw_io =
		//alsa_stream->pcm_indirect.hw_data = frames_to_bytes(runtime, bcm2835_pwm_aud_pointer(&chip->chip));
		//substream->ops->ack(substream);
		//dump_pcm_indirect(chip, &alsa_stream->pcm_indirect);
		bcm2835_pwm_aud_pause(&chip->chip, false);
		//dump_pcm_indirect(chip, &alsa_stream->pcm_indirect);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		//atomic_set(&chip->chip.running, 0);
		//dump_pcm_indirect(chip, &alsa_stream->pcm_indirect);
		bcm2835_pwm_aud_pause(&chip->chip, true);
		break;
	default:
		err = -EINVAL;
	}

	//audio_info(chip, " .. OUT\n");
	return err;
}

/* pointer callback */
static snd_pcm_uframes_t
snd_bcm2835_pwm_pcm_pointer(struct snd_pcm_substream *substream)
{
	bcm2835_alsa_chip_t *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	bcm2835_alsa_stream_t *alsa_stream = runtime->private_data;
	snd_pcm_uframes_t frames;
	//audio_info(chip, " .. IN\n");
	
	int dma_pos = bcm2835_pwm_aud_pointer(&chip->chip);
	if(dma_pos < 0) {
		audio_error(chip, "error getting pointer: %d\n", -dma_pos);
		return 0;
	}
	
	if(alsa_stream->use_resampling) {
		frames = snd_pcm_indirect_playback_pointer(substream, &alsa_stream->pcm_indirect, frames_to_bytes(runtime, dma_pos));
	} else {
		frames = dma_pos;
	}
	//dump_pcm_indirect(alsa_stream->chip, &alsa_stream->pcm_indirect);
	//audio_info(chip, " .. OUT %u\n", dma_pos);
	if(frames == 20475)
		audio_info(chip, " .. OUT %u\n", dma_pos);
	return frames;
}

static void snd_bcm2835_pwm_pcm_transfer(struct snd_pcm_substream *substream,
				    struct snd_pcm_indirect *rec, size_t bytes)
{
	bcm2835_alsa_chip_t *chip = snd_pcm_substream_chip(substream);
	char *src = (void *)(substream->runtime->dma_area + rec->sw_data);
	int err;

	//audio_info(chip, " .. IN %d, %d, %d\n", rec->hw_data, rec->sw_data, bytes);
	err = bcm2835_pwm_aud_write(&chip->chip,
								src,
							 bytes_to_frames(substream->runtime,bytes));
	if(err) {
		audio_error(chip, "failed to write audio data: %d\n", err);
	}
	//audio_info(chip, " .. OUT\n");
}

static int snd_bcm2835_pwm_pcm_ack(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	bcm2835_alsa_stream_t *alsa_stream = runtime->private_data;
	struct snd_pcm_indirect *pcm_indirect = &alsa_stream->pcm_indirect;

	//audio_info(alsa_stream->chip, " .. IN\n");
	if(alsa_stream->use_resampling) {
		snd_pcm_indirect_playback_transfer(substream, pcm_indirect,
						snd_bcm2835_pwm_pcm_transfer);
	}
	//dump_pcm_indirect(alsa_stream->chip, pcm_indirect);
	//audio_info(alsa_stream->chip, " .. OUT\n");
	//__WARN();
	return 0;
}

static int snd_bcm2835_pwm_pcm_lib_ioctl(struct snd_pcm_substream *substream,
				     unsigned int cmd, void *arg)
{
	bcm2835_alsa_chip_t *chip = snd_pcm_substream_chip(substream);
	int ret = snd_pcm_lib_ioctl(substream, cmd, arg);
	audio_info(chip, " .. substream=%p, cmd=%d, arg=%p (%x) ret=%d\n", substream,
		    cmd, arg, arg ? *(unsigned *)arg : 0, ret);
	return ret;
}


/* operators */
static struct snd_pcm_ops snd_bcm2835_playback_ops = {
	.open = snd_bcm2835_playback_open,
	.close = snd_bcm2835_playback_close,
	.ioctl = snd_bcm2835_pwm_pcm_lib_ioctl,
	.hw_params = snd_bcm2835_pwm_pcm_hw_params,
	.hw_free = snd_bcm2835_pwm_pcm_hw_free,
	.prepare = snd_bcm2835_pwm_pcm_prepare,
	.trigger = snd_bcm2835_pwm_pcm_trigger,
	.pointer = snd_bcm2835_pwm_pcm_pointer,
	.ack = snd_bcm2835_pwm_pcm_ack,
};

/* create a pcm device */
static int snd_bcm2835_pwm_new_pcm(bcm2835_alsa_chip_t * chip)
{
	int err = 0;

	audio_info(chip, " .. IN\n");
	mutex_init(&chip->audio_mutex);
	err =
	    snd_pcm_new(chip->card, NULL, 0, MAX_SUBSTREAMS, 0, &chip->pcm);
	if (err < 0)
		goto out;
	
	printk("Setting pcm chip to %p\n", chip);
	chip->pcm->private_data = chip;
	strcpy(chip->pcm->name, chip->card->shortname);
	
	/* set operators */
	snd_pcm_set_ops(chip->pcm, SNDRV_PCM_STREAM_PLAYBACK,
			&snd_bcm2835_playback_ops);

	/* pre-allocation of buffers */
	/* NOTE: this may fail */
	/*snd_pcm_lib_preallocate_pages_for_all(chip->pcm, SNDRV_DMA_TYPE_CONTINUOUS,
					      snd_dma_continuous_data (GFP_KERNEL),
					      bcm2835_pwm_pcm_hardware.buffer_bytes_max, bcm2835_pwm_pcm_hardware.buffer_bytes_max);*/


out:
	audio_info(chip, " .. OUT %d\n", err);

	return err;
}


///////////////////////////////////////////////////////////////////////////////////////////

static ssize_t snd_bcm2835_print_bar(char* buf, const char* name, int value, int size, int period) {
	int i;
	int count;

	if(value > size)
		value = size;
	else if(value < 0)
		value = 0;
	
	count = sprintf(buf, "%16s: %5d / %5d [", name, value, size);
	memset(buf + count, ' ', 64);
	
	for(i = 0; i < size; i += period) {
		buf[count + (i + 1) * 64 / size] = '|';
	}
	buf[count + value * 64 / size] = '#';
	buf[count + 64] = ']';
	buf[count + 65] = '\n';
	return count + 66;
}

static ssize_t snd_bcm2835_status_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	if(!g_chip) {
		return -ENODEV;
	} else {
		int alsa_write_pos = 0;
		int alsa_indirect_sw_pos = 0;
		int alsa_indirect_hw_pos = 0;
		int alsa_size = 1;
		int alsa_period = 1;
		int dma_write_pos = g_chip->chip.pos;
		int dma_read_pos = bcm2835_pwm_aud_pointer(&g_chip->chip);
		int alsa_indirect_sw_ptr = 0;
		int alsa_indirect_hw_ptr = 0;
		int count = 0;
		
		if(g_chip->alsa_stream) {
			struct snd_pcm_runtime *runtime = g_chip->alsa_stream->substream->runtime;
			
			alsa_size = runtime->buffer_size;
			alsa_period = runtime->period_size;
			alsa_write_pos = runtime->control->appl_ptr % runtime->buffer_size;
			alsa_indirect_sw_pos = g_chip->alsa_stream->pcm_indirect.sw_data / 4;
			alsa_indirect_hw_pos = g_chip->alsa_stream->pcm_indirect.hw_data / 4;
			alsa_indirect_sw_ptr = g_chip->alsa_stream->pcm_indirect.sw_io / 4;
			alsa_indirect_hw_ptr = g_chip->alsa_stream->pcm_indirect.hw_io / 4;
		}
		
		count += snd_bcm2835_print_bar(buf + count, "alsa_write_pos", alsa_write_pos, alsa_size, alsa_period);
		count += snd_bcm2835_print_bar(buf + count, "indirect_sw_pos", alsa_indirect_sw_pos, alsa_size, alsa_period);
		count += snd_bcm2835_print_bar(buf + count, "indirect_sw_ptr", alsa_indirect_sw_ptr, alsa_size, alsa_period);
		count += snd_bcm2835_print_bar(buf + count, "indirect_hw_pos", alsa_indirect_hw_pos, alsa_size, alsa_period);
		count += snd_bcm2835_print_bar(buf + count, "dma_write_pos", dma_write_pos * 3 / 25, alsa_size, alsa_period);
		count += snd_bcm2835_print_bar(buf + count, "indirect_hw_ptr", alsa_indirect_hw_ptr, alsa_size, alsa_period);
		count += snd_bcm2835_print_bar(buf + count, "dma_read_pos", dma_read_pos, alsa_size, alsa_period);
		//dump_pcm_indirect(g_chip, &g_chip->alsa_stream->pcm_indirect);
		
		return count;
	}
}
static DEVICE_ATTR(status, S_IRUGO, snd_bcm2835_status_show, NULL);

static void snd_bcm2835_chip_release(struct device *dev, void *res) {
	bcm2835_alsa_chip_t* chip = (bcm2835_alsa_chip_t*) res;
	
	dev_dbg(dev, "releasing chip\n");
	
	device_remove_file(dev, &dev_attr_status);
	
	g_chip = NULL;
	if(chip->card)
		snd_card_free(chip->card);
	chip->card = NULL;
	
	bcm2835_pwm_aud_free(&chip->chip);
}

static int snd_bcm2835_pwm_alsa_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	bcm2835_alsa_chip_t* chip = NULL;
	int error = 0;
	
	dev_dbg(&pdev->dev, "probed\n");
	
	chip = devres_alloc(snd_bcm2835_chip_release, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	error = bcm2835_pwm_aud_init(&chip->chip, pdev);
	if(error) {
		dev_err(&pdev->dev, "Failed to init: %d\n", error);
		goto err;
	}
	
	error = snd_card_new(&pdev->dev, -1, "bcm2835-pwm-aud", THIS_MODULE, 0, &chip->card);
	if (error) {
		dev_err(dev, "Failed to create soundcard structure\n");
		goto err;
	}
	
	strcpy(chip->card->driver, dev_driver_string(&pdev->dev));
	strcpy(chip->card->shortname, "bcm2835 PWM 0/1 jack");
	strcpy(chip->card->longname, dev_name(&pdev->dev));
	

	error = snd_bcm2835_pwm_new_pcm(chip);
	if (error < 0) {
		dev_err(dev, "Failed to create pcm device\n");
		goto err;
	}
	
	error = snd_card_register(chip->card);
	if (error < 0) {
		dev_err(dev, "Failed to register soundcard\n");
		goto err;
	}

	devres_add(dev, chip);
	g_chip = chip;
	
	device_create_file(dev, &dev_attr_status);
	return 0;
err:
	if(chip) {
		snd_bcm2835_chip_release(dev, chip);
		devres_free(chip);
	}
	dev_err(&pdev->dev, "error %d\n", error);
	return error;
}

static int snd_bcm2835_pwm_alsa_remove(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "removed\n");
	return 0;
}

#ifdef CONFIG_PM
static int snd_bcm2835_pwm_alsa_suspend(struct platform_device *pdev,
				    pm_message_t state)
{
	return 0;
}

static int snd_bcm2835_pwm_alsa_resume(struct platform_device *pdev)
{
	return 0;
}

#endif

static const struct of_device_id snd_bcm2835_pwm_of_match_table[] = {
	{ .compatible = "brcm,bcm2835-audio-pwm", },
	{},
};
MODULE_DEVICE_TABLE(of, snd_bcm2835_pwm_of_match_table);

static struct platform_driver bcm2835_alsa_driver = {
	.probe = snd_bcm2835_pwm_alsa_probe,
	.remove = snd_bcm2835_pwm_alsa_remove,
#ifdef CONFIG_PM
	.suspend = snd_bcm2835_pwm_alsa_suspend,
	.resume = snd_bcm2835_pwm_alsa_resume,
#endif
	.driver = {
		   .name = "snd-bcm2835-pwm",
		   .owner = THIS_MODULE,
		   .of_match_table = snd_bcm2835_pwm_of_match_table,
		   },
};

static int bcm2835_alsa_device_init(void)
{
	int err;
	err = request_module("snd-pcm");
	if(err)
		return err;
	
	err = platform_driver_register(&bcm2835_alsa_driver);
	if (err) {
		pr_err("Error registering bcm2835_alsa_driver %d .\n", err);
		return err;
	}

	return 0;
}

static void bcm2835_alsa_device_exit(void)
{
	platform_driver_unregister(&bcm2835_alsa_driver);
}

late_initcall(bcm2835_alsa_device_init);
module_exit(bcm2835_alsa_device_exit);
