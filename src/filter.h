#ifndef FILTER_H
#define FILTER_H

#include <asm/string.h>

typedef int16_t sample_t;
typedef int32_t sample_hires_t;

struct filter_sample_t {
	sample_t left;
	sample_t right;
};

struct filter_sample_hires_t {
	sample_hires_t left;
	sample_hires_t right;
};

struct filter_fir_t {
	sample_t coefs[128];
	struct filter_sample_hires_t history[128];
	int last_index;
	int upsampling_ratio;
	int downsampling_ratio;
	int modpos;
};

static inline void filter_fir_init(struct filter_fir_t* f, const sample_t coefs[128], int coef_size, int upsampling_ratio, int downsampling_ratio) {
	memcpy(f->coefs, coefs, sizeof(*f->coefs) * coef_size);
	memset(f->history, 0, sizeof(f->history));
	f->last_index = 0;
	f->upsampling_ratio = upsampling_ratio;
	f->downsampling_ratio = downsampling_ratio;
	f->modpos = 0;
}

static inline void filter_fir_put(struct filter_fir_t* f, sample_hires_t input_left, sample_hires_t input_right) {
	f->history[++f->last_index & (128-1)] = (struct filter_sample_hires_t){input_left, input_right};
}

static inline void filter_fir_get(struct filter_fir_t* f, struct filter_sample_hires_t* outputs, int* output_size) {
	#define TAP 128
	int upsampling_ratio = 25;
	int downsampling_ratio = 3;
	int n = f->last_index;
	int startout = f->modpos;
	int k;
	sample_t* coefs = f->coefs;
	int tap_lefts = TAP;
	
	memset(outputs, 0, sizeof(*outputs) * (*output_size));
	*output_size = TAP > upsampling_ratio ? (upsampling_ratio - startout - 1)/downsampling_ratio + 1 : (TAP - startout - 1)/downsampling_ratio + 1;

	for(k = 0; k < TAP; k += upsampling_ratio, n--, tap_lefts -= upsampling_ratio) {

		struct filter_sample_hires_t hval = f->history[n & (128-1)];
		int l, o;
		int end = upsampling_ratio;
		if(unlikely(tap_lefts < upsampling_ratio))
			end = tap_lefts;
		for(o = 0, l = startout; l < end; l += downsampling_ratio, o++) {
			sample_t coef = coefs[k + l];
			outputs[o].left += hval.left * coef;
			outputs[o].right += hval.right * coef;
		}
	};
	
	startout += ((downsampling_ratio - (upsampling_ratio % downsampling_ratio)) % downsampling_ratio);
	if(startout >= downsampling_ratio)
		startout -= downsampling_ratio;
	f->modpos = startout;
}

#endif
