#
# Makefile for ALSA
#

obj-m	+= snd-bcm2835-pwm.o
snd-bcm2835-pwm-objs		:= bcm2835-pwm-aud.o bcm2835-pwm-pcm.o
ccflags-y += -g
dtbo-y += pwm-audio.dtbo

targets += dtbs
targets += $(dtbo-y)

always		:= $(dtbo-y)
clean-files	:= *.dtbo

ifeq ($(CONFIG_ARCH_BCM2835),y)
	DTC_FLAGS ?= -@ -H epapr
endif
