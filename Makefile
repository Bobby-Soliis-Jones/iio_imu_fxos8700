#
# Makefile for Freescale FXOS8700 IMU
#
obj-$(CONFIG_FXOS8700) += fxos8700_core.o
obj-$(CONFIG_FXOS8700_I2C) += fxos8700_i2c.o
obj-$(CONFIG_FXOS8700_SPI) += fxos8700_spi.o