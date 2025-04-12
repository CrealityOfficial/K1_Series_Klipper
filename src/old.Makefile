# Main code build rules

src-y += sched.c command.c basecmd.c debugcmds.c fan_check.c
src-$(CONFIG_HAVE_GPIO) += initial_pins.c gpiocmds.c stepper.c endstop.c \
    trsync.c prtouch_v2_compile.c
src-$(CONFIG_HAVE_PRTOUCH) += hx711s.c dirzctl.c
src-$(CONFIG_HAVE_GPIO_ADC) += adccmds.c
src-$(CONFIG_HAVE_GPIO_SPI) += spicmds.c thermocouple.c
src-$(CONFIG_HAVE_GPIO_I2C) += i2ccmds.c
src-$(CONFIG_HAVE_GPIO_HARD_PWM) += pwmcmds.c
bb-src-$(CONFIG_HAVE_GPIO_SPI) := spi_software.c sensor_adxl345.c
#	sensor_angle.c
bb-src-$(CONFIG_HAVE_GPIO_I2C) += sensor_mpu9250.c
src-$(CONFIG_HAVE_GPIO_BITBANGING) += $(bb-src-y) tmcuart.c neopixel.c pulse_counter.c buttons.c
#	lcd_st7920.c lcd_hd44780.c 
    
