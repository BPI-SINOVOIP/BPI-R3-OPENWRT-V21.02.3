#
# Copyright (C) 2006-2011 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

LEDS_MENU:=LED modules

define KernelPackage/leds-gpio
  SUBMENU:=$(LEDS_MENU)
  TITLE:=GPIO LED support
  DEPENDS:= @GPIO_SUPPORT
  KCONFIG:=CONFIG_LEDS_GPIO
  FILES:=$(LINUX_DIR)/drivers/leds/leds-gpio.ko
  AUTOLOAD:=$(call AutoLoad,60,leds-gpio,1)
endef

define KernelPackage/leds-gpio/description
 Kernel module for LEDs on GPIO lines
endef

$(eval $(call KernelPackage,leds-gpio))

LED_TRIGGER_DIR=$(LINUX_DIR)/drivers/leds/trigger

define KernelPackage/ledtrig-activity
  SUBMENU:=$(LEDS_MENU)
  TITLE:=LED Activity Trigger
  KCONFIG:=CONFIG_LEDS_TRIGGER_ACTIVITY
  FILES:=$(LED_TRIGGER_DIR)/ledtrig-activity.ko
  AUTOLOAD:=$(call AutoLoad,50,ledtrig-activity)
endef

define KernelPackage/ledtrig-activity/description
 Kernel module that allows LEDs to blink based on system load
endef

$(eval $(call KernelPackage,ledtrig-activity))

define KernelPackage/ledtrig-gpio
  SUBMENU:=$(LEDS_MENU)
  TITLE:=LED GPIO Trigger
  KCONFIG:=CONFIG_LEDS_TRIGGER_GPIO
  FILES:=$(LED_TRIGGER_DIR)/ledtrig-gpio.ko
  AUTOLOAD:=$(call AutoLoad,50,ledtrig-gpio)
endef

define KernelPackage/ledtrig-gpio/description
 Kernel module that allows LEDs to be controlled by gpio events
endef

$(eval $(call KernelPackage,ledtrig-gpio))


define KernelPackage/ledtrig-transient
  SUBMENU:=$(LEDS_MENU)
  TITLE:=LED Transient Trigger
  KCONFIG:=CONFIG_LEDS_TRIGGER_TRANSIENT
  FILES:=$(LED_TRIGGER_DIR)/ledtrig-transient.ko
  AUTOLOAD:=$(call AutoLoad,50,ledtrig-transient,1)
endef

define KernelPackage/ledtrig-transient/description
 Kernel module that allows LEDs one time activation of a transient state.
endef

$(eval $(call KernelPackage,ledtrig-transient))


define KernelPackage/ledtrig-oneshot
  SUBMENU:=$(LEDS_MENU)
  TITLE:=LED One-Shot Trigger
  KCONFIG:=CONFIG_LEDS_TRIGGER_ONESHOT
  FILES:=$(LED_TRIGGER_DIR)/ledtrig-oneshot.ko
  AUTOLOAD:=$(call AutoLoad,50,ledtrig-oneshot)
endef

define KernelPackage/ledtrig-oneshot/description
 Kernel module that allows LEDs to be triggered by sporadic events in
 one-shot pulses
endef

$(eval $(call KernelPackage,ledtrig-oneshot))


define KernelPackage/ledtrig-pattern
  SUBMENU:=$(LEDS_MENU)
  TITLE:=LED Pattern Trigger
  KCONFIG:=CONFIG_LEDS_TRIGGER_PATTERN
  FILES:=$(LED_TRIGGER_DIR)/ledtrig-pattern.ko
  AUTOLOAD:=$(call AutoLoad,50,ledtrig-pattern)
endef

define KernelPackage/ledtrig-pattern/description
 This allows LEDs to be controlled by a software or hardware pattern
 which is a series of tuples, of brightness and duration (ms).
endef

$(eval $(call KernelPackage,ledtrig-pattern))


define KernelPackage/leds-pca963x
  SUBMENU:=$(LEDS_MENU)
  TITLE:=PCA963x LED support
  DEPENDS:=+kmod-i2c-core
  KCONFIG:=CONFIG_LEDS_PCA963X
  FILES:=$(LINUX_DIR)/drivers/leds/leds-pca963x.ko
  AUTOLOAD:=$(call AutoLoad,60,leds-pca963x,1)
endef

define KernelPackage/leds-pca963x/description
 Driver for the NXP PCA963x I2C LED controllers.
endef

$(eval $(call KernelPackage,leds-pca963x))


define KernelPackage/leds-pwm
  SUBMENU:=$(LEDS_MENU)
  TITLE:=PWM driven LED Support
  KCONFIG:=CONFIG_LEDS_PWM
  DEPENDS:= @PWM_SUPPORT
  FILES:=$(LINUX_DIR)/drivers/leds/leds-pwm.ko
  AUTOLOAD:=$(call AutoLoad,60,leds-pwm,1)
endef

define KernelPackage/leds-pwm/description
 This option enables support for pwm driven LEDs
endef

$(eval $(call KernelPackage,leds-pwm))

define KernelPackage/leds-uleds
  SUBMENU:=$(LEDS_MENU)
  TITLE:=Userspace LEDs
  KCONFIG:=CONFIG_LEDS_USER
  FILES:=$(LINUX_DIR)/drivers/leds/uleds.ko
  AUTOLOAD:=$(call AutoLoad,60,uleds,1)
endef

define KernelPackage/leds-uleds/description
 This option enables support for userspace LEDs.
endef

$(eval $(call KernelPackage,leds-uleds))
