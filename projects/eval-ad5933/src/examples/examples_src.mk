ifeq (y,$(strip $(REGISTER_EXAMPLE)))
CFLAGS += -DREGISTER_EXAMPLE
SRCS += $(PROJECT)/src/examples/register_example/register_example.c
INCS += $(PROJECT)/src/examples/register_example/register_example.h
endif

ifeq (y,$(strip $(IIO_EXAMPLE)))
IIOD=y
CFLAGS += -DIIO_EXAMPLE
SRCS += $(PROJECT)/src/examples/iio_example/iio_example.c
INCS += $(PROJECT)/src/examples/iio_example/iio_example.h
endif

ifeq (y,$(strip $(IIOD)))
SRC_DIRS += $(NO-OS)/iio/iio_app
INCS += $(DRIVERS)/impedance-analyzer/ad5933/iio_ad5933.h

SRCS += $(DRIVERS)/impedance-analyzer/ad5933/iio_ad5933.c

INCS += $(INCLUDE)/no_os_list.h \
		$(PLATFORM_DRIVERS)/$(PLATFORM)_uart.h
endif
