# Select the example you want to enable by choosing y for enabling and n for disabling
# Enable only one example at a time.
IIO_EXAMPLE = n
REGISTER_EXAMPLE = y

include ../../tools/scripts/generic_variables.mk

# The AD5933 IIO driver uses sqrt()/atan2() for impedance/phase math.
LIB_FLAGS += -lm

include src.mk

include ../../tools/scripts/generic.mk
