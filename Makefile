MCU=atmega328p
F_CPU=16000000UL
CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS=-mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os

all: traffic_controller.hex

traffic_controller.elf: traffic_controller.c
	$(CC) $(CFLAGS) -o $@ $<

traffic_controller.hex: traffic_controller.elf
	$(OBJCOPY) -O ihex $< $@

clean:
	rm -f *.elf *.hex 