CC = gcc
CFLAGS := -Wall -g -O2 -flto -fwhole-program -fno-use-linker-plugin

OBJECTS = pyhelper.o serialqueue.o stepcompress.o itersolve.o trapq.o \
		  pollreactor.o msgblock.o trdispatch.o \
		  kin_cartesian.o kin_corexy.o kin_corexz.o kin_delta.o \
		  kin_deltesian.o kin_polar.o kin_rotary_delta.o kin_winch.o \
		  kin_extruder.o kin_shaper.o

ifeq ($(wildcard serial_485_queue.c), serial_485_queue.c)
        OBJECTS += serial_485_queue.o
endif
ifeq ($(wildcard msgblock_485.c), msgblock_485.c)
        OBJECTS += msgblock_485.o
endif
ifeq ($(wildcard filament_change.c), filament_change.c)
        OBJECTS += filament_change.o
endif

DIST_LIB = c_helper.so

all: $(DIST_LIB)

clean:
	@echo "Cleaning up directory."
	rm -f *.a *.o  $(DIST_LIB) *~

$(DIST_LIB):$(OBJECTS)
	$(CC) -shared -fPIC -o $@ $(OBJECTS)

%.o: %.c
	$(CC) $(CFLAGS) -g -c -o $@ $<

