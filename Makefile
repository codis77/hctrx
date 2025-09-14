# makefile for the gmt application
# the '?=' combination sets a variable only
# if it has not been set previously

# General configuration variables:
DESTDIR ?= /
INCDIR ?= $(DESTDIR)/usr/include

LIBS = -lgpiod

HCTRX_OBJECTS = hctrx.c
HCTRX_TARGET = hctrx
HCCFG_OBJECTS = hccfg.c
HCCFG_TARGET = hccfg

# MODULES = $(SRCS:.c=.o)
# MODULES := $(MODULES:.c=.o)
CC ?= gcc
# CFLAGS ?= -Wall
CFLAGS += $(INCLUDE) -Wno-unused-result
# CFLAGS += -I./gpl
LINK_FLAGS = $(LIBS)


default: all

all: hctrx hccfg

hctrx:
	$(CC) -o $(HCTRX_TARGET) $(CFLAGS) -O1 $(HCTRX_OBJECTS) $(LINK_FLAGS)
hctrx_host:
	$(CC) -o $(HCTRX_TARGET) $(CFLAGS) -D_PC_HOST_ -g -O1 $(HCTRX_OBJECTS) $(LINK_FLAGS)

hccfg:
	$(CC) -o $(HCCFG_TARGET) $(CFLAGS) -O1 $(HCCFG_OBJECTS) $(LINK_FLAGS)
hccfg_host:
	$(CC) -o $(HCCFG_TARGET) $(CFLAGS) -D_PC_HOST_ -g -O1 $(HCCFG_OBJECTS) $(LINK_FLAGS)

.c.o:
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o $(HCCFG_TARGET) $(HCTRX_TARGET)
