CC=gcc
CFLAGS=-g -Wall $(shell pkg-config --cflags libftdi1)
LDFLAGS=-g -Wall $(shell pkg-config --libs libftdi1)
DEST=fixFT2232_ecp5evn
SRC=$(wildcard *.c)
OBJS=$(SRC:.c=.o)

all: $(DEST)
$(DEST): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)
%.o:%.c
	$(CC) $(CFLAGS) -o $@ -c $<
clean:
	@rm -rf $(DEST) *.o
