default: all

CFLAGS := -I./include -g --std=gnu99
CC := gcc

BINARIES := hubo-hw1
all : $(BINARIES)

LIBS := -lach 

hubo-hw1: src/hubo-hw1.o
	gcc -o $@ $< $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(BINARIES) src/*.o
