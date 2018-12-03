CC=clang
CFLAGS=-Wall -g


BINS=project4 test


all: $(BINS)

project4: project4.c
	$(CC) $(CFLAGS) -o notjustcats $<

%: %.c
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm $(BINS)
