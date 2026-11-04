SRC := ../../arm9/source/utils
CFLAGS := -std=c11 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined -I$(SRC)

test: test_cartref
	./test_cartref

test_cartref: test_cartref.c $(SRC)/cartref.c $(SRC)/cartref.h
	$(CC) $(CFLAGS) -o $@ test_cartref.c $(SRC)/cartref.c

clean:
	rm -f test_cartref

.PHONY: test clean
