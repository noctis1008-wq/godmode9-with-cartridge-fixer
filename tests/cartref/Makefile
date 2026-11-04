SRC := ../../arm9/source/utils
CFLAGS := -std=c11 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined -I$(SRC)

test: test_cartref test_crh
	./test_cartref
	python3 mkimage.py /tmp/crh_test.3ds
	python3 ../../tools/make_crh.py /tmp/crh_test.3ds /tmp/crh_test.crh
	./test_crh /tmp/crh_test.3ds /tmp/crh_test.crh

test_cartref: test_cartref.c $(SRC)/cartref.c $(SRC)/cartref.h
	$(CC) $(CFLAGS) -o $@ test_cartref.c $(SRC)/cartref.c

test_crh: test_crh.c $(SRC)/cartrefhash.h
	$(CC) $(CFLAGS) -o $@ test_crh.c -lcrypto

clean:
	rm -f test_cartref test_crh

.PHONY: test clean
