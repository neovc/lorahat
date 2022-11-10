PROG = loraip
$(PROG): lora.c
	gcc -o $@ -O2 -Wall -g -I. $^
clean:
	rm -f $(PROG) *.o
