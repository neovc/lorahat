PROG = lorahat
$(PROG): lora.c
	gcc -o $@ -O2 -Wall -g -I. $^ -levent -lpthread
clean:
	rm -f $(PROG) *.o
