CFLAGS=-Wall -Wextra -ggdb -fdiagnostics-color=auto
OBJS=sc16is7xx_demo.o sc16is7xx.o 

all : sc16is7xx_demo

sc16is7xx_demo : $(OBJS)
	gcc $(LDFLAGS) -o $@ $^

.PHONY : clean
clean :
	rm -f *~ *.o sc16is7xx_demo

