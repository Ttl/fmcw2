default: fir

fir.o: fir.c
	gcc -O3 -c fir.c -o fir.o

fir: fir.o
	gcc fir.o -o fir -lm

clean:
	-rm -f fir.o
	-rm -f fir
