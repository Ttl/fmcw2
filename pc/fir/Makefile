default: fir

fir.o: fir.c
	gcc -O3 -c fir.c -o fir.o

fir: fir.o
	gcc fir.o -o fir

clean:
	-rm -f fir.o
	-rm -f fir
