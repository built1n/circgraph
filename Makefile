CFLAGS=-Og -g
CXXFLAGS=$(CFLAGS)

all: genrand circgraph Makefile
genrand: genrand.o
circgraph: main.o
	$(CXX) -o $@ $< $(CFLAGS)

clean:
	rm -f genrand circgraph *.o
