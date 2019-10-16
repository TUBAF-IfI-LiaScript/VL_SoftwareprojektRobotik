OBJ = main.o logger.o
FLAGS = -g -Wall

all: main

logger.o: logger.hpp

%o: %cpp
	g++ -c $(FLAGS) -o $@ $<
	#g++ -S $(FLAGS) -o $@ $<

main: $(OBJ)
	g++ -o main $(OBJ)

clean:
	rm logger.o
	rm main.o
	rm main
