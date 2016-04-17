
CC = g++
CLFLAGS = -lGL -lGLU -lglut -lm 
CCFLAGS = -O3 --std=c++11

OBJS = main.o cloth.o

%.o: %.cpp
	g++ $(CLFLAGS) $(CCFLAGS) -c $< -o $@

main: $(OBJS)
	g++ $(CLFLAGS) $(CCFLAGS) $(OBJS) -o $@

clean:
	rm -rf main *.o
