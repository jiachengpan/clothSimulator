
CC = g++
CLFLAGS = -lGL -lGLU -lglut -lm 
CCFLAGS = -O3 --std=c++11

NVCC = nvcc
NVCLFLAGS = -lGL -lGLU -lglut -lm 
NVCCFLAGS = -O3 --std=c++11 -m64 -D_MWAITXINTRIN_H_INCLUDED

OBJS = main.o cloth.o gpu_cloth.o

all: main

%.o: %.cu
	$(NVCC) $(NVCLFLAGS) $(NVCCFLAGS) -c $< -o $@

%.o: %.cpp
	$(CC) $(CLFLAGS) $(CCFLAGS) -c $< -o $@

main: $(OBJS)
	$(NVCC) $(CLFLAGS) $(CCFLAGS) $(OBJS) -o $@

clean:
	rm -rf main *.o
