CC=gcc
CXX=g++
OBJ = mavmon.o

%.o: %.c
	$(CC) -c -o $@ $< $(CFLAGS)

%.o: %.cxx
	$(CXX) -c -o $@ $< $(CFLAGS)

all: mavmon 

mavmon: $(OBJ)
	gcc -o mavmon mavmon.o $(CFLAGS) -lpthread

clean:
	rm -f *.o mavmon 

submission:
	make clean
	tar -c *.c Makefile | gzip -n > $(USER).tgz

submit:
	make submission
	sudo /usr/local/share/CSE3320/scripts/submit.py -a $(assignment) -i $(USER).tgz -u $(USER)

verify:
	make submission
	sudo /usr/local/share/CSE3320/scripts/verify.py -a $(assignment) -i $(USER).tgz -u $(USER)
