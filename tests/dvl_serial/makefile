all: dvl_serial

dvl_serial: main.o serial.o dvl_protocol.o
	gcc -Wall -o dvl_serial main.o serial.o dvl_protocol.o

main.0: main.c
	gcc -c -Wall main.c

serial.o: serial.c
	gcc -c -Wall serial.c

dvl_protocol.o: dvl_protocol.c
	gcc -c -Wall dvl_protocol.c

clean:
	rm main.o
	rm serial.o
	rm dvl_protocol.o