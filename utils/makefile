serial_dump:	serial_dump.cc
	g++ -Wall -Wno-misleading-indentation -I../src -O2 -o serial_dump serial_dump.cc ../src/format.cpp

read_log:	read_log.cc
	g++ -Wall -Wno-misleading-indentation -I../src -O2 -o read_log read_log.cc ../src/format.cpp

aprs2igc:	aprs2igc.cc
	g++ -Wall -O2 -o aprs2igc aprs2igc.cc format.cpp ognconv.cpp

clean:
	rm serial_dump read_log aprs2igc

