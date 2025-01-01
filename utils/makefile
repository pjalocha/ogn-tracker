all:		serial_dump read_log tlg2aprs aprs2igc

serial_dump:	serial_dump.cc
	g++ -Wall -Wno-misleading-indentation -I../src -O2 -o serial_dump serial_dump.cc ../src/format.cpp

read_log:	read_log.cc
	g++ -Wall -Wno-misleading-indentation -I../src -O2 -o read_log read_log.cc ../src/format.cpp

tlg2aprs:	tlg2aprs.cc
	g++ -Wall -Wno-misleading-indentation -O2 -o tlg2aprs -I../src tlg2aprs.cc ../src/intmath.cpp ../src/format.cpp ../src/ognconv.cpp

aprs2igc:	aprs2igc.cc
	g++ -Wall -Wno-misleading-indentation -O2 -o aprs2igc -I../src aprs2igc.cc ../src/format.cpp ../src/ognconv.cpp

clean:
	rm serial_dump read_log aprs2igc

