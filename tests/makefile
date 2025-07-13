gps_sat_test:	gps_sat_test.cc
	g++ -Wall -Wno-misleading-indentation -o gps_sat_test -I../src gps_sat_test.cc ../src/format.cpp

bitshift_test:	bitshift_test.cc ../src/rx-pkt.h
	g++ -Wall -Wno-misleading-indentation -O2 -o bitshift_test -I../src \
                         bitshift_test.cc ../src/bitcount.cpp ../src/format.cpp ../src/ldpc.cpp

