CC = g++
CFLAGS = -shared -fPIC
LDFLAGS = -L. -lipasirz3 -lz3
LIBRARY = libipasirz3.so
# TEST_BINARY = test_ipasir
SOURCES = ipasir_z3.cpp # test_ipasir.cpp

all: $(LIBRARY) $(TEST_BINARY) test

$(LIBRARY): ipasir_z3.cpp
	$(CC) $(CFLAGS) -o $(LIBRARY) ipasir_z3.cpp -lz3

#$(TEST_BINARY): test_ipasir.cpp $(LIBRARY)
#	$(CC) -o $(TEST_BINARY) test_ipasir.cpp $(LDFLAGS)

#test: $(TEST_BINARY)
#	./$(TEST_BINARY)

clean:
	rm -f $(LIBRARY) $(TEST_BINARY)
