#!/bin/bash

# if [ -f libipasirz3.so ]; then
#     echo "z3 ipasir library already compiled - skipping build"
#     exit
# fi

# If we do not have the z3 library, we need to build it
if [ ! -f libz3.so ]; then
    echo "libz3.so not found - building z3"
    # Get and compile the z3 SMT solver
    if [ ! -d z3 ]; then
        git clone https://github.com/Z3Prover/z3
        cd z3
    else
        cd z3
        git clean -f
    fi

    cd z3
    python3 scripts/mk_make.py
    cd build
    make
    cp libz3.so ../../
    cd ../..
fi


# Now, compile the libipasirz3.so library
make


