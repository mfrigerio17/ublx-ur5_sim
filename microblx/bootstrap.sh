#!/bin/bash

for D in `find blocks/ -mindepth 1 -maxdepth 1 -type d`; do
    pushd $D > /dev/null
    mkdir -p build && cd build
    for F in `find .. -type f`; do ln -s  $F; done
    autoreconf --install
    ./configure
    make && sudo make install
    popd > /dev/null
done
