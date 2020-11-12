#!/bin/bash
BASE_DIR=~/quantum_ws

cd $BASE_DIR
catkin_make install
tar -czf /tmp/quantum_install.tar.gz install
scp      /tmp/quantum_install.tar.gz user@quantum:/tmp
ssh user@quantum 'tar xf /tmp/quantum_install.tar.gz -C ~/quantum_ws'
