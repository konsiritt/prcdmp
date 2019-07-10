#!/bin/sh -e


### make ###
cd PRCDMP/build
cmake ../..
sudo make -j8 install

### push all libraries ###

IP="$1"   

cd ../..

if [ ! -z "$1" ];
then
rsync -r PRCDMP/bin franka@$IP:~/prcdmp/PRCDMP/
rsync -r data franka@$IP:~/prcdmp/
rsync -r pydmp franka@$IP:~/prcdmp/
rsync -r record.sh franka@$IP:~/prcdmp/
rsync -r rundmp.sh franka@$IP:~/prcdmp/
#rsync kin_dyn_learn/libkin_dyn_learn.so franka@$IP:~/learn_self/
fi
