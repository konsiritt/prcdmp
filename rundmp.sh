#!/bin/bash
if [ ! -z "$1" ];
then

    DATASETPATH="$1"
    BASEDIR=$PWD

    # check if specified dataset structure exists
    if [ ! -d "data/$DATASETPATH" ];then
	    echo "Specified dataset path does not exist"
    else
	    if [ ! -f "data/$DATASETPATH/conf/DMP.json" ];then
		    echo "Specified dataset lacks $DATASETPATH/conf/DMP.json"
	    fi
    fi

    cd PRCDMP/bin
    
    if [ "$2" = "r" ]; then #this will also let the trajectory run on the robot!!
	read -n1 -r -p "trajectory will now be executed on the robot - Press space to confirm..." key	
		if [ "$key" = '' ]; then
			./runtraj $DATASETPATH
		else
			./rundmp $DATASETPATH		
		fi
    else
	./rundmp $DATASETPATH
    fi

else
    echo "No path to dataset specified."
fi
