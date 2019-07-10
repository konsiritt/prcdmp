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

    # prepare python environment
    pyEnvironment="pydmp"

    # start procedure to allow recording of a trajectory
    cd PRCDMP/bin
    if ./record $DATASETPATH; then
        echo "success"
        cd ../..
        echo $result
        echo "Robot trajectory recorded to file"

        if [ "$2" = "w" ];
        then
            # start up python environment and learn weights
            cd pydmp/examples
            source activate $pyEnvironment
            export PYTHONPATH=$PYTHONPATH:$BASEDIR/pydmp/examples
            python learn_dmp.py -d $1

            echo "DMP weights have been learned - ready to imitate motion" 
        fi
    else
        echo "Failure"
    fi
else
    echo "No path to dataset specified. Give relative path from data/"
fi
