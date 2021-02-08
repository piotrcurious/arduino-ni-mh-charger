#!/bin/bash
#./serial_to_file.sh &
read  -r -p "NAME  :"
echo "DEVICE 2: "$REPLY > name.txt
echo "insert battery : " $REPLY
./set_serial.sh
cat /dev/ttyUSB0 > data.raw &
SERIAL_TO_FILE_PID=$!
echo $SERIAL_TO_FILE_PID
sleep 30
./data_clean.sh

gnuplot device2.config &
GNUPLOT_PID=$!
echo GNUPLOT_PID

./data_clean_loop.sh &
DATA_CLEAN_PID=$!
echo $DATA_CLEAN_PID

while read -n1 -r -p "press q to finish session"
do
    if [[ $REPLY == q ]];
    then
        break;
    else
        tail  -n3 data.raw
    fi
done

kill $SERIAL_TO_FILE_PID
kill $GNUPLOT_PID
kill $DATA_CLEAN_PID

while read  -r -p "directory to move data to  , or q to quit:"
do
    if [[ $REPLY == q ]];
    then
        break;
    else
	mkdir $REPLY 
	cp data.raw $REPLY
	cp device2.txt $REPLY
	cp device2.config $REPLY
	cp name.txt $REPLY
	fi
done
