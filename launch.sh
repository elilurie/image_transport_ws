#!/bin/bash
###################################################################
# handle_argument
###################################################################
handle_argument() {
	echo "handle_argument()" 	

}	
echo "###################################"
echo "Kill all ros processes first..."
echo "####################################"
#ps -ax | grep ros | awk '{print $1}' | xargs kill -9 $1
# FIND PROCESS
p() {
        ps aux | grep -i $1 | grep -v grep
}
# KILL ALL
ka() {

    cnt=$( p $1 | wc -l)  # total count of processes found
    klevel=${2:-15}       # kill level, defaults to 15 if argument 2 is empty

    echo -e "\nSearching for '$1' -- Found" $cnt "Running Processes .. "
    p $1
    if [ "${cnt}" -eq "0" ]; then
    	echo "ros processes were not found. kill no one"
    else

	echo -e '\nTerminating' $cnt 'processes .. '

	ps aux  |  grep -i $1 |  grep -v grep   | awk '{print $2}' | xargs sudo kill -9
	echo -e "Done!\n"

	echo "Running search again:"
	p "$1"
	echo -e "\n"
    fi
}
ka camera_publish
ka findlines_sub_pub

echo "###################################"
echo "Kill all ros processes DONE..."
echo "###################################"
source ./devel/setup.bash
echo "Launch node that publish video...(/dev/video0)"
rosrun image_transport_tutorial camera_publish.py 0 &
#rosrun image_transport_tutorial camera_publish.py '../line-following/py/gals deck.mp4' 
echo "Launch node that subscribes to video, implements algorithm on it and republishes it again..."
rosrun image_transport_tutorial findlines_sub_pub.py &

