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
echo "###################################"
echo "killme.sh: Kill camera_publish process..."
echo "####################################"
ka camera_publish
echo "###################################"
echo "killme.sh: Kill camera_publish process DONE"
echo "####################################"
echo "###################################"
echo "killme.sh: Kill findlines_sub_pub process..."
echo "####################################"
ka findlines_sub_pub
echo "###################################"
echo "killme.sh: Kill findlines_sub_pub process DONE"
echo "####################################"
echo "###################################"
echo "killme.sh: Kill detectobj_sub_pub process..."
echo "####################################"
ka detectobj_sub_pub
echo "###################################"
echo "killme.sh: Kill detectobj_sub_pub process DONE"
echo "####################################"



