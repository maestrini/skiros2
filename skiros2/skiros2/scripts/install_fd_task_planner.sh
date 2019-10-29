#!/bin/bash

tfd='tfd-src-0.4'

echo "Installing planner..."

#Navigate to install folder
folder="$HOME/utils"
mkdir -p $folder
cd $folder

# Installing planner
if [ ! -d "$folder/${tfd}" ]; then
    echo "Installing FD planner in $folder"
    mkdir -p $folder
    cd $folder
    echo $folder
    wget "http://gki.informatik.uni-freiburg.de/tools/tfd/downloads/version-0.4/${tfd}.tgz"
    tar xzf "${tfd}.tgz"
    cd "${tfd}"
    sed -e s/"-Werror"//g -i ./downward/search/Makefile
    sed -e s/"translate\/"//g -i ./downward/plan.py
    sed -e s/"preprocess\/"//g -i ./downward/plan.py
    sed -e s/"search\/"//g -i ./downward/plan.py
    ./build
    cd -
    rm -r "${tfd}.tgz"
else
    echo "Folder $folder/${tfd} already exists. Skipping installation."
fi

#Add environment variable to bashrc
export TFD_HOME=$(pwd)
string="export TFD_HOME=$(pwd)"
string2="export PATH=$""TFD_HOME/${tfd}/downward:$""TFD_HOME/${tfd}/downward/translate:$""TFD_HOME/${tfd}/downward/preprocess:$""TFD_HOME/${tfd}/downward/search:""$""PATH"
temp=$(cat ~/.bashrc | grep "$string")
if [ -z "$temp" ]; then
	echo $string >> ~/.bashrc
	echo $string2 >> ~/.bashrc
	echo "Printing string in .bashrc: $string"
else
	echo "Found planner path in bashrc: $string. Skip printing."
fi

# source ~/.bashrc
