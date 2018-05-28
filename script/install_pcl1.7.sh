#!/bin/bash


system_platform=$(uname)
distribution_name=$(lsb_release -is)
release_version=$(lsb_release -rs)

# if the linux distribution is LINUX
if [ "$system_platform" = "Linux" ]
then
	# check whether the distribution is Ubuntu
	if [ "$distribution_name" = "Ubuntu" ]
	then
		# First make sure everything is up to date
		sudo apt-get update
		sudo apt-get upgrade
		# check what version of ubuntu version is it
		if [ "$release_version" = "16.04" ]
		then
			echo "System detected: $distribution_name-$release_version"
			sudo apt-get install -y libpcl-dev
		elif [ "$release_version" = "14.04" ]
		then
			sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
			sudo apt-get update
			sudo apt-get install -y libpcl-all
		fi

		sudo ldconfig
	fi
elif [ "$system_platform" = "Darwin" ]
then
	# install homebrew
	/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"

	# install PCL via homebrew
	brew update
	brew install pcl

else
	echo "Window"
fi