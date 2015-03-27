cd ~/

git clone https://github.com/UMEECS467W15/eecs467.git

sudo apt-get -y install emacs vim-gtk cvs subversion git-core gitk units octave imagemagick ssh smartmontools nmap netcat curl wireshark traceroute libav-tools mplayer vlc sysstat valgrind autopoint xchat mesa-utils pkg-config curl autoconf automake autotools-dev libglib2.0-dev manpages-dev manpages-posix-dev libgl1-mesa-dev gtk-doc-tools libgtk2.0-dev python-dev libusb-dev libusb-1.0-0-dev libfuse-dev libi2c-dev libdc1394-22-dev libdc1394-utils libgsl0-dev gsl-doc-info gsl-doc-pdf ant openjdk-6-jdk

cd ~/
git clone https://code.google.com/p/lcm/
cd lcm && ./bootstrap.sh && ./configure && make && sudo make install
sudo ldconfig

sudo cp ~/eecs467/misc/99-eecs467.rules /etc/udev/rules.d/.

cd ~/eecs467
make clean && make
