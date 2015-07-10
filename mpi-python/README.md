*********************
#1. How to install mpi4py on Ubuntu 
All The prerequisites can be installed with a couple of commands on Ubuntu. The only choice to make is between openmpi and mpich2. Most of our testing is done with MPICH2 but openmpi should also work.
For openmpi do:  <br />
> sudo apt-get install build-essential gfortran python-dev \
  libopenmpi-dev openmpi-bin \
  libgsl0-dev cmake libfftw3-3 libfftw3-dev \
  libgmp3-dev libmpfr4 libmpfr-dev \
  libhdf5-serial-dev hdf5-tools \
  python-nose python-numpy python-setuptools python-docutils \
  python-h5py python-setuptools <br />

> sudo easy_install mpi4py 

<br />
<br />

For mpich2 do: <br />
> sudo apt-get install build-essential gfortran python-dev \
  mpich2 libmpich2-dev \
  libgsl0-dev cmake libfftw3-3 libfftw3-dev \
  libgmp3-dev libmpfr4 libmpfr-dev \
  libhdf5-serial-dev hdf5-tools \
  python-nose python-numpy python-setuptools python-docutils \
  python-h5py python-setuptools

*********************
#2. How to run hello.py 
After saving a program such as  hello.py, it is executed using the following command-line syntax, run from the file’s directory:
$ mpiexec  -n 5 python  hello.py
