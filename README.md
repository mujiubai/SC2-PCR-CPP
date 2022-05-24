# SC2-PCR-CPP
SC2-PCR c++ version

required:
    PCL
    Eigen

linux compile:
    git clone https://github.com/mujiubai/SC2-PCR-CPP.git
    cd SC2-PCR-CPP &&mkdir build &&cd build
    cmake ..
    make

run:
    ./sc2pcr config.txt
