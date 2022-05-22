#include <iostream>
#include "pcr.h"

void test(Eigen::MatrixXf &t){
    t.resize(5,6);

    return ;
}

int main(int argc, char *argv[])
{
    sc2pcr::PCR pcr(argv);
    pcr.registration();
    
    return 0;
}