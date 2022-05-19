#include <iostream>
#include "pcr.h"

void test(Eigen::MatrixXf &t){
    t.resize(5,6);

    return ;
}

int main(int argc, char *argv[])
{
    Eigen::MatrixXf a;
    test(a);
    std::cout<<a<<endl;
    return 0;
}