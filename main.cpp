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
    // srand((unsigned)time(NULL));
	// Eigen::MatrixXd randvalue = (Eigen::MatrixXd::Random(4,4)).array().abs()  *2*M_PI;
	// std::cout << randvalue << std::endl;
	// cout << endl;
	// Eigen::MatrixXf randvalue2 = Eigen::MatrixXf::Random(100, 1).array().abs();
	// std::cout << randvalue2 << std::endl;
    return 0;
}