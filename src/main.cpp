#include"FPFHPCR.h"

int main(int argc, char *argv[])
{
    sc2pcr::FPFHPCR pcr(argv);
    pcr.registration();
    
    return 0;
}