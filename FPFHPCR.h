#include "pcr.h"

namespace sc2pcr
{
    class FPFHPCR : public PCR
    {
    private:
        fpfhPtr sourFpfh;       //存储sour的Fpfh特征
        fpfhPtr tarFpfh;        //存储tar的Fpfh特征
        float normalSizeTimes; //计算FPFH特征时，每个点法向量的搜索范围=filterSize*normalSizeTimes
        float fpfhSizeTimes;   //计算FPFH特征时，每个点FPFH的搜索范围=filterSize*fpfhSizeTimes
        void computeFpfh(const pointCloudPtr cloud, fpfhPtr fpfh);

    public:
        void init();
        void calFeature();
        void matchPair();
        void registration();
        FPFHPCR(char **argv) : PCR(argv) {}
    };
}