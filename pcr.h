#include <unordered_map>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigen>

using namespace std;

namespace sc2pcr
{
    typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr;
    typedef pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhPtr;
    typedef pcl::PointCloud<pcl::PointNormal> pointnormal;
    typedef flann::Index<flann::L2<float>> KDTree;

    class PCR
    {
    private:
        string configFile;             //存储config路径
        string sourFilePath;           //存储Source点云路径
        string tarFilePath;            //存储Target点云路径
        pointCloudPtr cloudSour;       //文件输入Source点云
        pointCloudPtr cloudTar;        //文件输入Target点云
        pointCloudPtr cloudSourDs;     // source降采样点云
        pointCloudPtr cloudTarDs;      // target降采样点云
        pointCloudPtr sour;            //被处理的点云
        pointCloudPtr tar;             //被处理的点云
        fpfhPtr sourFpfh;              //存储sour的Fpfh特征
        fpfhPtr tarFpfh;               //存储tar的Fpfh特征
        Eigen::Matrix4f transMatrix;   // transMatrix*sour=tar
        Eigen::MatrixXf scMat;         //一阶兼容矩阵
        Eigen::MatrixXd seedHardMat;   //种子点二值化SC矩阵
        Eigen::MatrixXd sc2Mat;        //二阶兼容矩阵
        float filterSize;              //降采样grid大小
        float normalSizeTimes;         //计算FPFH特征时，每个点法向量的搜索范围=filterSize*normalSizeTimes
        float fpfhSizeTimes;           //计算FPFH特征时，每个点FPFH的搜索范围=filterSize*fpfhSizeTimes
        float dthr;                    //二值化SC矩阵时用于判断是否相似的阈值
        float pointThre;               //计算corres中点变换是否正确的阈值
        float seedsRatio;              //种子点选择比例
        float radius;                  //种子点采样时，在radius范围内只采样一个点，非极大线性抑制
        int k1;                        //每个种子点扩充时第一阶段选择corres个数
        int k2;                        //扩充时第二阶段选择corres个数
        int leadVecIter;               //主向量
        vector<pair<int, int>> corres; //存储匹配的FPFH特征对，值表示点云序列号
        vector<int> seeds;             //存储种子点 其值指在scMat中的序列号

        void init();                                                                                                              //初始化数据
        void computeFpfh(pointCloudPtr cloud, fpfhPtr fpfh);                                                                      //计算FPFH特征
        void matchPair(pointCloudPtr sour, pointCloudPtr tar, fpfhPtr sourFpfh, fpfhPtr tarFpfh, vector<pair<int, int>> &corres); //匹配两个点云的FPFH特征
        void dsCloud(float filterSize);                                                                                           //降采样点云

    public:
        PCR(char **argv);
        bool lodaData();                                                                                                                                                   //加载点云数据
        void registration();                                                                                                                                               //注册点云
        void pickSeeds(const pointCloudPtr sour, const pointCloudPtr tar, const Eigen::MatrixXf &scMat, vector<int> &seeds, const vector<pair<int, int>> &corres, int sn); //选择种子点
        Eigen::Matrix4f getBestTrans();
        void readConfig(string fileName);
        void calScMat(Eigen::MatrixXf &scMat, const pointCloudPtr sour, const pointCloudPtr tar, const vector<pair<int, int>> &corres);
        void calScHardMat(Eigen::MatrixXd &hardMat, const Eigen::MatrixXf &scMat, const vector<int> &seeds);
        void calSc2Mat(Eigen::MatrixXd &sc2Mat, const Eigen::MatrixXd &hardMat);
        Eigen::Matrix4f calBestTrans(const pointCloudPtr sour, const pointCloudPtr tar, const Eigen::MatrixXd &sc2Mat, const vector<int> &seeds, const vector<pair<int, int>> &corres);
        Eigen::Matrix4f calTrans(const pointCloudPtr sour, const pointCloudPtr tar, const Eigen::MatrixXd &sc2Mat, const vector<pair<int, int>> &corres, const vector<int> &seeds, const int index);
    };
}