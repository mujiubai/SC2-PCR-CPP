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

    class ConfigRead
    {
        unordered_map<string, string> confMap;
        string configFile;

    public:
        void readConfigFile(string fileName);
        bool setConfig(double &conf, string name);
        bool setConfig(float &conf, string name);
        bool setConfig(int &conf, string name);
        bool setConfig(string &conf, string name);

        ConfigRead(string filename) : configFile(filename)
        {
            readConfigFile(configFile);
        }
    };

    typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr;
    typedef pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhPtr;
    typedef pcl::PointCloud<pcl::PointNormal> pointnormal;
    typedef flann::Index<flann::L2<float>> KDTree;

    class PCR
    {
    protected:
        string configFile;           //存储config路径
        string sourFilePath;         //存储Source点云路径
        string tarFilePath;          //存储Target点云路径
        pointCloudPtr cloudSour;     //文件输入Source点云
        pointCloudPtr cloudTar;      //文件输入Target点云
        pointCloudPtr cloudSourDs;   // source降采样点云
        pointCloudPtr cloudTarDs;    // target降采样点云
        pointCloudPtr sour;          //被处理的点云
        pointCloudPtr tar;           //被处理的点云
        Eigen::Matrix4f transMatrix; // transMatrix*sour=tar
        Eigen::MatrixXf scMat;       //一阶兼容矩阵
        Eigen::MatrixXd seedHardMat; //种子点二值化SC矩阵
        Eigen::MatrixXd sc2Mat;      //二阶兼容矩阵
        Eigen::MatrixXf weights;
        float filterSize;              //降采样grid大小
        float dthr;                    //二值化SC矩阵时用于判断是否相似的阈值
        float pointThre;               //计算corres中点变换是否正确的阈值
        float seedsRatio;              //种子点选择比例
        float radius;                  //种子点采样时，在radius范围内只采样一个点，非极大线性抑制
        int k1;                        //每个种子点扩充时第一阶段选择corres个数
        int k2;                        //扩充时第二阶段选择corres个数
        int leadVecIter;               //主向量
        vector<pair<int, int>> corres; //存储匹配的FPFH特征对，值表示点云序列号
        vector<int> seeds;             //存储种子点 其值指在scMat中的序列号
        ConfigRead *config;

        virtual void init(); //初始化数据
        void dsCloud();      //降采样点云

    public:
        PCR(char **argv);
        bool lodaData();             //加载点云数据
        virtual void registration(); //注册点云
        void pickSeeds();            //选择种子点
        Eigen::Matrix4f getBestTrans();
        void readConfig();
        void calScMat();
        void calScHardMat();
        void calSc2Mat();
        void calWeight(vector<int> &Col);
        Eigen::Matrix4f calBestTrans();
        Eigen::Matrix4f calTrans(const int index);
        virtual void setCorres() = 0; //子类必须实现找corres
    };

}