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
        string sourFilePath;
        string tarFilePath;
        pointCloudPtr cloudSour;
        pointCloudPtr cloudTar;
        pointCloudPtr cloudSourDs;
        pointCloudPtr cloudTarDs;
        fpfhPtr sourFpfh;
        fpfhPtr tarFpfh;
        float filterSize;
        float dthr;
        float pointThre;//计算点变换是否正确的阈值
        int k1;
        int k2;
        Eigen::Matrix4f transMatrix;
        Eigen::MatrixXf scMat;
        Eigen::MatrixXd seedHardMat;
        Eigen::MatrixXd sc2Mat;

        void init();
        void computeFpfh(pointCloudPtr cloud, fpfhPtr fpfh);
        void matchPair(pointCloudPtr sour, pointCloudPtr tar, fpfhPtr sourFpfh, fpfhPtr tarFpfh, vector<pair<int, int>> &corres);
        void dsCloud(float filterSize);

    public:
        PCR(char **argv);
        bool lodaData();
        void registration();
        void pickSeeds(const Eigen::MatrixXf &scMat, vector<int> &seeds, int sn);
        Eigen::Matrix4f getBestTrans();
        void readConfig(string fileName);
        void calScMat(Eigen::MatrixXf &scMat, const pointCloudPtr sour, const pointCloudPtr tar, const vector<pair<int, int>> &corres);
        void calScHardMat(Eigen::MatrixXd &hardMat, const Eigen::MatrixXf &scMat, const vector<int> &seeds, float dthr);
        void calSc2Mat(Eigen::MatrixXd &sc2Mat, const Eigen::MatrixXd &hardMat);
        Eigen::Matrix4f calBestTrans(const pointCloudPtr sour, const pointCloudPtr tar, const Eigen::MatrixXd &sc2Mat, const vector<int> &seeds, const vector<pair<int, int>> &corres);
        Eigen::Matrix4f calTrans(const pointCloudPtr sour, const pointCloudPtr tar, const Eigen::MatrixXd &sc2Mat,const vector<pair<int, int>> &corres, const vector<int> &seeds, const int index);
    };
}