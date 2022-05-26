#include "FPFHPCR.h"

using namespace sc2pcr;
using namespace std;

void FPFHPCR::calFeature()
{
    computeFpfh(sour, sourFpfh);
    computeFpfh(tar, tarFpfh);
}

void FPFHPCR::computeFpfh(const pointCloudPtr cloud, fpfhPtr fpfh)
{
    //计算点的法向量
    pointnormal::Ptr point_normal(new pointnormal);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> est_normal;
    est_normal.setNumberOfThreads(8); //使用核心数
    est_normal.setInputCloud(cloud);
    est_normal.setRadiusSearch(filterSize * normalSizeTimes); //设置搜索范围
    est_normal.compute(*point_normal);
    //计算FPFH特征
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::PointNormal, pcl::FPFHSignature33> fest;
    fest.setNumberOfThreads(8);
    fest.setRadiusSearch(filterSize * fpfhSizeTimes);
    fest.setInputCloud(cloud);
    fest.setInputNormals(point_normal);
    fest.compute(*fpfh);
    // printf("%f %f\n",filterSize*normalSizeTimes,filterSize*fpfhSizeTimes);
    printf("compute the cloud FPFH feature finish.\n");
}

//搜索FPFH特征最近邻
void SearchKDTree(KDTree *tree, const fpfhPtr &input, int index,
                  std::vector<int> &indices,
                  std::vector<float> &dists, int nn)
{
    int rows_t = 1;
    int dim = 33;

    std::vector<float> query;
    query.resize(rows_t * dim);
    for (int i = 0; i < dim; i++)
        query[i] = input->points[index].histogram[i];
    flann::Matrix<float> query_mat(&query[0], rows_t, dim);

    indices.resize(rows_t * nn);
    dists.resize(rows_t * nn);
    flann::Matrix<int> indices_mat(&indices[0], rows_t, nn);
    flann::Matrix<float> dists_mat(&dists[0], rows_t, nn);

    tree->knnSearch(query_mat, indices_mat, dists_mat, nn, flann::SearchParams(128));
}

//建立FPFH特征的KDtree
void BuildKDTree(const fpfhPtr &data, KDTree *tree)
{
    int rows, dim;
    rows = (int)data->size();
    dim = 33;
    std::vector<float> dataset(rows * dim);
    flann::Matrix<float> dataset_mat(&dataset[0], rows, dim);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < dim; j++)
            dataset[i * dim + j] = data->points[i].histogram[j];
    KDTree temp_tree(dataset_mat, flann::KDTreeSingleIndexParams(15));
    temp_tree.buildIndex();
    *tree = temp_tree;
}

void FPFHPCR::matchPair()
{
    KDTree sourFeaTree(flann::KDTreeSingleIndexParams(15));
    KDTree tarFeaTree(flann::KDTreeSingleIndexParams(15));
    vector<int> indices;
    vector<float> dists;
    BuildKDTree(sourFpfh, &sourFeaTree);
    BuildKDTree(tarFpfh, &tarFeaTree);
    corres.reserve(sour->size());
    //只有当都互为最近邻时 才添加为corres
    vector<int> corres_st(sour->size(), -1), corres_ts(tar->size(), -1);
    for (int i = 0; i < sour->size(); ++i)
    {
        SearchKDTree(&tarFeaTree, sourFpfh, i, indices, dists, 1);
        int j = indices[0];

        if (corres_ts[j] == -1)
        {
            SearchKDTree(&sourFeaTree, tarFpfh, j, indices, dists, 1);
            corres_ts[j] = indices[0];
            corres.push_back(pair<int, int>(i, j));
        }
    }

    printf("match pair success,the number of pairs is %d \n", int(corres.size()));
}

void FPFHPCR::setCorres(){
    calFeature();
    matchPair();
}

void FPFHPCR::registration()
{

    PCR::registration();
    
    cout << transMatrix << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSourTrans(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloudSour, *cloudSourTrans, transMatrix);
    pcl::io::savePLYFileBinary("cloudSour.ply", *cloudSourTrans);
    *cloudSourTrans += *cloudTar;
    pcl::io::savePLYFileBinary("cloud_out.ply", *cloudSourTrans);
}

void FPFHPCR::init()
{
    PCR::init();
    sourFpfh.reset(new pcl::PointCloud<pcl::FPFHSignature33>());
    tarFpfh.reset(new pcl::PointCloud<pcl::FPFHSignature33>());
    config->setConfig(normalSizeTimes, "normalSizeTimes");
    config->setConfig(fpfhSizeTimes, "fpfhSizeTimes");
    printf("config test\n");
}
