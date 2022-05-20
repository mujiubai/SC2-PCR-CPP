#include "pcr.h"

#include <string>

using namespace sc2pcr;
using namespace std;

void PCR::registration()
{
    vector<pair<int, int>> corres;
    vector<int> seeds; //存储种子点 其值指在scMat中的序列号
    pointCloudPtr sour, tar;

    init();
    lodaData();
    dsCloud(filterSize);
    sour = this->cloudSourDs;
    tar = this->cloudTarDs;
    computeFpfh(sour, sourFpfh);
    computeFpfh(tar, tarFpfh);
    matchPair(sour, tar, sourFpfh, tarFpfh, corres);
    calScMat(this->scMat, sour, tar, corres);
    pickSeeds(sour, tar, scMat, seeds, corres, this->seedsRatio * scMat.cols());
    calScHardMat(this->seedHardMat, scMat, seeds);
    calSc2Mat(this->sc2Mat, seedHardMat);

    Eigen::Matrix4f trans = calBestTrans(sour, tar, sc2Mat, seeds, corres);
    cout << trans << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSourTrans(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloudSour, *cloudSourTrans, trans);
    *cloudSourTrans += *cloudTar;
    pcl::io::savePLYFileBinary("cloud_out.ply", *cloudSourTrans);
}

PCR::PCR(char **argv)
{
    string configFile = argv[1];
    readConfig(configFile);
}

bool readConfigFile(string fileName, unordered_map<string, string> &confMap)
{
    fstream cfgFile;
    cfgFile.open(fileName.c_str()); //打开文件
    if (!cfgFile.is_open())
    {
        cout << "can not open cfg file!" << endl;
        return false;
    }
    char tmp[100];
    while (!cfgFile.eof()) //循环读取每一行
    {
        cfgFile.getline(tmp, 100); //每行读取前1000个字符，1000个应该足够了
        string line(tmp);
        size_t pos = line.find('='); //找到每行的“=”号位置，之前是key之后是value
        if (pos == string::npos)
            return false;

        string tmpKey = line.substr(0, pos);                                //取=号之前
        string tempValue = line.substr(pos + 1, line.find('\r') - pos - 1); //取=号之后
        confMap.insert(pair<string, string>(tmpKey, tempValue));
    }
    return true;
}

void PCR::readConfig(string fileName)
{
    unordered_map<string, string> confMap;
    readConfigFile(fileName, confMap);
    this->sourFilePath = confMap.find("cloudSour") != confMap.end() ? confMap["cloudSour"] : "source.las";
    this->tarFilePath = confMap.find("cloudTarg") != confMap.end() ? confMap["cloudTarg"] : "target.las";
    this->filterSize = confMap.find("filterSize") != confMap.end() ? atof(confMap["filterSize"].c_str()) : 1;
    this->dthr = confMap.find("dthr") != confMap.end() ? atof(confMap["dthr"].c_str()) : 1;
    this->pointThre = confMap.find("pointThre") != confMap.end() ? atof(confMap["pointThre"].c_str()) : 1;
    this->k1 = confMap.find("k1") != confMap.end() ? atoi(confMap["k1"].c_str()) : 30;
    this->k2 = confMap.find("k2") != confMap.end() ? atoi(confMap["k2"].c_str()) : 20;
    this->leadVecIter = confMap.find("leadVecIter") != confMap.end() ? atoi(confMap["leadVecIter"].c_str()) : 20;
    this->seedsRatio = confMap.find("seedsRatio") != confMap.end() ? atof(confMap["seedsRatio"].c_str()) : 0.1;
    this->radius = confMap.find("radius") != confMap.end() ? atof(confMap["radius"].c_str()) : 1;
    printf("config data load success!\n");
}

void PCR::init()
{
    this->cloudSour.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->cloudTar.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->cloudSourDs.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->cloudTarDs.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->sourFpfh.reset(new pcl::PointCloud<pcl::FPFHSignature33>());
    this->tarFpfh.reset(new pcl::PointCloud<pcl::FPFHSignature33>());
    this->transMatrix.Identity();
}

bool PCR::lodaData()
{
    string fileName = this->sourFilePath;
    if (fileName.find(".pcd") != string::npos)
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(sourFilePath, *cloudSour) == -1)
        {
            printf("read %s failed", sourFilePath.c_str());
            return false;
        }
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(tarFilePath, *cloudTar) == -1)
        {
            printf("read %s failed", tarFilePath.c_str());
            return false;
        }
    }
    else if (fileName.find(".ply") != string::npos)
    {
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(sourFilePath, *cloudSour) == -1)
        {
            printf("read %s failed", sourFilePath.c_str());
            return false;
        }
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(tarFilePath, *cloudTar) == -1)
        {
            printf("read %s failed", tarFilePath.c_str());
            return false;
        }
    }
    else
    {
        printf("the file format is not pcd or las, wrong!!");
        return false;
    }
    printf("data load finish!\n");
    return true;
}

void PCR::computeFpfh(pointCloudPtr cloud, fpfhPtr fpfh)
{
    pointnormal::Ptr point_normal(new pointnormal);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> est_normal;
    est_normal.setNumberOfThreads(8);
    est_normal.setInputCloud(cloud);
    est_normal.setRadiusSearch(0.2);
    est_normal.compute(*point_normal);

    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::PointNormal, pcl::FPFHSignature33> fest;
    fest.setNumberOfThreads(8);
    fest.setRadiusSearch(0.5);
    fest.setInputCloud(cloud);
    fest.setInputNormals(point_normal);
    fest.compute(*fpfh);

    printf("compute the cloud FPFH feature finish.\n");
}

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

    indices.clear();
    dists.clear();
    indices.resize(rows_t * nn);
    dists.resize(rows_t * nn);
    flann::Matrix<int> indices_mat(&indices[0], rows_t, nn);
    flann::Matrix<float> dists_mat(&dists[0], rows_t, nn);

    tree->knnSearch(query_mat, indices_mat, dists_mat, nn, flann::SearchParams(128));
}

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

void PCR::matchPair(pointCloudPtr sour, pointCloudPtr tar, fpfhPtr sourFpfh, fpfhPtr tarFpfh, vector<pair<int, int>> &corres)
{
    KDTree sourFeaTree(flann::KDTreeSingleIndexParams(15));
    KDTree tarFeaTree(flann::KDTreeSingleIndexParams(15));
    vector<int> indices;
    vector<float> dists;
    BuildKDTree(sourFpfh, &sourFeaTree);
    BuildKDTree(tarFpfh, &tarFeaTree);
    // corres.resize(sour->size());
    for (int i = 0; i < sour->size(); ++i)
    {
        SearchKDTree(&tarFeaTree, sourFpfh, i, indices, dists, 1);
        int j = indices[0];
        SearchKDTree(&sourFeaTree, tarFpfh, j, indices, dists, 1);
        if(i==indices[0]){
            corres.push_back(pair<int,int>(i,j));
        }
        // corres[i].first = i;
        // corres[i].second = j;
    }
    printf("match pair success!\n");
}

void cloudFilter(pointCloudPtr cloud, pointCloudPtr cloudOut, float leafSize)
{
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(leafSize, leafSize, leafSize);
    filter.filter(*cloudOut);
}

void PCR::dsCloud(float filterSize)
{
    cloudFilter(cloudSour, cloudSourDs, filterSize);
    cloudFilter(cloudTar, cloudTarDs, filterSize);
    printf("DownSample point cloud success, the filter size is %f \n", filterSize);
}

template <class T>
void calLeadEigenVec(const T &Mat, vector<float> &vec, const int iterNum)
{
    // int maxVal = INT_MIN, maxInde = -1;
    int n = Mat.cols();
    vec.resize(n);

    // Eigen::EigenSolver<T> solver(Mat);

    // T D = solver.pseudoEigenvalueMatrix();
    // T V = solver.pseudoEigenvectors();

    // for (int i = 0; i < n; ++i)
    // {
    //     if (maxVal < D(i, i))
    //     {
    //         maxVal = D(i, i);
    //         maxInde = i;
    //     }
    // }

    // for (int i = 0; i < n; ++i)
    // {
    //     vec[i] = V(maxInde, i);
    // }

    Eigen::MatrixXf V = Eigen::MatrixXf::Random(n, 1).array().abs();
    for (int i = 0; i < iterNum; ++i)
    {
        V = Mat * V;
        double sum = 0;
        for (int j = 0; j < n; ++j)
        {
            sum += pow(V(j, 0), 2);
        }
        for (int j = 0; j < n; ++j)
        {
            V(j, 0) /= sum;
        }
    }

    for (int i = 0; i < n; ++i)
    {
        vec[i] = V(i, 0);
    }
}

bool myCompareVec(pair<float, int> &a, pair<float, int> &b)
{
    return a.first > b.first;
}

void PCR::pickSeeds(const pointCloudPtr sour, const pointCloudPtr tar, const Eigen::MatrixXf &scMat, vector<int> &seeds, const vector<pair<int, int>> &corres, int sn)
{
    seeds.resize(sn);
    vector<float> leadVec;
    calLeadEigenVec<Eigen::MatrixXf>(scMat, leadVec, leadVecIter);
    //从特征向量中选择最大的sn个点，且一定范围内不重复选点
    vector<pair<float, int>> sortLeadVec(leadVec.size());
    for (int i = 0; i < leadVec.size(); ++i)
    {
        sortLeadVec[i].first = leadVec[i];
        sortLeadVec[i].second = i;
    }
    sort(sortLeadVec.begin(), sortLeadVec.end(), myCompareVec); //这里是直接比较大小，没考虑绝对值

    //非极大线性抑制
    for (int i = 0; i < sortLeadVec.size(); ++i)
    {
        if (sortLeadVec[i].first == 0)
            continue;
        pcl::PointXYZ &a = sour->points[corres[sortLeadVec[i].second].first];
        for (int j = i + 1; j < sortLeadVec.size(); ++j)
        {
            pcl::PointXYZ &b = sour->points[corres[sortLeadVec[j].second].first];
            double dd = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
            if (dd <= radius)
            {
                sortLeadVec[j].first = 0;
            }
        }
    }

    //经过非极大线性抑制后再次排序，并取前sn个为种子点
    sort(sortLeadVec.begin(), sortLeadVec.end(), myCompareVec);
    for (int i = 0; i < sn; ++i)
    {
        seeds[i] = sortLeadVec[i].second;
    }

    printf("pick seed pair success! The number of seeds is %d\n", sn);
}

double calDis(const pcl::PointXYZ &p, const pcl::PointXYZ &q)
{
    return sqrt(pow(p.x - q.x, 2) + pow(p.y - q.y, 2) + pow(p.z - q.z, 2));
}

void PCR::calScMat(Eigen::MatrixXf &scMat, const pointCloudPtr sour, const pointCloudPtr tar, const vector<pair<int, int>> &corres)
{
    int n = corres.size();
    scMat.resize(n, n);

    for (int i = 0; i < n; ++i)
    {
        scMat(i, i) = 0;
        for (int j = i + 1; j < n; ++j)
        {
            double dis1 = calDis(sour->points[corres[i].first], sour->points[corres[j].first]);
            double dis2 = calDis(sour->points[corres[i].second], sour->points[corres[j].second]);
            double dis = abs(dis1 - dis2);
            double score = 1 - pow(dis, 2) / pow(this->dthr, 2);
            scMat(i, j) = score < 0 ? 0 : score;
            scMat(j, i) = scMat(i, j);
        }
    }
    printf("calculate first order sc matrix success!\n");
}

template <typename T>
void calLeadEigenVec(vector<vector<T>> &Mat, vector<float> &v)
{
}

void PCR::calSc2Mat(Eigen::MatrixXd &sc2Mat, const Eigen::MatrixXd &hardMat)
{
    int n = hardMat.cols();
    sc2Mat.resize(n, n);
    for (int i = 0; i < n; ++i)
    {
        for (int j = i + 1; j < n; ++j)
        {
            sc2Mat(i, j) = 0;
            for (int k = 0; k < n; ++k)
            {
                sc2Mat(i, j) += hardMat(i, k) * hardMat(k, j);
                sc2Mat(j, i) = sc2Mat(i, j);
            }
        }
    }
    printf("calculate the second order SC matrix success!\n");
}

void PCR::calScHardMat(Eigen::MatrixXd &hardMat, const Eigen::MatrixXf &scMat, const vector<int> &seeds)
{
    int m = scMat.cols(), sn = seeds.size();
    hardMat.resize(sn, sn);
    for (int i = 0; i < sn; ++i)
    {
        for (int j = i; j < sn; ++j)
        {
            hardMat(i, j) = scMat(seeds[i], seeds[j]) <= 0 ? 0 : 1;
            hardMat(j, i) = hardMat(i, j);
        }
    }
}

bool compareCor(pair<int, int> &a, pair<int, int> &b)
{
    return a.first > b.first;
}

Eigen::Matrix4f PCR::calTrans(const pointCloudPtr sour, const pointCloudPtr tar, const Eigen::MatrixXd &sc2Mat, const vector<pair<int, int>> &corres, const vector<int> &seeds, const int index)
{
    int n = sc2Mat.cols();
    int fn = this->k1;
    int sn = this->k2;
    vector<int> firstColIndex(fn); //存储第一阶段共识集合index
    vector<int> secondColIndex(sn);
    vector<pair<int, int>> firstCorr(n); //存储待排序的值 相似度，位置
    vector<pair<int, int>> secondCorr(fn);
    Eigen::MatrixXf localScMat(fn, fn);
    Eigen::MatrixXd localHardMat(fn, fn);
    Eigen::MatrixXd localSc2Mat(fn, fn);
    Eigen::MatrixXf leadVec;
    Eigen::MatrixXf cloudP(3, sn), cloudQ(3, sn);
    Eigen::MatrixXf H(3, 3);
    Eigen::Matrix4f trans;  // trans*P=Q
    Eigen::Matrix3f transR; // transR*p+transT=Q
    Eigen::MatrixXf transT(3, 1);
    Eigen::MatrixXf temP, temQ;
    double meanPX = 0, meanPY = 0, meanPZ = 0;
    double meanQX = 0, meanQY = 0, meanQZ = 0;

    //第一阶段，排序，取前k1个数作为共识集合
    for (int i = 0; i < n; ++i)
    {
        firstCorr[i].first = sc2Mat(index, i);
        firstCorr[i].second = i;
    }
    sort(firstCorr.begin(), firstCorr.end(), compareCor);
    firstColIndex[0] = seeds[index];
    for (int i = 1; i < fn; ++i)
    {
        firstColIndex[i] = seeds[firstCorr[i - 1].second];
    }

    //计算localScMat、localHardMat、localSc2Mat
    for (int i = 0; i < fn; ++i)
    {
        for (int j = i + 1; j < fn; ++j)
        {
            localScMat(i, j) = this->scMat(firstColIndex[i], firstColIndex[j]);
            localScMat(j, i) = localScMat(i, j);
            localHardMat(i, j) = localScMat(i, j) <= 0 ? 0 : 1;
            localHardMat(j, i) = localHardMat(i, j);
        }
    }
    for (int i = 0; i < fn; ++i)
    {
        for (int j = i + 1; j < fn; ++j)
        {
            localSc2Mat(i, j) = 0;
            for (int l = 0; l < fn; ++l)
            {
                localSc2Mat(i, j) += localHardMat(i, l) * localHardMat(l, j);
                localSc2Mat(j, i) = localSc2Mat(i, j);
            }
        }
    }

    //第二阶段，取前k2个
    for (int i = 0; i < fn; ++i)
    {
        secondCorr[i].first = localSc2Mat(0, i);
        secondCorr[i].second = i;
    }
    sort(secondCorr.begin(), secondCorr.end(), compareCor);
    secondColIndex[0] = index;
    for (int i = 1; i < sn; ++i)
    {
        secondColIndex[i] = firstColIndex[secondCorr[i - 1].second];
    }

    //求权重

    // svd分解计算变换矩阵
    for (int i = 0; i < sn; ++i)
    {
        pcl::PointXYZ &ts = sour->points[corres[secondColIndex[i]].first];
        pcl::PointXYZ &tt = tar->points[corres[secondColIndex[i]].second];
        cloudP(0, i) = ts.x;
        cloudP(1, i) = ts.y;
        cloudP(2, i) = ts.z;
        cloudQ(0, i) = tt.x;
        cloudQ(1, i) = tt.y;
        cloudQ(2, i) = tt.z;
        meanPX += ts.x;
        meanPY += ts.y;
        meanPZ += ts.z;
        meanQX += tt.x;
        meanQY += tt.y;
        meanQZ += tt.z;
    }
    temP = cloudP.col(0);
    temQ = cloudQ.col(0);
    meanPX /= sn;
    meanPY /= sn;
    meanPZ /= sn;
    meanQX /= sn;
    meanQY /= sn;
    meanQZ /= sn;
    for (int i = 0; i < sn; ++i)
    {
        cloudP(0, i) -= meanPX;
        cloudP(1, i) -= meanPY;
        cloudP(2, i) -= meanPZ;
        cloudQ(0, i) -= meanQX;
        cloudQ(1, i) -= meanQY;
        cloudQ(2, i) -= meanQZ;
    }
    H = cloudP * cloudQ.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXf> svdH(H, Eigen::DecompositionOptions::ComputeFullU | Eigen::DecompositionOptions::ComputeFullV);
    transR = svdH.matrixV() * svdH.matrixU().transpose();
    transT = temQ - transR * temP;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            trans(i, j) = transR(i, j);
        }
        trans(3, i) = 0;
        trans(i, 3) = transT(i);
    }
    trans(3, 3) = 1;

    return trans;
}

int calCount(const pointCloudPtr sour, const pointCloudPtr tar, const vector<int> &seeds, Eigen::Matrix4f &trans, const vector<pair<int, int>> &corres, float thre)
{
    int count = 0;
    for (int i = 0; i < seeds.size(); ++i)
    {
        pcl::PointXYZ ts = sour->points[corres[seeds[i]].first];
        pcl::PointXYZ tt = tar->points[corres[seeds[i]].second];
        float dx = trans(0, 0) * ts.x + trans(0, 1) * ts.y + trans(0, 2) * ts.z + trans(0, 3) - tt.x;
        float dy = trans(1, 0) * ts.x + trans(1, 1) * ts.y + trans(1, 2) * ts.z + trans(1, 3) - tt.y;
        float dz = trans(2, 0) * ts.x + trans(2, 1) * ts.y + trans(2, 2) * ts.z + trans(2, 3) - tt.z;
        if (sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2)) < thre)
        {
            ++count;
        }
    }

    return count;
}

Eigen::Matrix4f PCR::calBestTrans(const pointCloudPtr sour, const pointCloudPtr tar, const Eigen::MatrixXd &sc2Mat, const vector<int> &seeds, const vector<pair<int, int>> &corres)
{
    int n = seeds.size();
    int maxCount = -1, index = -1;
    Eigen::Matrix4f finalTrans;

    for (int i = 0; i < n; ++i)
    {
        Eigen::Matrix4f curTrans = calTrans(sour, tar, sc2Mat, corres, seeds, i);
        int count = calCount(sour, tar, seeds, curTrans, corres, this->pointThre);
        if (count >= maxCount)
        {
            maxCount = count;
            finalTrans = curTrans;
            index = i;
        }
    }
    printf("max count:%d index:%d pointThre:%f\n ", maxCount, index, this->pointThre);
    return finalTrans;
}
