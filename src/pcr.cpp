#include "pcr.h"
#include <string>

using namespace sc2pcr;
using namespace std;

void PCR::registration()
{
    init();
    lodaData();
    dsCloud();
    if (sour == nullptr || tar == nullptr || sour->size() == 0 || tar->size() == 0 )
    {
        printf("input data error!!\n");
        exit(1);
    }
    setCorres();
    calScMat();
    pickSeeds();
    calScHardMat();
    calSc2Mat();

    transMatrix = calBestTrans();
}

PCR::PCR(char **argv) : configFile(argv[1])
{
    config = new ConfigRead(configFile);
}

//读取配置文件
void PCR::readConfig()
{
    config->setConfig(sourFilePath, "cloudSour");
    config->setConfig(tarFilePath, "cloudTarg");
    config->setConfig(filterSize, "filterSize");
    config->setConfig(dthr, "dthr");
    config->setConfig(pointThre, "pointThre");
    config->setConfig(k1, "k1");
    config->setConfig(k2, "k2");
    config->setConfig(leadVecIter, "leadVecIter");
    config->setConfig(seedsRatio, "seedsRatio");
    config->setConfig(radius, "radius");

    printf("config data load success!\n");
}

//初始化数据并读取配置文件
void PCR::init()
{
    cloudSour.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloudTar.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloudSourDs.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloudTarDs.reset(new pcl::PointCloud<pcl::PointXYZ>);
    transMatrix.Identity();
    readConfig();
}

//归一化点云
void PCNormalized(pointCloudPtr cloud)
{
    vector<long double> meanPoint(3, 0);
    int num = cloud->size();
    for (int i = 0; i < num; ++i)
    {
        meanPoint[0] += cloud->points[i].x;
        meanPoint[1] += cloud->points[i].y;
        meanPoint[2] += cloud->points[i].z;
    }
    meanPoint[0] /= num;
    meanPoint[1] /= num;
    meanPoint[2] /= num;
    for (int i = 0; i < num; ++i)
    {
        cloud->points[i].x -= meanPoint[0];
        cloud->points[i].y -= meanPoint[1];
        cloud->points[i].z -= meanPoint[2];
    }
}

//加载点云数据，目前支持pcd和ply格式
bool PCR::lodaData()
{
    string fileName = sourFilePath;
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
    // PCNormalized(cloudSour);
    // PCNormalized(cloudTar);
    sour = cloudSour;
    tar = cloudTar;
    printf("data load finish!\n");
    return true;
}

//降采样点云
void cloudFilter(pointCloudPtr cloud, pointCloudPtr cloudOut, float leafSize)
{
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(leafSize, leafSize, leafSize);
    filter.filter(*cloudOut);
}

void PCR::dsCloud()
{
    if (filterSize > 0)
    {
        cloudFilter(cloudSour, cloudSourDs, filterSize);
        cloudFilter(cloudTar, cloudTarDs, filterSize);
        sour = cloudSourDs;
        tar = cloudTarDs;
    }

    printf("DownSample point cloud success, the filter size is %f \n", filterSize);
}

//计算主特征向量
template <class T>
void calLeadEigenVec(const T &Mat, vector<float> &vec, const int iterNum)
{
    // power iteration 算法计算主特征向量
    int n = Mat.cols();
    vec.resize(n);
    // Eigen::MatrixXf V = Eigen::MatrixXf::Random(n, 1).array().abs();
    Eigen::MatrixXf V = Eigen::MatrixXf::Ones(n, 1);
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
    //使用特征分解
    // int maxVal = INT_MIN, maxInde = -1;
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
}

bool myCompareVec(pair<float, int> &a, pair<float, int> &b)
{
    return a.first > b.first;
}

//选择种子点
void PCR::pickSeeds()
{
    int sn = seedsRatio * scMat.cols();
    seeds.reserve(sn);
    vector<float> leadVec;
    calLeadEigenVec<Eigen::MatrixXf>(scMat, leadVec, leadVecIter);
    //从特征向量中选择最大的sn个点，且一定范围内不重复选点（非极大线性抑制）
    vector<pair<float, int>> sortLeadVec(leadVec.size());
    for (int i = 0; i < leadVec.size(); ++i)
    {
        sortLeadVec[i].first = leadVec[i];
        sortLeadVec[i].second = i;
    }
    sort(sortLeadVec.begin(), sortLeadVec.end(), myCompareVec); //直接比较大小

    //非极大线性抑制
    for (int i = 80; i < sortLeadVec.size(); ++i) //!!
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
        // seeds[i] = sortLeadVec[i].second;
        if (sortLeadVec[i].first != 0)
        {
            seeds.push_back(sortLeadVec[i].second);
        }
    }

    printf("pick seed pair success! The number of seeds is %d\n", (int)seeds.size());
}

double calDis(const pcl::PointXYZ &p, const pcl::PointXYZ &q)
{
    return sqrt(pow(p.x - q.x, 2) + pow(p.y - q.y, 2) + pow(p.z - q.z, 2));
}

//计算一阶空间兼容矩阵（SC mat）
void PCR::calScMat()
{
    int n = corres.size();
    scMat.resize(n, n);

    for (int i = 0; i < n; ++i)
    {
        scMat(i, i) = 0;
        for (int j = i + 1; j < n; ++j)
        {
            double dis1 = calDis(sour->points[corres[i].first], sour->points[corres[j].first]);
            double dis2 = calDis(tar->points[corres[i].second], tar->points[corres[j].second]);
            double dis = abs(dis1 - dis2);
            double score = 1 - pow(dis, 2) / pow(dthr, 2);
            scMat(i, j) = score < 0 ? 0 : score;
            scMat(j, i) = scMat(i, j);
        }
    }
    printf("calculate first order sc matrix success!\n");
}

//计算二阶空间兼容矩阵
void PCR::calSc2Mat()
{
    Eigen::MatrixXd &hardMat = seedHardMat;
    // int n = hardMat.cols();
    // sc2Mat.resize(n, n);
    // for (int i = 0; i < n; ++i)
    // {
    //     for (int j = i + 1; j < n; ++j)
    //     {
    //         sc2Mat(i, j) = 0;
    //         for (int k = 0; k < n; ++k)
    //         {
    //             sc2Mat(i, j) += hardMat(i, k) * hardMat(k, j);
    //             sc2Mat(j, i) = sc2Mat(i, j);
    //         }
    //     }
    // }

    sc2Mat = hardMat * hardMat;
    printf("calculate the second order SC matrix success!\n");
}

//计算一阶空间兼容矩阵的二值化矩阵，为计算二阶空间兼容矩阵做准备
void PCR::calScHardMat()
{
    Eigen::MatrixXd &hardMat = seedHardMat;
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
    printf("calculate seed hard sc matrix success!\n");
}

bool compareCor(pair<int, int> &a, pair<int, int> &b)
{
    return a.first > b.first;
}

void PCR::calWeight(vector<int> &Col)
{
    int n = Col.size();
    weights.resize(n, n);

    Eigen::MatrixXf k2SC(n, n);
    for (int i = 0; i < n; ++i)
    {
        for (int j = i; j < n; ++j)
        {
            k2SC(i, j) = scMat(Col[i], Col[j]);
            k2SC(j, i) = k2SC(i, j);
        }
    }
    Eigen::MatrixXf SC_ = k2SC * k2SC;
    Eigen::MatrixXf SC_2(n, n);
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            SC_2(i, j) = SC_(i, j) * k2SC(i, j);
        }
    }
    vector<float> vec;
    calLeadEigenVec(SC_2, vec, 40);
    for (int i = 0; i < vec.size(); ++i)
    {
        weights(i, i) = vec[i];
    }
}

//计算每个共识集合的变换矩阵
Eigen::Matrix4f PCR::calTrans(const int index)
{
    int n = sc2Mat.cols();
    int fn = k1; // first
    int sn = k2;
    vector<int> firstColIndex(fn);       //存储第一阶段共识集合index
    vector<int> secondColIndex(sn);      //存储第二阶段共识集合index
    vector<pair<int, int>> firstCorr(n); //存储待排序的值： 相似度，位置
    vector<pair<int, int>> secondCorr(fn);
    Eigen::MatrixXf localScMat(fn, fn);
    Eigen::MatrixXd localHardMat(fn, fn);
    Eigen::MatrixXd localSc2Mat(fn, fn);
    Eigen::MatrixXf leadVec;
    Eigen::MatrixXf cloudP(3, sn), cloudQ(3, sn);
    Eigen::MatrixXf cloudPtem(3, sn), cloudQtem(3, sn);
    Eigen::MatrixXf transTem(3, sn);
    Eigen::MatrixXf H(3, 3);
    Eigen::Matrix4f trans;  // trans*P=Q
    Eigen::Matrix3f transR; // transR*p+transT=Q
    Eigen::MatrixXf transT=Eigen::MatrixXf::Zero(3, 1);
    Eigen::MatrixXf meanP=Eigen::MatrixXf::Zero(3,1);
    Eigen::MatrixXf meanQ=Eigen::MatrixXf::Zero(3,1);

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
            localScMat(i, j) = scMat(firstColIndex[i], firstColIndex[j]);
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
    // calWeight(secondColIndex);
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
        meanP+=cloudP.col(i);
        meanQ+=cloudQ.col(i);
    }
    meanP/=sn;
    meanQ/=sn;
    cloudPtem = cloudP;
    cloudQtem = cloudQ;
    for (int i = 0; i < sn; ++i)
    {
        cloudP.col(i)-=meanP;
        cloudQ.col(i)-=meanQ;
    }
    H = cloudP * cloudQ.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXf> svdH(H, Eigen::DecompositionOptions::ComputeFullU | Eigen::DecompositionOptions::ComputeFullV);
    transR = svdH.matrixV() * svdH.matrixU().transpose();
    transTem = cloudQtem - transR * cloudPtem;

    for (int i = 0; i < sn; ++i)
    {
        transT(0, 0) += transTem(0, i);
        transT(1, 0) += transTem(1, i);
        transT(2, 0) += transTem(2, i);
    }
    transT /= sn;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            trans(i, j) = transR(i, j);
        }
        trans(3, i) = 0;
        trans(i, 3) = transT(i, 0);
    }
    trans(3, 3) = 1;

    return trans;
}

//计算种子corres经过矩阵变换后能变换正确的个数
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

//计算得到最终的矩阵变换
Eigen::Matrix4f PCR::calBestTrans()
{
    int n = seeds.size();
    int maxCount = -1, index = -1;
    Eigen::Matrix4f finalTrans;

    for (int i = 0; i < n; ++i)
    {
        Eigen::Matrix4f curTrans = calTrans(i);
        // cout<<curTrans<<endl;
        int count = calCount(sour, tar, seeds, curTrans, corres, pointThre);
        if (count >= maxCount)
        {
            maxCount = count;
            finalTrans = curTrans;
            index = i;
        }
    }
    printf("max count:%d index:%d pointThre:%f\n ", maxCount, index, pointThre);
    return finalTrans;
}

void ConfigRead::readConfigFile(string fileName)
{
    fstream cfgFile;
    cfgFile.open(fileName.c_str()); //打开文件
    if (!cfgFile.is_open())
    {
        cout << "can not open cfg file!" << endl;
        // return false;
    }
    char tmp[100];
    while (!cfgFile.eof()) //循环读取每一行
    {
        cfgFile.getline(tmp, 100); //每行读取前1000个字符，1000个应该足够了
        string line(tmp);
        size_t pos = line.find('='); //找到每行的“=”号位置，之前是key之后是value
        if (pos == string::npos)
            continue;

        string tmpKey = line.substr(0, pos);                               //取=号之前
        string tempValue = line.substr(pos + 1, line.find(' ') - pos - 1); //取=号之后
        confMap.insert(pair<string, string>(tmpKey, tempValue));
    }
    // return true;
}


bool ConfigRead::setConfig(double &conf, string name)
{
    conf = confMap.find(name) != confMap.end() ? atof(confMap[name].c_str()) : 0;
}

bool ConfigRead::setConfig(float &conf, string name)
{
    conf = confMap.find(name) != confMap.end() ? atof(confMap[name].c_str()) : 0;
}

bool ConfigRead::setConfig(string &conf, string name)
{
    conf = confMap.find(name) != confMap.end() ? confMap[name] : "";
}

bool ConfigRead::setConfig(int &conf, string name)
{
    conf = confMap.find(name) != confMap.end() ? atoi(confMap[name].c_str()) : 0;
}

