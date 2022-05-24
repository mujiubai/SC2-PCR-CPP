#include"ConfigRead.h"

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