#include <unordered_map>
#include <iostream>
#include <fstream>

using namespace std;

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

