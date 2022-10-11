#include <iostream>
#include <fstream>
#include <algorithm>

#include "../include/common.h"

HJPredictor::HJPredictor():device(torch::kCPU)
{
    model_file = "../model/model_20220518.pt";
    csv_file = "../csvfile/Data1_1.csv";
    LoadModels();
}

void HJPredictor::LoadModels()
{
    module = torch::jit::load(model_file);
    module.to(device);
}

void HJPredictor::LoadCSV()
{
    std::ifstream fp(csv_file); //定义声明一个ifstream对象，指定文件路径
    std::string line;
    std::string str;

    ObstacleState state;
    // getline(fp,line);       //舍弃第一行数据

    while(getline(fp,line)) //循环读取每行数据
    {
        std::istringstream readstr(line); //string数据流化
        //与csv文件的组织有关
        getline(readstr,str,','); 
        state.timestamp = atoi(str.c_str());
        getline(readstr,str,','); 
        state.latency = atoi(str.c_str());
        getline(readstr,str,','); 
        getline(readstr,str,','); 
        getline(readstr,str,','); 
        state.px = atof(str.c_str());
        getline(readstr,str,','); 
        state.py = atof(str.c_str());
        getline(readstr,str,','); 
        state.vx = atof(str.c_str());
        getline(readstr,str,','); 
        state.vy = atof(str.c_str());
        getline(readstr,str,','); 
        state.ax = atof(str.c_str());
        getline(readstr,str,',');
        state.ay = atof(str.c_str());

        states.push_back(state);
        // std::cout<<str<<std::endl;

    }


    fp.close();

}

void HJPredictor::Viewer2d(std::vector<std::vector<float>> &mat2d)
{
    int num_row = mat2d.size();
    int num_col = mat2d[0].size();
    torch::Tensor tensor2d=torch::zeros({num_row,num_col});

    for(int j=0;j<mat2d[0].size();j++)
    {
        for(int i=0;i<mat2d.size();i++)
        {
            tensor2d[i][j]=mat2d[i][j];
        }
    }

    std::cout<<tensor2d<<std::endl;

}

void HJPredictor::MinMaxScaler(std::vector<std::vector<float>> &mmfeatures,
        const std::vector<std::vector<float>> &features)
{
    int num_row = features.size();
    int num_col = features[0].size();

    float minValue;
    float maxValue;
    float diffValue;
    MinMaxInfo info;
    std::vector<float> temp;
    //计算最大最小值
    for(int j=0;j<num_col;j++)
    {
        temp.clear();
        for(int i=0;i<num_row;i++)
        {
            temp.push_back(features[i][j]);
        }
        minValue=*std::min_element(temp.begin(),temp.end());
        maxValue=*std::max_element(temp.begin(),temp.end());
        diffValue = maxValue-minValue;
        info.max = maxValue;
        info.min = minValue;
        info.diff = diffValue;
        minmaxinfo.push_back(info);

        //开始归一化
        for(int i=0;i<num_row;i++)
        {
            mmfeatures[i][j]=(features[i][j]-minValue)/diffValue;
        }

    }

}

void HJPredictor::MinMaxReScaler(std::vector<std::vector<float>> &result,
        const std::vector<std::vector<float>> &mmresult)
{
    int num_row=mmresult.size();
    int num_col=mmresult[0].size();

    float minValue;
    float diffValue;

    for(int j=0;j<num_col;j++)
    {
        minValue=minmaxinfo[j].min;
        diffValue=minmaxinfo[j].diff;

        for(int i=0;i<num_row;i++)
        {
            result[i][j]=mmresult[i][j]*diffValue+minValue;
        }
    }

}

void HJPredictor::SetFeatureValues(int k)
{
    if(k>=states.size() || k<50)
        exit(1);

    torch_inputs_.clear();

    torch::Tensor xinputs = torch::zeros({1,50,6});
    torch::Tensor xinput=torch::zeros({50,6});

    torch::Tensor yinputs = torch::zeros({1,100,6});

    std::vector<std::vector<float>> features;
    std::vector<std::vector<float>> mmfeatures;
    features.resize(50);
    for(auto& iter:features)
    {
        iter.resize(6);
    }
    mmfeatures.resize(50);
    for(auto& iter:mmfeatures)
    {
        iter.resize(6);
    }

    for(int i=0;i<50;i++)
    {   
        features[i][0]=states[k-50+i].px;
        features[i][1]=states[k-50+i].py;
        features[i][2]=states[k-50+i].vx;
        features[i][3]=states[k-50+i].vy;
        features[i][4]=states[k-50+i].ax;
        features[i][5]=states[k-50+i].ay;
    }

    Viewer2d(features);

    // 归一化
    MinMaxScaler(mmfeatures,features);
    for(int j=0;j<mmfeatures[0].size();j++)
    {
        for(int i=0;i<mmfeatures.size();i++)
        {
            xinput[i][j]=mmfeatures[i][j];
        }
    }

    xinputs[0] = xinput;

    // std::cout<<xinputs.sizes()<<std::endl;
    // std::cout<<yinputs.sizes()<<std::endl;

    // std::cout<<xinputs[0]<<std::endl;


    xinputs=xinputs.transpose(0,1);
    yinputs=yinputs.transpose(0,1);

    torch_inputs_.push_back(xinputs.to(device));
    torch_inputs_.push_back(yinputs.to(device));

}

bool HJPredictor::predict()
{   
    // torch::Tensor xinputs=torch::ones({1,50,6});
    // xinputs=xinputs.transpose(0,1);
    // torch::Tensor yinputs=torch::ones({1,100,6});
    // yinputs=yinputs.transpose(0,1);
    // torch_inputs_.push_back(xinputs.to(device));
    // torch_inputs_.push_back(yinputs.to(device));

    torch::Tensor output=module.forward(torch_inputs_).toTensor();
    output=output.transpose(0,1);

    //std::cout<<output<<std::endl;
    //std::cout<<"output is :"<<output[0].sizes()<<std::endl;

    torch::Tensor temp;
    temp = output[0];

    std::cout<<temp<<std::endl;
    std::vector<std::vector<float>> mmresult;
    std::vector<std::vector<float>> result;

    //初始化二维数组容器
    mmresult.resize(100);
    for(auto& iter:mmresult)
    {
        iter.resize(6);
    }
    result.resize(100);
    for(auto& iter:result)
    {
        iter.resize(6);
    }
    //从tensor转化为二维数组容器
    for(int j=0;j<6;j++)
    {
        for(int i=0;i<100;i++)
        {
            mmresult[i][j] = *temp[i][j].data_ptr<float>();
            
        }
    }

    MinMaxReScaler(result,mmresult);

    Viewer2d(result);

    // return true;
}

void HJPredictor::Run()
{

    int k = 50;
    SetFeatureValues(k);
    predict();

}

int main(int argc, char **argv)
{
    HJPredictor llc;
    llc.LoadCSV();
    llc.Run();

    return 0;
}

