#pragma once

#include <iostream>
#include <memory>
#include <vector>

#include <torch/torch.h>
#include <torch/script.h>

struct ObstacleState    // map frame
{
    int timestamp;
    int latency;
    float px;
    float py;
    float vx;
    float vy;
    float ax;
    float ay;
};

struct MinMaxInfo
{
    float min;
    float max;
    float diff;
};

class HJPredictor
{
public:
    HJPredictor();
    ~HJPredictor() {};
    void LoadModels();
    void SetFeatureValues(int k);
    void LoadCSV();
    void Viewer2d(std::vector<std::vector<float>> &mat2d);


    void MinMaxScaler(std::vector<std::vector<float>> &mmfeatures,
        const std::vector<std::vector<float>> &features);

    void MinMaxReScaler(std::vector<std::vector<float>> &result,
        const std::vector<std::vector<float>> &mmresult);

    bool predict();
    void Run();
    
private:
    std::string model_file;
    std::string csv_file;

    std::vector<torch::jit::IValue> torch_inputs_;
    std::vector<torch::jit::IValue> torch_outputs_;
    
    torch::jit::script::Module module;
    torch::Device device;

private:
    std::vector<ObstacleState> states;
    
    //用于归一化和反归一化
    std::vector<MinMaxInfo> minmaxinfo;

};




