#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "hjdata.h"

// #include <torch/torch.h>
// #include <torch/script.h>

#include "qcustomplot.h"
#include <QMainWindow>


namespace Ui {
class MainWindow;
}

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

struct PointStart
{
    float x0;
    float y0;
};

struct HisState
{
    int id;
    bool flag;
    int m;
    std::vector<ObstacleState> states;
};



class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void InitPlot();
    void VisualHdmap(nop::MapData *hdmap);
    void VisualLanes(const MAP_DataBus &camera);

    void InitHJData();
    InfoPkg GetInfoPkg(int r);

public slots:
    void Timer50msCallback();

private:
    Ui::MainWindow *ui;
    QCustomPlot *mPlot;
    QTimer *timer50ms;
private:
    HJData *hjdata;
    vector<InfoPkg> infopkgs;

public:

    void MinMaxReScaler(std::vector<std::vector<float>> &result,
        const std::vector<std::vector<float>> &mmresult);
    void MinMaxScaler(std::vector<std::vector<float>> &mmfeatures,
        const std::vector<std::vector<float>> &features);

    void SetHisStates(OBJ_DataBus *msg);
    bool PredictOne(int i);
    void VisualMarker(int i);
    void PredictAll();
    void Viewer2d(std::vector<std::vector<float>> &mat2d); //for debug
    void SetFeatureValues(int i);

private:
    std::string model_file;
    // torch::Device device;
    // std::vector<torch::jit::IValue> torch_inputs_;

    std::vector<struct HisState> hisObjsState;

    std::vector<MinMaxInfo> minmaxinfo;
    struct PointStart start;
};

#endif // MAINWINDOW_H

