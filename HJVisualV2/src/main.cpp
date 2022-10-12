#include "mainwindow.h"
#include <QApplication>
#include <qdesktopwidget.h>

#include <time.h>


void MainWindow::Viewer2d(std::vector<std::vector<float>> &mat2d)
{
    int num_row = mat2d.size();
    int num_col = mat2d[0].size();
    torch::Tensor tensor2d=torch::zeros({num_row,num_col});

    for(int j=0;j<num_col;j++)
    {
        for(int i=0;i<num_row;i++)
        {
            tensor2d[i][j]=mat2d[i][j];
        }
    }
    std::cout<<tensor2d<<std::endl;
}

void MainWindow::LoadModels()
{
    module = torch::jit::load(model_file);
    module.to(device);
}

void MainWindow::MinMaxReScaler(std::vector<std::vector<float>> &result,
        const std::vector<std::vector<float>> &mmresult,int k)
{
    int num_row=mmresult.size();
    int num_col=mmresult[0].size();

    float minValue;
    float diffValue;

    mminfo = hisMMInfo[k];

    for(int j=0;j<num_col;j++)
    {
        minValue=mminfo[j].min;
        diffValue=mminfo[j].diff;

        for(int i=0;i<num_row;i++)
        {
            result[i][j]=mmresult[i][j]*diffValue+minValue;
        }
    }


}

void MainWindow::MinMaxScaler(std::vector<std::vector<float>> &mmfeatures,
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
    mminfo.clear();
    // mminfo.resize(6);
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
        mminfo.push_back(info);

        //开始归一化
        for(int i=0;i<num_row;i++)
        {
            mmfeatures[i][j]=(features[i][j]-minValue)/diffValue;
        }
    }

    hisMMInfo.push_back(mminfo);

}

void MainWindow::SetHisStates(OBJ_DataBus *msg)
{
    ObstacleState state;
    int num_objects = hisObjsState.size();

    cout<<msg->num_obstacles<<endl;
    if(num_objects!=0)
    {
        for(int i=0;i<msg->num_obstacles;i++)
        {
            int id = int(msg->obstacles[i].id);
            float dx = msg->obstacles[i].nearest_point_rel.x;
            float dy = msg->obstacles[i].nearest_point_rel.y;
            float yaw = veh.yaw;

            state.px=dx*cos(yaw)-dy*sin(yaw)+veh.px;
            state.py=dx*sin(yaw)+dy*cos(yaw)+veh.py;
            state.vx = msg->obstacles[i].velocity_abs.x;
            state.vy = msg->obstacles[i].velocity_abs.y;
            state.ax = msg->obstacles[i].acceleration_abs.x;
            state.ay = msg->obstacles[i].acceleration_abs.y;

            state.latency=msg->latency;
            state.timestamp=msg->timestamp_low;
            if(state.vx<3 || dy>5 || dy<-5) continue;  //限制纵向距离
            if(id==0) continue;

            int k=0;
            for(;k<num_objects;k++)
            {
                if(hisObjsState[k].id==id)
                {
                    int m = hisObjsState[k].m;

                    if(m<49)
                    {
                        hisObjsState[k].m=m+1;
                        hisObjsState[k].states.push_back(state);
                    }
                    else
                    {
                        hisObjsState[k].states.erase(hisObjsState[k].states.begin());
                        hisObjsState[k].states.push_back(state);
                        hisObjsState[k].m=49;
                        hisObjsState[k].flag=true;
                    }
                    break;
                }
            }
            if(k==num_objects)
            {
                struct HisState hisState;
                hisState.id = id;
                hisState.flag=false;
                hisState.m=0;
                hisState.states.clear();
                hisState.states.push_back(state);                
                hisObjsState.push_back(hisState);
            }
        }
    }
    else
    {
        for(int i=0;i<msg->num_obstacles;i++)
        {
            int id = int(msg->obstacles[i].id);
            float dx = msg->obstacles[i].nearest_point_rel.x;
            float dy = msg->obstacles[i].nearest_point_rel.y;
            float yaw = veh.yaw;

            state.px=dx*cos(yaw)-dy*sin(yaw)+veh.px;
            state.py=dx*sin(yaw)+dy*cos(yaw)+veh.py;
            state.vx = msg->obstacles[i].velocity_abs.x;
            state.vy = msg->obstacles[i].velocity_abs.y;
            state.ax = msg->obstacles[i].acceleration_abs.x;
            state.ay = msg->obstacles[i].acceleration_abs.y;
            state.latency=msg->latency;
            state.timestamp=msg->timestamp_low;

            if(state.vx<5) continue;
            if(id==0) continue;

            struct HisState hisState;
            hisState.id = id;
            hisState.flag=false;
            hisState.m=0;
            hisState.states.clear();
            hisState.states.push_back(state);
            hisObjsState.push_back(hisState);
        }
    }

    // 删除过时轨迹
    for(auto iter=hisObjsState.begin();iter<hisObjsState.end();iter++)
    {
        HisState hisState = *iter;
        int m = hisState.m;
        if(hisState.states.size()!=0)
        {
            int dt = msg->timestamp_low - hisState.states[m].timestamp;
            if(dt>500)
            {
                hisObjsState.erase(iter);
            }
        }
    }
}

void MainWindow::SetFeatureValues()
{
    torch_inputs_.clear();
    int num = hisObjsPredicted.size();

    torch::Tensor xinput=torch::zeros({50,6});
    torch::Tensor xinputs=torch::zeros({num,50,6});
    torch::Tensor yinputs=torch::zeros({num,100,6});

    hisMMInfo.clear();
    for(int k=0;k<num;k++)
    {
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
        HisState hisState = hisObjsState[k];
        // start.x0 =hisObjsState[k].states[0].px;
        // start.y0 =hisObjsState[k].states[0].py;
        float yaw = veh.yaw;

        for(int j=0;j<50;j++)
        {   
            
            float dx = hisState.states[j].px-veh.px;
            float dy = hisState.states[j].py-veh.py;
            features[j][0]=dx*cos(yaw)+dy*sin(yaw);
            features[j][1]=-dx*sin(yaw)+dy*cos(yaw);
            // features[j][0]=hisState.states[j].px-start.x0;
            // features[j][1]=hisState.states[j].py-start.y0;
            features[j][2]=hisState.states[j].vx;
            features[j][3]=hisState.states[j].vy;
            features[j][4]=hisState.states[j].ax;
            features[j][5]=hisState.states[j].ay;
            // features[j][6]=hisObjsState[k].states[j].timestamp;
            // features[j][7]=hisObjsState[k].id;
        }

        // Viewer2d(features);
        MinMaxScaler(mmfeatures,features);

        for(unsigned int j=0;j<mmfeatures[0].size();j++)
        {
            for(unsigned int i=0;i<mmfeatures.size();i++)
            {
                xinput[i][j]=mmfeatures[i][j];
            }
        }
        xinputs[k] = xinput;
    }
    xinputs=xinputs.transpose(0,1);
    yinputs=yinputs.transpose(0,1);

    torch_inputs_.push_back(xinputs.to(device));
    torch_inputs_.push_back(yinputs.to(device));
}

bool MainWindow::Predict()
{
    int num = hisObjsPredicted.size();
    // torch::Tensor xinputs = torch::ones({1,50,6});
    // xinputs=xinputs.transpose(0,1);
    // torch::Tensor yinputs = torch::ones({1,100,6});
    // yinputs=yinputs.transpose(0,1);
    // torch_inputs_.push_back(xinputs.to(device));
    // torch_inputs_.push_back(yinputs.to(device));
 
    torch::Tensor output=module.forward(torch_inputs_).toTensor();
    output=output.transpose(0,1);
    
    torch::Tensor temp;
    for(int k=0;k<num;k++)
    {
        temp = output[k];
        // std::cout<<temp<<std::endl;
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
        //反归一化
        MinMaxReScaler(result,mmresult,k);
        // Viewer2d(result);

        struct Trajectory traj;

        traj.id = hisObjsState[k].id;

        // PointStart start;
        // // start.x0 =hisObjsState[i].states[0].px;
        // // start.y0 =hisObjsState[i].states[0].py;
        // // float yaw = map.yaw;
        // // float dx = result[i][0];
        // // float dy = result[i][1];

        for(int i=0;i<100;i++)
        {
            TrajectoryPoint p;
            // float dx = map.px-start.x0;
            // float dy = map.py-start.y0;

            // p.x=result[i][0]+dx*cos(yaw)+dy*sin(yaw);
            // p.y=result[i][1]-dx*sin(yaw)+dy*cos(yaw);
            p.x=result[i][0];
            p.y=result[i][1];
            traj.points[i] = p;

        } 
        mult.trajectories.push_back(traj);
    }

    return true;
}

void MainWindow::PredictAll()
{
    int num_objects=hisObjsState.size();

    std::cout<<"num of container objs is "<<num_objects<<std::endl;

    int n=0;
    hisObjsPredicted.clear();
    for(int i=0;i<num_objects;i++)
    {
        if(hisObjsState[i].flag)
        {
            // std::cout<<"id is "<<hisObjsState[i].id<<std::endl;
            
            // start=clock();
            hisObjsPredicted.push_back(hisObjsState[i]);
            n++;
            if(n>12) break;
        }
    }
    clock_t start,end;
    start=clock();
    SetFeatureValues();
    Predict();
    end = clock();
    double dtime = (double)(end-start)/CLOCKS_PER_SEC;
    cout<<"predict time is:"<<dtime<<"s"<<endl;

    mult.num_obstacles = mult.trajectories.size();

}


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.setWindowTitle("HJDrive Prediction Module");
    w.resize(1440,1200);
    QDesktopWidget *desktop = QApplication::desktop();

    w.move((desktop->width() - w.width())/ 2, (desktop->height() - w.height()) /2);

    w.show();

    QApplication::exec();
    return a.exec();
}
