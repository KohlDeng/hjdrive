#include "mainwindow.h"
#include <QApplication>
#include <qdesktopwidget.h>
// #include "../common.h"


// void MainWindow::Viewer2d()
// {
//     int num_row = mat2d.size();
//     int num_col = mat2d[0].size();
//     torch::Tensor tensor2d=torch::zeros({num_row,num_col});

//     for(int j=0;j<mat2d[0].size();j++)
//     {
//         for(int i=0;i<mat2d.size();i++)
//         {
//             tensor2d[i][j]=mat2d[i][j];
//         }
//     }
//     std::cout<<tensor2d<<std::endl;
// }

// void MainWindow::SetHisStates(OBJ_DataBus *msg)
// {
//     ObstacleState state;
//     int num_objects = hisObjsState.size();

//     if(num_objects!=0)
//     {
//         for(int i=0;i<msg->objects.size();i++)
//         {
//             int id = int(msg->objects[i].id);
//             float dx = msg->objects[i].pos_rel.x;
//             float dy = msg->objects[i].pos_rel.y;
//             float yaw = map.yaw;

//             state.px=dx*cos(yaw)-dy*sin(yaw)+map.px;
//             state.py=dx*sin(yaw)+dy*cos(yaw)+map.py;
//             state.vx=msg->objects[i].vel_rel_abs.x;
//             state.vy=msg->objects[i].vel_rel_abs.y;
//             state.ax=msg->objects[i].acc_rel.x+vdyn.LongitudinalAcceleration;
//             state.ay=msg->objects[i].acc_rel.y+vdyn.LateralAcceletation;
//             state.latency=msg->latency;
//             state.timestamp=msg->timestamp;
//             if(state.vx<3)
//             {
//                 break;
//             }
//             int k=0;
//             for(;k<num_objects;k++)
//             {
//                 if(hisObjsState[k].id==id)
//                 {
//                     int m = hisObjsState[k].m;

//                     if(m<49)
//                     {
//                         hisObjsState[k].m=m+1;
//                         hisObjsState[k].states.push_back(state);
//                     }
//                     else
//                     {
//                         hisObjsState[k].states.erase(hisObjsState[k].states.begin());
//                         hisObjsState[k].states.push_back(state);
//                         hisObjsState[k].m=49;
//                         hisObjsState[k].flag=true;
//                     }
//                     break;
//                 }
//             }
//             if(k==num_objects)
//             {
//                 struct HisState hisState;
//                 hisState.id = id;
//                 hisState.flag=false;
//                 hisState.m=0;
//                 hisState.states.clear();
//                 hisState.states.push_back(state);                
//                 hisObjsState.push_back(hisState);
//             }
//         }
//     }
//     else
//     {
//         for(int i=0;i<msg->objects.size();i++)
//         {
//             int id = int(msg->objects[i].id);

//             state.px=msg->objects[i].pos_rel.x+map.px;
//             state.py=msg->objects[i].pos_rel.y+map.py;
//             state.vx=msg->objects[i].vel_rel_abs.x;
//             state.vy=msg->objects[i].vel_rel_abs.y;
//             state.ax=msg->objects[i].acc_rel.x+vdyn.LongitudinalAcceleration;
//             state.ay=msg->objects[i].acc_rel.y+vdyn.LateralAcceletation;
//             state.latency=msg->latency;
//             state.timestamp=msg->timestamp;
        
//             struct HisState hisState;
//             hisState.id = id;
//             hisState.flag=false;
//             hisState.m=0;
//             hisState.states.clear();
//             hisState.states.push_back(state);
//             hisObjsState.push_back(hisState);
//         }
//     }

//     // //删除过时轨迹
//     // for(auto iter=hisObjsState.begin();iter!=hisObjsState.end();iter++)
//     // {
//     //     int m = iter->m;
//     //     int size=iter->states.size();
//     //     if (size < m+1)
//     //     {
//     //         hisObjsState.erase(iter);
//     //     }
//     //     else
//     //     {
//     //         int dt = msg->timestamp - iter->states[m].timestamp;
//     //         if(dt>2500)
//     //         {
//     //             hisObjsState.erase(iter);
//     //         }
//     //     }
//     // }
// }

// void MainWindow::PredictAll()
// {
//     int num_objects=hisObjsState.size();

//     //std::cout<<"num is "<<num_objects<<std::endl;
//     int m=0;

//     for(int i=0;i<num_objects;i++)
//     {
//         if(hisObjsState[i].flag)
//         {
//             std::cout<<"id is "<<hisObjsState[i].id<<std::endl;
//             SetFeatureValues(i);
//             predict(i);
//             m++;
//         }
//     }

// }




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
