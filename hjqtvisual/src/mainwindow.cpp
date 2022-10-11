#include "mainwindow.h"
#include "ui_mainwindow.h"



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    // this->resize();
    InitHJData();
    mPlot = new QCustomPlot(this);
    InitPlot();
    timer50ms = new QTimer(this);
    // timer50ms->setInterval(100);

    // connect(timer50ms,&QTimer::timeout,this,&MainWindow::Timer50msCallback);
    connect(timer50ms,SIGNAL(timeout()),this,SLOT(Timer50msCallback()));
    timer50ms->start(50);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::InitPlot()
{
    mPlot->resize(1440,1200);
    mPlot->addGraph();
    mPlot->axisRect()->setBackground(QBrush(Qt::black));//背景黑色
    mPlot->xAxis->grid()->setPen(QPen(QColor(180, 180, 180), 1, Qt::PenStyle::DashLine));//网格白色虚线
    mPlot->yAxis->grid()->setPen(QPen(QColor(180, 180, 180), 1, Qt::PenStyle::DashLine));//网格白色虚线
    mPlot->xAxis->grid()->setSubGridPen(QPen(QColor(50, 50, 50), 1, Qt::DotLine));//网格浅色点线
    mPlot->yAxis->grid()->setSubGridPen(QPen(QColor(50, 50, 50), 1, Qt::DotLine));//网格浅色点线
    mPlot->xAxis->grid()->setSubGridVisible(true);//显示x轴子网格线
    mPlot->yAxis->grid()->setSubGridVisible(true);//显示要轴子网格线
    mPlot->xAxis->grid()->setZeroLinePen(QPen(Qt::white));//x轴0线颜色白色
    mPlot->yAxis->grid()->setZeroLinePen(QPen(Qt::white));//y轴0线颜色白色

    mPlot->xAxis->setLabel("x");             // 设置x轴的标签
    mPlot->yAxis->setLabel("y");        
    mPlot->xAxis->setRange(-30, 30);         // 设置x轴的范围
    mPlot->yAxis->setRange(-20, 100);
}

void MainWindow::VisualHdmap(nop::MapData *hdmap)
{   

    float x0 = hdmap->host_info.position.x;
    float y0 = hdmap->host_info.position.y;
    float yaw = hdmap->host_info.orientation.y;
    //host_lane_info
    int num = hdmap->host_lane.lane_info.num_center_line_points;
    // QVector<double> x(num),y(num);
    QVector<double> x,y;
    for(int i=0;i<num;i++)
    {
        float dx = hdmap->host_lane.lane_info.center_line_points[i].position.x-x0;
        float dy = hdmap->host_lane.lane_info.center_line_points[i].position.y-y0;
        float px = dx*cos(yaw)+dy*sin(yaw);
        float py = -dx*sin(yaw)+dy*cos(yaw);

        if(px<-50)
            continue;
        if(px>100)
            break;

        if(x.size()>!0)
        {
            auto itx = x.end();
            auto ity = y.end();

            Vector2f lastp,newp;
            lastp.x = *(itx-1);
            lastp.y = *(ity-1);

            float df = px - lastp.x;
            float coff = (py-lastp.y)/df;
            while(df>1.0)
            {
                newp.x = lastp.x+0.5;
                newp.y = lastp.y+coff*0.5;

                x.push_back(newp.x);
                y.push_back(newp.y);
                //
                lastp.x = newp.x;
                lastp.y = newp.y;
                df = px-lastp.x;
            }
        }
        x.push_back(px);
        y.push_back(py);
    }
    QPen pen; 
    mPlot->addGraph();
    pen.setColor(Qt::yellow);
    mPlot->graph()->setPen(pen);
    mPlot->graph()->setLineStyle(QCPGraph::lsNone);
    mPlot->graph()->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc,5));
    mPlot->graph()->setData(y, x);          // 为曲线图添加数据

    // lane_info
    for(int j=0;j<4;j++)
    {
        nop::LaneInfo lane_info = hdmap->lanes[j];
        num = lane_info.num_center_line_points;
        if(num>0)
        {
            mPlot->addGraph();
            x.clear();
            y.clear();
            for(int i=0;i<num;i++)
            {
                float dx = lane_info.center_line_points[i].position.x-x0;
                float dy = lane_info.center_line_points[i].position.y-y0;
                float px = dx*cos(yaw)+dy*sin(yaw);
                float py = -dx*sin(yaw)+dy*cos(yaw);

                if(px<-50)
                    continue;
                if(px>150)
                    break;

                if(x.size()>!0)
                {
                    auto itx = x.end();
                    auto ity = y.end();

                    Vector2f lastp,newp;
                    lastp.x = *(itx-1);
                    lastp.y = *(ity-1);

                    float df = px - lastp.x;
                    float coff = (py-lastp.y)/df;
                    while(df>1.0)
                    {
                        newp.x = lastp.x+0.5;
                        newp.y = lastp.y+coff*0.5;

                        x.push_back(newp.x);
                        y.push_back(newp.y);
                        //
                        lastp.x = newp.x;
                        lastp.y = newp.y;
                        df = px-lastp.x;
                    }
                }
                x.push_back(px);
                y.push_back(py);
            }
            pen.setColor(Qt::green);
            mPlot->graph()->setPen(pen);
            mPlot->graph()->setLineStyle(QCPGraph::lsNone);
            mPlot->graph()->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc,5));
            mPlot->graph()->setData(y, x);          // 为曲线图添加数据

        }
    }

}

void MainWindow::VisualLanes(const MAP_DataBus &camera)
{
    float dx=0.5;
    //lane visualization
    for(int i=0;i<camera.num_lane_markers;i++)
    {
        MAP_LaneMarker laneMarker = camera.lane_markers[i];
        if(!laneMarker.is_valid)
            continue;

        QPen pen;
        mPlot->addGraph();

        float c0 =laneMarker.c0_position;
		float c1 = laneMarker.c1_heading;
		float c2 = laneMarker.c2_curvature;
		float c3 = laneMarker.c3_curvature_derivative;

        // uint8_t color = laneMarker.color;
		uint8_t type = laneMarker.type; 

        float end = laneMarker.end_rel_x;
        if(end<=0)	//judge if end point <= 0
		{
			std::cout<<"end point error"<<std::endl;
			exit(1);
			
		}
        int num = floor(end/dx);
		if(type==3 && num%2!=0)
		{
			num=num+1;
		}
        QVector<double> x(num),y(num);

        for(int j=0;j<num;j++)
		{
			x[j] = dx*j;
			y[j] = c0+c1*x[j]+c2*pow(x[j],2)+c3*pow(x[j],3);
		}

        if(type==3)
		{
            pen.setStyle(Qt::PenStyle::DashLine);
            pen.setWidth(7);
		}
		else
		{
            mPlot->graph()->setLineStyle(QCPGraph::lsLine);
            pen.setWidth(7);
		}
        pen.setColor(Qt::white);
        mPlot->graph()->setPen(pen);
        mPlot->graph()->setData(y, x);          // 为曲线图添加数据
    }
    // mPlot->replot();

}

void MainWindow::Timer50msCallback()
{
    static int value = 0;
    mPlot->clearGraphs();
    // cout<<"timer is working"<<endl;
    InfoPkg pkg = GetInfoPkg(value);
    if(pkg.hdmap_valid)
    {
        VisualHdmap(&pkg.hdmap);
    }
    if(pkg.lanes_valid)
    {
        MAP_DataBus camera = pkg.lanes;
        VisualLanes(camera);
    }
    if(pkg.objs_valid)
    {
        
    }

    mPlot->replot();
    value++;
}

InfoPkg MainWindow::GetInfoPkg(int r)
{
    return infopkgs[r];
}

void MainWindow::InitHJData()
{
    HJData llc;
    hjdata = &llc;
    
    hjdata->SetPkgs();
    infopkgs.clear();
    hjdata->GetPkgs(infopkgs);  
    cout<<"-------------------------"<<endl;  
    cout<<"pkg size is "<<infopkgs.size()<<endl;

    //验证时间间隔
    // for(auto it=infopkgs.begin();it!=infopkgs.end()-1;it++)
    // {
    //     int dt = (it+1)->timestamp - it->timestamp;

    //     if(dt>70)
    //     {
    //         i++;
    //         cout<<"error occurs at : "<< i <<endl;
    //         // exit(1);
    //     }
    // }
}

void MainWindow::MinMaxReScaler(std::vector<std::vector<float>> &result,
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