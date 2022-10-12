#pragma once
struct  TrajectoryPoint
{
    float x;
    float y;
};

struct Trajectory
{
    int id;
    TrajectoryPoint points[100];
};

struct MultiTrajectory
{

    int timestamp;
    int latency;
    int num_obstacles;
    char frame_id[30];
    std::vector<Trajectory> trajectories; 
};