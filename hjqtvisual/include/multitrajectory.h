struct  Point
{
    float x;
    float y;
};

struct Trajectory
{
    Point points[50];
};

typedef struct MultiTrajectory
{
int timestamp;
int latency;
int num_obstacles;
char frame_id[30];
Trajectory trajectories[5]; 
} MultiTrajectory;