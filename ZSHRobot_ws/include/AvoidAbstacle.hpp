#ifndef AVOIDABSTACLE_HPP
#define AVOIDABSTACLE_HPP
#pragma pack(1)

struct AbstacleINFO
{
    bool AbstacleFlag;
};

typedef struct
{
    __syscall_slong_t mtype;
    AbstacleINFO AbstacleINFO_;
} AbstacleMSG2Main;

class AvoidAbstacle
{
private:
    /* data */
public:
    AvoidAbstacle(/* args */);
    ~AvoidAbstacle();
    static AbstacleINFO AbstacleINFO_;
    void RecvAbstacleINFO();
};


#endif /* DRIVER */