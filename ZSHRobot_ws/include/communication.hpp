#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP

// #pragma pack(1)
#pragma pack(push, 1)

typedef struct
{
    float vehicleLineVelocity;
    float vehicleAngularVelocity; // 前轮转角
    float Accerlerator;           // 节气门 开的角度
    float rightArmAngleVelocity;
    float rearLeftArmAngleVelocity;
    float brakernum;
    int BCARArmAngle;
} ControllerMsg;

struct KQYKMS
{
    char KQYKMSmode;
};

struct KQDPYK
{
    char KQDPYKmode;
};

struct GBDPYK
{
    char GBDPYKmode;
};

struct DPYDKZ
{
    ControllerMsg TeleControlData;
};
// 以上为遥控模式

struct KQZZMS
{
    char KQZZMSmode;
};

struct GBZZMS
{
    char GBZZMSmode;
};

struct KQSSP
{
    char SSP_mode;
};

struct JSSSP
{
    char stopSSP_mode;
};

struct ZDSS
{
    char autoSearchMode;
};

struct SDSS
{
    char manualSearchMode;
    double pAimHeading;
};
// 以上为自主模式

struct tagPCHeader
{

    int msgID;
    // float UTC;
};

struct DLXWSZ
{
    short positiveTorque;
    short negativeTorque;
};

// 路点
struct FSLD
{
    double longitude[30];
    double latitude[30];
};

// 档位变更
struct DWBG
{
    int Gearsvalue;
};

// 摇臂角度设置
// struct YBJDSZ
// {
//     int Armangle;
// };

struct YBZTSZ
{
    int state;
};

struct BBJDSZ
{
    float legAngle;
};

struct YJDSZ
{
    float waistAngle;
};
struct ZBJDSZ
{
    float LeftJXBAngle;
};

struct YBJDSZ
{
    float RightJXBAngle;
};

struct YBKSorJS
{
    bool RightJXBStartOrFinish;
};

struct ZBKSorJS
{
    bool leftJXBStartOrFinish;
};
// 越障模式
struct KQYZMS
{
    char YZ_model;
};
// 构型模式
struct DPGXMS
{
    int DPGX_model;
};
// 舱门开与关
struct RBCMZT
{
    char RBCM_model;
};
// 采集路点
struct BUSCJLD
{
    char CJLD_flag; // 可以不用赋值
};
// 刹车变更
struct SCDWBG
{
    char SCDW_model;
};

struct RBZSTZ
{
    char setRBZSTZ;
};

struct RBZZRW
{
    char setRBZZRW;
};

struct BBFW
{
    char setBBFW;
};

struct JGJDSZ
{
    char setJGJDSZ;
};
struct JGBBFW
{
    char setJGBBFW;
};
struct KYC
{
    float setKYC;
};
struct RBJXZX // 任务继续
{
    char setRBJXZX;
};
// 轨迹跟踪
struct GJZZ
{
    char GJZZ_mode;
};
// 路径规划
struct LJGH
{
    int LJGH_mode;
};

struct LDGZ
{
    char LDGZ_mode;
};
// 有人端至机器人
typedef struct
{
    struct tagPCHeader msgHeader;
    union
    {
        // 档位变更
        // struct DWBG DWBG_;
        // struct DLXWSZ DLXWSZ_[6];
        // 遥控模式
        struct KQYKMS KQYKMS_;
        struct KQDPYK KQDPYK_;
        struct GBDPYK GBDPYK_;
        struct DPYDKZ DPYDKZ_;

        /// 自主导航
        struct KQZZMS KQZZMS_;
        struct GBZZMS GBZZMS_;
        struct FSLD FSLD_;

        struct KQSSP KQSSP_;
        struct JSSSP JSSSP_;
        struct SDSS SDSS_;
        struct ZDSS ZDSS_;

        /// 越障
        // struct YBJDSZ YBJDSZ_;
        struct YBZTSZ YBZTSZ_;

        // struct DPGXMS DPGXMS_;
        struct KQYZMS KQYZMS_;

        /// 舱门
        // struct RBCMZT RBCMZT_;

        // 采集路点
        // struct BUSCJLD BUSCJLD_;

        /// 摆臂
        struct BBJDSZ BBJDSZ_;
        struct YJDSZ YJDSZ_;

        /// 北理机械臂
        ///  左
        struct ZBJDSZ ZBJDSZ_[6];
        struct ZBKSorJS ZBKSorJS_;
        /// 左
        struct YBJDSZ YBJDSZ_[6];
        struct YBKSorJS YBKSorJS_;

        /// zy
        struct RBZSTZ RBZSTZ_;
        struct RBZZRW RBZZRW_;
        struct BBFW BBFW_;
        struct JGJDSZ JGJDSZ_;
        struct JGBBFW JGBBFW_;
        struct KYC KYC_;
        struct RBJXZX RBJXZX_; // 任务继续
        /// 跟踪和规划
        struct GJZZ GJZZ_;
        struct LJGH LJGH_;
        struct LDGZ LDGZ_;
    };
} MsgHuman2Rob;

typedef struct
{
    uint16_t crc16;
    MsgHuman2Rob MsgHuman2Rob_;
} MsgHuman2RobFrame;

// message que qt to vrep
typedef struct
{
    __syscall_slong_t mtype;
    ControllerMsg ControllerMsg_;
} QT_msg2Coppeliasim;

// 机器人至有人端
struct message_feedback
{
    unsigned short status[6];
    short current[6];
    short torque[6];
    int Position[6];
    int waistPosition;
    double pHeading;
    double px;
    double py;
    double pz;
    //  bool JXBFlag;     //true：右，false：左
    uint8_t JXBStatusFlag; // 2：右，1：左
    float JXBAngle[6];
    /// zy
    double Lvelocity;
    double Avelocity;
    double HeadAnlge;
    double RightDistance;
    double CorridorWidth;
    /// 2023.9zy 雷达
    double GPS_longitude;
    double GPS_latitude;
    double GPS_heading;
    /// qianxun
    double QX_position_x;
    double QX_position_y;
    double QX_position_z;
    double QX_RPY_r;
    double QX_RPY_p;
    double QX_RPY_y;

    double CH4_r;
    double CH4_c;
    double CH4_a;

    double H2S_a;
    double H2S_b;
    double H2S_c;

    double VOC_a;
    double VOC_b;
    double VOC_c;

    int Flag_ludian;
    int Flag_num;
};

typedef struct
{
    uint16_t crc16;
    message_feedback message_feedback_;
} ZSHRob2MsgHumanFrame;

class communication
{
private:
    /* data */
public:
    communication(/* args */);
    ~communication();
    char buff_joystick[1024];
    void communication_run(void);
    void Receive_PC_MSG(void);
    void Set2PC_MSG(void);
};
int countLines(const std::string &filename);
int Removefile(const char *filename);

#endif /* COMMUNICATION_HPP */