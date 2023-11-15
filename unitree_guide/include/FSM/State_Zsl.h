#ifndef ZSL_H      //这是一个条件预处理指令，它检查是否宏已经被定
#define ZSL_H

#include "FSM/FSMState.h"
#include "Gait/GaitGenerator.h"
#include "control/BalanceCtrl.h"
//#include "unitree_guide/RISE.h"
// #include "/home/zsl/unitree_guide_11.13/src/unitree_guide/unitree_guide/include/control/pub.h"


class State_Zsl:public FSMState{
public:
    State_Zsl(CtrlComponents *ctrlComp);
    ~State_Zsl();
    void enter();
    void run();
    void exit();
    virtual FSMStateName checkChange();
    void setHighCmd(double vx, double vy, double wz);
    int sgn(double val);

private:
    void calcTau();
    void calcQQd();     //目标关节角度和关节角速度
    void calcCmd();
    virtual void getUserCmd();
    void calcBalanceKp();
    bool checkStepOrNot();

    GaitGenerator *_gait;
    Estimator *_est;
    QuadrupedRobot *_robModel;
    BalanceCtrl *_balCtrl;

    // Rob State
    Vec3  _posBody, _velBody;
    double _yaw, _dYaw;
    Vec34 _posFeetGlobal, _velFeetGlobal;
    Vec34 _posFeet2BGlobal;
    RotMat _B2G_RotMat, _G2B_RotMat;
    Vec12 _q;

    // Robot command
    Vec3 _pcd;
    Vec3 _vCmdGlobal, _vCmdBody;
    double _yawCmd, _dYawCmd;
    double _dYawCmdPast;
    Vec3 _wCmdGlobal;
    Vec34 _posFeetGlobalGoal, _velFeetGlobalGoal;
    Vec34 _posFeet2BGoal, _velFeet2BGoal;
    RotMat _Rd;
    Vec3 _ddPcd, _dWbd;
    Vec34 _forceFeetGlobal, _forceFeetBody;
    Vec34 _qGoal, _qdGoal;
    Vec12 _tau;

    // Control Parameters
    double _gaitHeight;
    Vec3 _posError, _velError;
    Mat3 _Kpp, _Kdp, _Kdw;
    double _kpw;
    Mat3 _KpSwing, _KdSwing;
    Vec2 _vxLim, _vyLim, _wyawLim;
    Vec4 *_phase;
    VecInt4 *_contact;

    // Calculate average value
    AvgCov *_avg_posError = new AvgCov(3, "_posError", true, 1000, 1000, 1);
    AvgCov *_avg_angError = new AvgCov(3, "_angError", true, 1000, 1000, 1000);

    //*********************RISE**************
    Vec3 error_1;
    Vec3 error_2;
    Vec3 r;
    Mat3 alphe_1;
    Mat3 alphe_2;
    Vec3 integral;
    Mat3 K_s;
    Mat3 beita;
    Vec3 template_sgn;
    Vec3 tau_t;
    
    Vec3 c_error_1;
    Vec3 c_error_2;
    Vec3 c_r;
    Mat3 c_alphe_1;
    Mat3 c_alphe_2;
    Vec3 c_integral;
    Mat3 c_K_s;
    Mat3 c_beita;
    Vec3 c_template_sgn;
    Vec3 c_tau_t;

    Mat3 I_1; //三维单位矩阵

    // Pub *_pub;
    // int count =0;

};

#endif 
