#ifndef ZSL_H      //这是一个条件预处理指令，它检查是否宏已经被定
#define ZSL_H

#include "FSM/FSMState.h"
#include "Gait/GaitGenerator.h"
#include "control/BalanceCtrl.h"
#include "unitree_guide/RISE.h"
#include "/home/zsl/l1_adp/src/unitree_guide/unitree_guide/include/control/pub.h"


class State_Zsl:public FSMState{
public:
    State_Zsl(CtrlComponents *ctrlComp);
    ~State_Zsl();
    void enter();
    void run();
    void exit();
    virtual FSMStateName checkChange();
    void setHighCmd(double vx, double vy, double wz);
    double sgn(double val);
    Eigen::VectorXd eulereMethod(const Eigen::MatrixXd &A,const Eigen::VectorXd &x0,double t, double dt);
    Eigen::VectorXd systemDynamics(const Eigen::VectorXd x,const Eigen::MatrixXd A);
    Eigen::VectorXd Proj(const Eigen::VectorXd &alpha_hat,const Eigen::VectorXd &y_alpha,Eigen::VectorXd &alpha);
    Eigen::VectorXd rungeKuttaStep(const Eigen::MatrixXd &A, const Eigen::VectorXd &x, double h);
    Eigen::VectorXd second_filter(Vec6 theta);
    Eigen::MatrixXd Solve_P(Eigen::MatrixXd A_m,Eigen::MatrixXd Q);
    Eigen::VectorXd Proj(const Eigen::VectorXd &alpha_hat,const Eigen::VectorXd &y_alpha,double theta_max,double E);




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

    Pub *_pub;
    int count =0;
    Vec3 miu_t;
    Vec3 c_miu_t;
    double mass_;
    Mat3 mass_mat;

    //***************ESO****************
    Vec3 DeltaV_hat;
    Vec3 DeltaV_hat_v1;
    Vec3 DeltaV_hat_v2;
    Vec3 DeltaV_hat_pre;
    Vec3 d_DeltaV_hat;
    Vec3 d_DeltaV_hat_pre;
    Vec3 Omega_p;
    Vec3 uv;
    Vec3 vi_hat;
    Vec3 vi_hat_pre;
    Vec3 d_vi_hat;
    Vec3 pi_hat;
    Vec3 d_pi_hat;
    Vec3 pi;
    Vec3 F;
    Mat3 Lambda_p;
    Mat3 K_v;
    Vec3 pos_error_integral;
    Vec3 rv;
    Vec3 d_rv;
    Vec3 vr;
    Vec3 vr_pre;
    Vec3 d_vr;
    Vec3 E_posError;
    Vec3 E_posError_pre;
    Vec3 d_E_posError;
    Vec3 E_velError;
    Vec3 E_ddPcd;
    Vec3 E_dWbd;
    Vec3 g;
    Vec3 _vCmdGlobal_pre;
    Vec3 _pcd_pre;
    Vec3 d_pcd;

    Vec6 e;
    Vec6 d_e;
    Vec12 yita;
    Vec12 yita_tilde;
    Vec12 yita_hat;
    Mat12 D;
    Eigen::Matrix<double,12,6> H;
    Vec6 u;
    Vec6 d_d_e;
    Vec6 b_d;
    Mat6 M;
    Vec6 G;
    Vec6 theta;
    Vec6 theta_hat;
    Vec6 d_theta_hat;
    Vec12 d_yita;
    Vec6 alpha_t;
    Vec6 alpha_t_past;
    Vec6 d_alpha_hat;
    Vec6 beita_t;
    Vec6 beita_t_past;
    Vec6 d_beita_hat;
    Vec6 u_1;
    Vec6 u_2;
    Mat6 K_P;
    Mat6 K_D;
    Mat6 Identity_6;
    Mat6 Ones_6;
    Eigen::Matrix<double,6,12> matrix_KP_KD;
    Eigen::Matrix<double,3,3> matrix_3_I;
    double C_s;
    double w_n;
    double kesi;
    Vec6 y_alpha;
    Vec6 y_beita;
    Eigen::Matrix<double,6,6> gama;
    Eigen::Matrix<double,12,12> P;
    Eigen::Matrix<double, 12,12> big_A;
    Vec12  big_A_x0;

    Eigen::VectorXd xx;

    Eigen::VectorXd solve_big_A;
    Eigen::Matrix<double,6,6> gama_2;
    Eigen::Matrix<double,12,12> A_m;
    Eigen::Matrix<double,12,12> Q;


};

#endif 
