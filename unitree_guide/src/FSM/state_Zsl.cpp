#include "FSM/State_Zsl.h"
#include <iomanip>
#include <Eigen/Dense>
#include <fstream>
#include <Eigen/Core>

// 这个头文件通常用于控制输出的格式，包括控制浮点数的小数位数、字段宽度、对齐方式等等。
// 在C++中，可以使用 std::setw、std::setprecision、std::fixed、std::scientific 等函数和标志，
// 以及 std::left、std::right 等对齐标志来格式化输出。


State_Zsl::State_Zsl(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::ZSL, "zsl"),
      _est(ctrlComp->estimator), _phase(ctrlComp->phase),
      _contact(ctrlComp->contact), _robModel(ctrlComp->robotModel),
      _balCtrl(ctrlComp->balCtrl), _pub(ctrlComp->pub)
{
    _gait = new GaitGenerator(ctrlComp);

    _gaitHeight = 0.08;

#ifdef ROBOT_TYPE_Go1
    _Kpp = Vec3(70, 70, 70).asDiagonal();
    _Kdp = Vec3(10, 10, 10).asDiagonal();
    _kpw = 780;
    _Kdw = Vec3(70, 70, 70).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif

#ifdef ROBOT_TYPE_A1
    _Kpp = Vec3(20, 20, 100).asDiagonal();
    _Kdp = Vec3(20, 20, 20).asDiagonal();
    _kpw = 400;
    _Kdw = Vec3(50, 50, 50).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif

    _vxLim = _robModel->getRobVelLimitX();
    _vyLim = _robModel->getRobVelLimitY();
    _wyawLim = _robModel->getRobVelLimitYaw();
}

State_Zsl::~State_Zsl()
{
    delete _gait;
}




// 求解李雅普诺夫方程的函数
Eigen::MatrixXd solveLyapunovEquation(const Eigen::MatrixXd& A, const Eigen::MatrixXd& W) {
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(A.rows(), A.cols()); // 初始化X矩阵为零矩阵

    // 迭代求解，假设收敛条件为迭代次数
    int max_iter = 1000; // 设置最大迭代次数
    double epsilon = 1e-6; // 设置迭代精度
    double residual = epsilon + 1.0; // 初始化残差
    int iter = 0; // 迭代计数器

    while (residual > epsilon && iter < max_iter) {
        Eigen::MatrixXd X_new = (A * X + X * A.transpose() - W) / 2.0; // 李雅普诺夫方程迭代更新
        residual = (X_new - X).norm(); // 计算迭代步长
        X = X_new; // 更新X矩阵
        iter++; // 迭代次数加1
    }

    return X; // 返回求解结果
}



double euclideanNorm(const Eigen::VectorXd& vec) {
    double sum = 0.0;

    for (size_t i = 0; i < vec.size(); i++)
    {
        sum += vec[i] * vec[i];
    }
    
    // for (double element : vec) {
    //      // 累加每个元素的平方
    // }
    return sqrt(sum); // 返回平方和的平方根
}

Eigen::VectorXd State_Zsl::rungeKuttaStep(const Eigen::MatrixXd &A, const Eigen::VectorXd &x, double h)
{
    auto f = [&](const Eigen::VectorXd &x)
    { return A * x; };

    Eigen::VectorXd k1 = h * f(x);
    Eigen::VectorXd k2 = h * f(x + 0.5 * k1);
    Eigen::VectorXd k3 = h * f(x + 0.5 * k2);
    Eigen::VectorXd k4 = h * f(x + k3);

    return x + (k1 + 2 * k2 + 2 * k3 + k4) / 6;
}

Eigen::VectorXd State_Zsl::Proj(const Eigen::VectorXd &alpha_hat, const Eigen::VectorXd &y_alpha, Eigen::VectorXd &alpha)
{
    for (size_t i = 0; i < alpha_hat.size(); i++)
    {
        if (alpha[i] > y_alpha[i])
        {
            alpha[i] = y_alpha[i];
        }
        else if (alpha[i] < alpha_hat[i])
        {
            alpha[i] = alpha_hat[i];
        }
        else
        {
            alpha[i] = alpha[i];
        }
    }
    return alpha;
}

    Eigen::VectorXd State_Zsl::Proj(const Eigen::VectorXd &alpha_hat,const Eigen::VectorXd &y_alpha,double theta_max,double E)
    {
        double f=0;
        double df=0;
        Vec6 temp1;
        temp1.setZero();
        for (size_t i = 0; i < alpha_hat.size(); i++)
        {
            f=(alpha_hat[i]*alpha_hat[i]-theta_max*theta_max)/(E*theta_max*theta_max);
            df=2*alpha_hat[i]/(E*theta_max*theta_max);

            if (f<0)
            {
                temp1[i]=y_alpha[i];
            }
            else if(df*y_alpha[i]<=0)
            {
                temp1[i]=y_alpha[i];
            }
            else
            {
                temp1[i]=y_alpha[i]-y_alpha[i]*f;
            }
            
        }

        return temp1;
        
    }


Eigen::VectorXd State_Zsl::second_filter(Vec6 theta)
{
    double Wc = 400 ;
    double Tsw = 0.002;
    double ly = 0.7;
    double LPF2_b0 = 0.64;
    double LPF_a0 = 6.88;
    double LPF_a1 = -6.72;
    double LPF_a2 = 2.4;
    static double LPF2_Yout[3] = {0};
    static double LPF2_Xin[3] = {0};
    double y1 =0;

    for (size_t i = 0; i < theta.size(); i++)
    {
        LPF2_Xin[2] = theta[i];
        LPF2_Yout[2] = (0.64 * LPF2_Xin[2] + 1.28 * LPF2_Xin[1] + 0.64 * LPF2_Xin[0] - 6.72 * LPF2_Yout[1] - 2.4 * LPF2_Yout[0]) / 6.88;
        LPF2_Yout[0] = LPF2_Yout[1];
        LPF2_Yout[1] = LPF2_Yout[2];
        LPF2_Xin[0] = LPF2_Xin[1];
        LPF2_Xin[1] = LPF2_Xin[2];
        y1 = LPF2_Yout[2];

        theta[i]=y1;
    }
    return theta;

    // static Vec3 xn;
    // static Vec3 yn;
    // xn << 0, 0, 0;
    // yn << 0, 0, 0;
    // yn(2) = (0.64 * xn(2) + 1.28 * xn(1) + 0.64 * xn(0) + 6.72 * yn(1) + 1.6 * yn(0)) / 6.88;
    // xn(0) = xn(1);
    // xn(1) = xn(2);
}

#define Const_2pi       (6.283185)
#define Const_TS        (0.001) //100us
 
//二阶低通滤波器
double LPF2(double xin) {
   double f= 400;
   double wc = Const_2pi * f;
   double dampingRatio = 0.7;
 
   double lpf2_b0 = wc*wc*Const_TS*Const_TS;
   double lpf2_a0 = 4 + 4*dampingRatio*wc*Const_TS + lpf2_b0;
   double lpf2_a1 = -8 + 2*lpf2_b0;

   double lpf2_a2 = lpf2_b0 + 4 - 4*dampingRatio*wc*Const_TS;
 
   static double lpf2_yout[3] = {0};
   static double lpf2_xin[3] = {0};
 
   lpf2_xin[2] = xin;
   lpf2_yout[2] = (lpf2_b0 * lpf2_xin[2] + 2*lpf2_b0 *lpf2_xin[1] + lpf2_b0 *lpf2_xin[0] -lpf2_a1 *lpf2_yout[1] - lpf2_a2*lpf2_yout[0]) / lpf2_a0;
   lpf2_xin[0] = lpf2_xin[1];
   lpf2_xin[1] = lpf2_xin[2];
   lpf2_yout[0] = lpf2_yout[1];
   lpf2_yout[1] = lpf2_yout[2];
 
   return lpf2_yout[2];
}

Eigen::MatrixXd State_Zsl::Solve_P(Eigen::MatrixXd A_m,Eigen::MatrixXd Q)
{
    Eigen::MatrixXd X;
    X=Q+A_m*Q*A_m.transpose()+A_m*A_m*Q*A_m.transpose()*A_m.transpose()+A_m*A_m*A_m*Q*A_m.transpose()*A_m.transpose()*A_m.transpose();
    return X;
}

Eigen::MatrixXd solveLyapunov(const Eigen::MatrixXd& A, const Eigen::MatrixXd& W) {
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(A.rows(), A.cols()); // 初始化X矩阵为零矩阵

    // 迭代求解，假设收敛条件为迭代次数
    int max_iter = 100; // 设置最大迭代次数
    double epsilon = 1e-6; // 设置迭代精度
    double residual = epsilon + 1.0; // 初始化残差
    int iter = 0; // 迭代计数器
    while (residual > epsilon && iter < max_iter) {
       Eigen:: MatrixXd X_new = (A * X + X * A.transpose() - W) / 2.0; // 李雅普诺夫方程迭代更新
        residual = (X_new - X).norm(); // 计算迭代步长
        X = X_new; // 更新X矩阵
        iter++; // 迭代次数加1
    }


    return X; // 返回求解结果
}

void State_Zsl::enter()
{

    FILE *input1;
    input1 = fopen("AC_RISE.txt", "w");
    FILE *input3;
    input3 = fopen("AC_RISE_error.txt", "w");
    FILE *input5;
    input5=fopen("L1_adaptive_error.txt","w");
    




    _pcd = _est->getPosition();
    _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0);
    _vCmdBody.setZero();
    _yawCmd = _lowState->getYaw();
    _Rd = rotz(_yawCmd);
    _wCmdGlobal.setZero();

    _ctrlComp->ioInter->zeroCmdPanel();
    _gait->restart();

    //****************RISE**************    //力主要调节y，z方向
    // K_s=Vec3(10, 1, 10).asDiagonal();
    // beita=Vec3(50, 2, 50).asDiagonal();
    // alphe_1=Vec3(10, 2, 10).asDiagonal();
    // alphe_2=Vec3(5, 1, 5).asDiagonal();

    // 仿真优秀参数
    //  K_s=Vec3(10, 1, 3).asDiagonal();
    //  beita=Vec3(5, 2, 2).asDiagonal();
    //  alphe_1=Vec3(5, 2, 2).asDiagonal();
    //  alphe_2=Vec3(5, 1, 1).asDiagonal();
    K_s = Vec3(5, 1, 2).asDiagonal();
    beita = Vec3(2, 2, 2).asDiagonal();
    alphe_1 = Vec3(2, 2, 2).asDiagonal();
    alphe_2 = Vec3(1, 1, 1).asDiagonal();
    // K_s=Vec3(1, 1, 1).asDiagonal();
    // beita=Vec3(2, 2, 2).asDiagonal();
    // alphe_1=Vec3(1, 1, 1).asDiagonal();
    // alphe_2=Vec3(1, 1, 1).asDiagonal();

    // K_s=Vec3(5, 5, 5).asDiagonal();
    // beita=Vec3(10, 10, 10).asDiagonal();，。
    // alphe_1=Vec3(10, 10, 10).asDiagonal();
    // alphe_2=Vec3(5, 5, 5).asDiagonal();

    // K_s=5;
    // beita=10;
    // alphe_1=5;
    // alphe_2=2;

    error_1.setZero();
    error_2.setZero();
    integral.setZero();
    tau_t.setZero();
    template_sgn.setZero();

    //***************RISE力矩***********    //力矩主要调节x，y方向
    // c_K_s=Vec3(2, 2, 2).asDiagonal();
    // c_beita=Vec3(5, 5, 5).asDiagonal();
    // c_alphe_1=Vec3(5, 5, 5).asDiagonal();
    // c_alphe_2=Vec3(2, 2, 2).asDiagonal();

    // c_K_s=Vec3(10, 1, 0.5).asDiagonal();
    // c_beita=Vec3(25, 2, 1).asDiagonal();
    // c_alphe_1=Vec3(30, 2, 1).asDiagonal();
    // c_alphe_2=Vec3(15, 1, 1).asDiagonal();

    // 仿真优秀参数
    c_K_s = Vec3(1, 1, 1).asDiagonal();
    c_beita = Vec3(2, 2, 2).asDiagonal();
    c_alphe_1 = Vec3(2, 2, 2).asDiagonal();
    c_alphe_2 = Vec3(1, 1, 1).asDiagonal();
    // c_K_s=Vec3(1, 1, 1).asDiagonal();
    // c_beita=Vec3(2, 2, 2).asDiagonal();
    // c_alphe_1=Vec3(2, 2, 2).asDiagonal();
    // c_alphe_2=Vec3(1, 1, 1).asDiagonal();

    c_error_1.setZero();
    c_error_2.setZero();
    c_integral.setZero();
    c_tau_t.setZero();
    c_template_sgn.setZero();

    I_1 = Vec3(1, 1, 1).asDiagonal();

    //**************ESO****************
    d_DeltaV_hat.setZero();
    DeltaV_hat.setZero();
    DeltaV_hat_v1.setZero();
    DeltaV_hat_v2.setZero();
    DeltaV_hat_pre.setZero();
    d_DeltaV_hat_pre.setZero();
    Omega_p.setZero(); // 可调参数
    uv.setZero();
    vi_hat.setZero();
    vi_hat_pre.setZero();
    d_vi_hat.setZero();
    pi_hat.setZero();
    d_pi_hat.setZero();
    pi.setZero();
    pi_hat.setZero();
    F.setZero();
    Lambda_p.setIdentity(); // 可调参数
    K_v.setIdentity();      // 可调参数
    pos_error_integral.setZero();
    rv.setZero();
    d_rv.setZero();
    vr.setZero();
    vr_pre.setZero();
    d_vr.setZero();
    E_posError.setZero();
    E_posError_pre.setZero();
    d_E_posError.setZero();
    E_velError.setZero();
    E_ddPcd.setZero();
    E_dWbd.setZero();
    g << 0, 0, -9.81;
    _vCmdGlobal_pre.setZero();
    // _pcd.setZero();
    d_pcd.setZero();

    //**************L1 adaptive************
    e.setZero();
    d_e.setZero();
    yita.setZero();
    yita_tilde.setZero();
    yita_hat.setZero();
    D.setIdentity();
    H.setZero();
    u.setZero();
    d_d_e.setZero();
    b_d.setZero();
    M.setIdentity();
    G.setZero();
    theta.setZero();
    d_yita.setZero();
    alpha_t.setZero();
    alpha_t_past.setZero();
    beita_t.setZero();
    u_1.setZero();
    u_2.setZero();
    K_P.setZero();
    K_D.setZero();
    Identity_6.setIdentity();
    Ones_6.setOnes();
    matrix_KP_KD.setZero();
    matrix_3_I.setIdentity();
    C_s = 0;
    w_n = 0;
    kesi = 0;
    y_alpha.setZero();
    y_beita.setZero();
    theta_hat.setZero();
    d_theta_hat.setZero();
    d_alpha_hat.setZero();
    d_beita_hat.setZero();
    gama.diagonal() << 1000, 1000, 5000, 2000, 5000, 1000;
    P << -1.6167, 0, 0, 0, 0, 0, 0.0167, 0, 0, 0, 0, 0,
        0, -1.6167, 0, 0, 0, 0, 0, 0.0167, 0, 0, 0, 0,
        0, 0, -2.5500, 0, 0, 0, 0, 0, 0.0100, 0, 0, 0,
        0, 0, 0, -1.1025, 0, 0, 0, 0, 0, 0.0063, 0, 0,
        0, 0, 0, 0, -1.1025, 0, 0, 0, 0, 0, 0.0063, 0,
        0, 0, 0, 0, 0, -1.1025, 0, 0, 0, 0, 0, 0.0063,
        0.0167, 0, 0, 0, 0, 0, 0.0483, 0, 0, 0, 0, 0,
        0, 0.0167, 0, 0, 0, 0, 0, 0.0483, 0, 0, 0, 0,
        0, 0, 0.0100, 0, 0, 0, 0, 0, 0.0490, 0, 0, 0,
        0, 0, 0, 0.0063, 0, 0, 0, 0, 0, 0.0099, 0, 0,
        0, 0, 0, 0, 0.0063, 0, 0, 0, 0, 0, 0.0099, 0,
        0, 0, 0, 0, 0, 0.0063, 0, 0, 0, 0, 0, 0.0099;
    // P.setZero();
    // P.setZero();
    big_A.setZero();
    xx.setZero();
    big_A_x0.setZero();
    solve_big_A.setZero();
    gama_2.diagonal()<<1000,1000,5000,2000,5000,1000;
    A_m.setZero();
}

void State_Zsl::exit()
{
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();

   
    

    // 关闭文件
}

FSMStateName State_Zsl::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B)
    {
        return FSMStateName::PASSIVE;
    }
    else if (_lowState->userCmd == UserCommand::L2_A)
    {
        return FSMStateName::FIXEDSTAND;
    }
    else if (_lowState->userCmd == UserCommand::START)
    {
        return FSMStateName::TROTTING;
    }
    else
    {
        return FSMStateName::ZSL;
    }
}

void State_Zsl::run()
{
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();
    _yaw = _lowState->getYaw();
    _dYaw = _lowState->getDYaw();

    _userValue = _lowState->userValue;

    getUserCmd();
    calcCmd();

    _gait->setGait(_vCmdGlobal.segment(0, 2), _wCmdGlobal(2), _gaitHeight);
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

    calcTau();
    calcQQd();

    if (checkStepOrNot())
    {
        _ctrlComp->setStartWave();
    }
    else
    {
        _ctrlComp->setAllStance();
    }

    _lowCmd->setTau(_tau);
    _lowCmd->setQ(vec34ToVec12(_qGoal));
    _lowCmd->setQd(vec34ToVec12(_qdGoal));

    for (int i(0); i < 4; ++i)
    {
        if ((*_contact)(i) == 0)
        {
            _lowCmd->setSwingGain(i);
        }
        else
        {
            _lowCmd->setStableGain(i);
        }
    }
}

bool State_Zsl::checkStepOrNot()
{
    if ((fabs(_vCmdBody(0)) > 0.03) ||
        (fabs(_vCmdBody(1)) > 0.03) ||
        (fabs(_posError(0)) > 0.08) ||
        (fabs(_posError(1)) > 0.08) ||
        (fabs(_velError(0)) > 0.05) ||
        (fabs(_velError(1)) > 0.05) ||
        (fabs(_dYawCmd) > 0.20))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void State_Zsl::setHighCmd(double vx, double vy, double wz)
{
    _vCmdBody(0) = vx;
    _vCmdBody(1) = vy;
    _vCmdBody(2) = 0;
    _dYawCmd = wz;
}

void State_Zsl::getUserCmd()
{
    /* Movement */
    _vCmdBody(0) = invNormalize(_userValue.ly, _vxLim(0), _vxLim(1));
    _vCmdBody(1) = -invNormalize(_userValue.lx, _vyLim(0), _vyLim(1));
    _vCmdBody(2) = 0;


    //     if(count<500){
    //     _vCmdBody(0) =  count*0.001;
    // }else{
    //     _vCmdBody(0) =  0.5;
    // }

    /* Turning */
    _dYawCmd = -invNormalize(_userValue.rx, _wyawLim(0), _wyawLim(1));
    _dYawCmd = 0.9 * _dYawCmdPast + (1 - 0.9) * _dYawCmd;

    //     if(count<500){
    //     _dYawCmd =  count*0.001;
    // }else{
    //     _dYawCmd =  0.5;
    // }

    _dYawCmdPast = _dYawCmd;
}

void State_Zsl::calcCmd()
{
    /* Movement */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody;

    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0) - 0.35, _velBody(0) + 0.35)); // 饱和函数，应该是防止速度突变  0.2改0.35
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1) - 0.35, _velBody(1) + 0.35));

    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05)); // 根据速度计算机器人在全局坐标系下的位置 _pcd，并对位置进行饱和处理
    _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _ctrlComp->dt, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));

    _vCmdGlobal(2) = 0;

    /* Turning */
    _yawCmd = _yawCmd + _dYawCmd * _ctrlComp->dt;

    _Rd = rotz(_yawCmd);
    _wCmdGlobal(2) = _dYawCmd;
}

//*************RISE************
double left_min = -1;
double right_max = 1;
double State_Zsl::sgn(double val)
{
    if (val > 1)
    {
        return 1;
    }
    else if (val < -1)
    {
        return -1;
    }
    else
    {
        return val;
    }
}

// double State_Zsl::sgn(double val){
// if (val > 0.05) {
//     return 1;
// } else if (val < -0.05) {
//     return -1;
// } else {
//     return 0;
// }
// }
void State_Zsl::calcTau()
{
    _posError = _pcd - _posBody;
    _velError = _vCmdGlobal - _velBody;
    // Vec3 _rotError =  rotMatToExp(_Rd*_G2B_RotMat);
    _ddPcd = _Kpp * _posError + _Kdp * _velError;
    std::cout << "*****ddpcd：" << _ddPcd.transpose() << std::endl;
    std::cout << "*****_posError：" << _posError.transpose() << std::endl;
    std::cout << "*****_velError：" << _velError.transpose() << std::endl;

    _dWbd = _kpw * rotMatToExp(_Rd * _G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowState->getGyroGlobal());

    _ddPcd(0) = saturation(_ddPcd(0), Vec2(-3, 3));
    _ddPcd(1) = saturation(_ddPcd(1), Vec2(-3, 3));
    _ddPcd(2) = saturation(_ddPcd(2), Vec2(-5, 5));

    _dWbd(0) = saturation(_dWbd(0), Vec2(-40, 40));
    _dWbd(1) = saturation(_dWbd(1), Vec2(-40, 40));
    _dWbd(2) = saturation(_dWbd(2), Vec2(-10, 10));

    Vec3 _rotError = rotMatToExp(_Rd * _G2B_RotMat);
    Vec3 _wError = -_lowState->getGyroGlobal() + _wCmdGlobal; // getGyroGlobal计算角速度
    Mat3 _Ibody = Vec3(0.0792, 0.2085, 0.2265).asDiagonal();

    for (int i = 0; i < 3; i++)
    {
        template_sgn(i) = sgn(error_2(i));
    }

    // 力主要调节y，z方向
    mass_ = _robModel->getRobMass();
    mass_mat = Vec3(mass_, mass_, mass_).asDiagonal();
    error_1 = _posError;
    error_2 = _velError + alphe_1 * error_1;
    static Vec3 error_2_0 = error_2;
    integral = integral + (K_s + I_1) * alphe_2 * error_2 * _ctrlComp->dt + beita * template_sgn * _ctrlComp->dt; // 相关于ks，alphe_2,beita
    miu_t = integral + (K_s + I_1) * error_2 - (K_s + I_1) * error_2_0;
    // tau_t=_Ibody*_ddPcd+_Ibody*alphe_1*_velError+_Ibody*alphe_2*error_2+(K_s+I_1)*error_2-(K_s+I_1)*error_2_0+integral;      //+integral
    tau_t = mass_mat * alphe_1 * error_1 + mass_mat * alphe_2 * error_2 + (K_s + I_1) * error_2 - (K_s + I_1) * error_2_0 + integral; //+integral

    for (int i = 0; i < 3; i++)
    {
        c_template_sgn(i) = sgn(c_error_2(i));
    }
    // 力矩，主要调节x，y轴的旋转
    c_error_1 = _rotError;
    c_error_2 = _wError + c_alphe_1 * c_error_1;
    static Vec3 c_error_2_0 = c_error_2;
    c_integral = c_integral + (c_K_s + I_1) * c_alphe_2 * c_error_2 * _ctrlComp->dt + c_beita * c_template_sgn * _ctrlComp->dt;
    c_miu_t = (c_K_s + I_1) * c_error_2 - (c_K_s + I_1) * c_error_2_0 + c_integral;
    c_tau_t = _Ibody * _dWbd + _Ibody * c_alphe_1 * _wError + _Ibody * c_alphe_2 * c_error_2 +
              (c_K_s + I_1) * c_error_2 - (c_K_s + I_1) * c_error_2_0 + c_integral; //+c_integral

    static int count_1 = 0;
    if (count_1 % 5 == 0)
    {
        // std::cout<<"ccc角速度是："<<_lowState->getGyroGlobal().transpose()<<std::endl;
        // std::cout<<"ccc误差1："<<c_error_1.transpose()<<std::endl;
        // std::cout<<"ccc误差2："<<c_error_2.transpose()<<std::endl;
        // std::cout<<"ccc误差2的sgn："<<c_template_sgn.transpose()<<std::endl;
        // std::cout<<"ccc误差2_0："<<c_error_2_0.transpose()<<std::endl;
        // std::cout<<"ccc积分项："<<c_integral.transpose()<<std::endl;
        // std::cout<<"ccc合力："<<c_tau_t.transpose()<<std::endl;
        // std::cout<<"**************************"<<std::endl;
    }

    if (count_1 % 5 == 0)
    {
        // std::cout<<"alphe_1："<<alphe_1<<std::endl;
        // std::cout<<"alphe_2："<<alphe_2<<std::endl;
        // std::cout<<"K_s："<<K_s<<std::endl;
        // std::cout<<"beita："<<beita<<std::endl;
        // std::cout<<"c_alphe_1："<<c_alphe_1<<std::endl;
        // std::cout<<"c_alphe_2："<<c_alphe_2<<std::endl;
        // std::cout<<"c_K_s："<<c_K_s<<std::endl;
        // std::cout<<"c_beita："<<c_beita<<std::endl;

        // std::cout<<"FFF_U："<<_U.transpose()<<std::endl;

        // std::cout<<"FFF速度是："<<_velBody<<std::endl;
        // std::cout<<"FFF误差1："<<error_1.transpose()<<std::endl;
        // std::cout<<"FFF误差2："<<error_2.transpose()<<std::endl;
        // std::cout<<"FFF误差2_0："<<error_2_0.transpose()<<std::endl;
        // std::cout<<"FFF积分项："<<integral.transpose()<<std::endl;
        // std::cout<<"FFF合力："<<tau_t.transpose()<<std::endl;
    }
    count_1++;

    // ************ESO***************
    // 求解公式（15.3）位置误差，速度误差。
    E_posError = _posBody - _pcd;
    d_E_posError = (E_posError - E_posError_pre) / _ctrlComp->dt;
    E_posError_pre = E_posError;
    E_velError = _velBody - _vCmdGlobal;

    // ************构建位置积分误差、速度误差***************
    pos_error_integral = pos_error_integral + E_posError * _ctrlComp->dt;

    // ************ESO的参数设置**************
    Omega_p << 1.5, 1.5, 1.5;
    Lambda_p.diagonal() << 1, 1, 1;
    K_v.diagonal() << 1, 1, 1;

    // ************ESO期望加速度***************
    d_pcd = (_pcd - _pcd_pre) / _ctrlComp->dt;
    _pcd_pre = _pcd;
    E_ddPcd = (_vCmdGlobal - _vCmdGlobal_pre) / _ctrlComp->dt;
    // std::cout << "ESO的期望加速度是：" << E_ddPcd.transpose() << std::endl;
    _vCmdGlobal_pre = _vCmdGlobal;

    // ************构建F中的d_vr***************
    vr = d_pcd - 2 * Lambda_p * E_posError - Lambda_p * Lambda_p * pos_error_integral;
    d_vr = (vr - vr_pre) / (_ctrlComp->dt);
    vr_pre = vr;

    // ************ESO期望角加速度***************
    E_dWbd = _kpw * rotMatToExp(_Rd * _G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowState->getGyroGlobal());

    rv = d_E_posError + 2 * Lambda_p * E_posError + Lambda_p * Lambda_p * pos_error_integral;

    // d_vr=E_ddPcd-2*Lambda_p*d_E_posError-Lambda_p*Lambda_p*E_posError;
    //  ************ESO中Deltav_v求解***************
    //  ******************************************

    pi = _posBody;
    for (size_t i = 0; i < 3; i++)
    {
        d_DeltaV_hat(i) = std::pow(Omega_p(i), 3) * (pi(i) - pi_hat(i));
        // std::cout<<"ESO的d_DeltaV_hat(i)是："<<d_DeltaV_hat(i)<<std::endl;
    }

    DeltaV_hat = d_DeltaV_hat * _ctrlComp->dt + DeltaV_hat;
    DeltaV_hat_pre = DeltaV_hat;

    uv = -K_v * rv + d_vr - DeltaV_hat;
    for (size_t i = 0; i < 3; i++)
    {
        d_vi_hat(i) = uv(i) + DeltaV_hat(i) + 3.0 * std::pow(Omega_p(i), 2) * (pi(i) - pi_hat(i));
    }

    vi_hat = d_vi_hat * _ctrlComp->dt + vi_hat;
    vi_hat_pre = vi_hat;
    // std::cout<<"ESO的d_DeltaV_hat是："<<d_DeltaV_hat.transpose()<<std::endl;
    // std::cout<<"ESO的DeltaV_hat是："<<DeltaV_hat.transpose()<<std::endl;
    // std::cout<<"ESO的d_vi_hat是："<<d_vi_hat.transpose()<<std::endl;
    // std::cout<<"ESO的vi_hat是："<<vi_hat.transpose()<<std::endl;

    for (size_t i = 0; i < 3; i++)
    {
        d_pi_hat(i) = vi_hat(i) + 3 * Omega_p(i) * (pi(i) - pi_hat(i));
    }
    pi_hat = d_pi_hat * _ctrlComp->dt + pi_hat;

    // ************ESO中Deltav_v求解***************
    // ******************************************

    // F=mass_*uv+mass_*(K_v*rv)-mass_*(d_vr)+DeltaV_hat;
    F = mass_ * (-K_v * rv - DeltaV_hat);

    // std::cout<<"ESO的位置误差是："<<E_posError.transpose()<<std::endl;
    // std::cout<<"ESO的位置误差导数是："<<d_E_posError.transpose()<<std::endl;
    // std::cout<<"ESO的速度误差是："<<E_velError.transpose()<<std::endl;
    // std::cout<<"ESO的位置积分误差是："<<pos_error_integral.transpose()<<std::endl;
    // std::cout<<"ESO的rv是："<<rv.transpose()<<std::endl;
    // std::cout<<"ESO的d_vr是："<<d_vr.transpose()<<std::endl;
    // std::cout<<"ESO的F是："<<F.transpose()<<std::endl;
    // std::cout<<"RISE的F是："<<tau_t.transpose()<<std::endl;
    // std::cout<<"RISE的_ddPcd是："<<_ddPcd.transpose()<<std::endl;

    //_forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);

    //**************L1 adaptive************
    e << _posError, c_error_1;
    d_e << _velError, _wError;
    // P=Solve_P()
    yita << e, d_e;
    std::cout<<"**********************分割线**********************"<<std::endl;
    
    std::cout<<"e的值是："<<e.transpose()<<std::endl;
    std::cout<<"d_e的值是："<<d_e.transpose()<<std::endl;
    std::cout<<"yita的值是："<<yita.transpose()<<std::endl;
    std::cout<<"**********************分割线**********************"<<std::endl;

    D.setZero();
    D.block(0, 6, 6, 6) = Ones_6;
    // H.setZero();
    H.block(6, 0, 6, 6) = Ones_6;
    // K_P.diagonal() << 0.05, 0.05, 5, 8, 8, 8;
    // K_D.diagonal() << 1, 1, 1, 5, 5, 5;
    //  K_P.diagonal() << 30, 30, 120, 80, 80, 80;

     K_P.diagonal() << 30, 30, 300, 300, 80, 80;

    //  K_P.diagonal() << 30, 30, 50, 80, 80, 80;
    //  K_D.diagonal() << 1, 0.1, 1, 1, 1, 1;
     K_D.diagonal() << 10, 10, 10, 50, 50, 50;
    matrix_KP_KD.block(0, 0, 6, 6) = K_P;
    matrix_KP_KD.block(0, 6, 6, 6) = K_D;
    A_m.block(0,6,6,6)=Ones_6;
    A_m.block(6,0,6,6)=-K_P;
    A_m.block(6,6,6,6)=-K_D;
    Q.diagonal()<<1,1,1,1,1,1,1,1,1,1,1,1;

    // P=solveLyapunov(A_m,Q);
    // P=Solve_P(A_m,Q);
    // P=solveLyapunov(A_m,Q);



    u = matrix_KP_KD * yita;
    d_d_e = u; //***
    M.block(0, 0, 3, 3) = mass_ * matrix_3_I;
    // M.block(3, 3, 3, 3) = (_B2G_RotMat * _Ibody * _B2G_RotMat.transpose());
    M.block(3, 3, 3, 3) = _Ibody;
    std::cout<<"M值是："<<M<<std::endl;

    G << mass_ * g, 0, 0, 0;
    b_d = M * u + G;
    // d_alpha_hat=gama*
    // C_s=
    // theta=M.inverse()*[(M)];

    // d_yita=

    // static double tt = 0.06;
    // static double d_tt = 0.02;
    // xx = eulereMethod(big_A, yita, tt, d_tt);

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(big_A);
    // std::cout<<"big_A的特征值是：\n"<<eigensolver.eigenvalues();
    // // std::cout<<"big_A的特征向量是：\n"<<eigensolver.eigenvectors();

    // std::cout << "xx的值" << xx << std::endl;
    // std::cout << "big_A的值" << big_A << std::endl;

    big_A = D + H * matrix_KP_KD*-1;
    // big_A_x0<<0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1;
     big_A_x0<<yita;

    solve_big_A = rungeKuttaStep(big_A, big_A_x0, _ctrlComp->dt);

    std::cout << "solve_big_A的解解解是：\n"<< solve_big_A.transpose() << std::endl;
    yita_hat = solve_big_A;
    yita_tilde = yita_hat - yita;
    y_alpha = -H.transpose() * P * yita_tilde * euclideanNorm(yita);
    Eigen::MatrixXd first_1=-H.transpose() * P;
    double first_2=euclideanNorm(yita);
    std::cout<<"first_1的值是：\n"<<first_1<<std::endl;
    std::cout<<"first_2的值是：\n"<<first_2<<std::endl;

    std::cout<<"**********************分割线**********************"<<std::endl;

    // std::cout<<"yita_tilde的值是："<<yita_tilde.transpose()<<std::endl;

    std::cout<<"yita_tilde的值是："<<yita_tilde.transpose()<<std::endl;
    std::cout<<"yita_hat的值是："<<yita_hat.transpose()<<std::endl;

    std::cout<<"P的值是：\n"<<P<<std::endl;
    std::cout<<"yita_tilde的值是："<<yita_tilde.transpose()<<std::endl;
    std::cout<<"yita的值是："<<yita.transpose()<<std::endl;
    std::cout<<"**********************分割线**********************"<<std::endl;

    y_beita = -H.transpose() * P * yita_tilde;
    // d_alpha_hat=gama*Proj(alpha_t_past,y_alpha,alpha_t);
    // d_beita_hat=gama*Proj(beita_t_past,y_beita,beita_t);
    // d_alpha_hat = gama * y_alpha - 0.01 * alpha_t;
    // d_alpha_hat = Proj(alpha_t,gama*y_alpha,2,0.1);
    d_alpha_hat = Proj(alpha_t,gama*y_alpha,2,0.1);

    alpha_t = alpha_t + d_alpha_hat * _ctrlComp->dt;
    std::cout<<"d_alpha_hat的值是："<<d_alpha_hat.transpose()<<std::endl;
    std::cout<<"y_alpha的值是："<<y_alpha.transpose()<<std::endl;
    std::cout<<"alpha_t的值是："<<alpha_t.transpose()<<std::endl;
    std::cout<<"gama的值是："<<gama<<std::endl;
    std::cout<<"gama2的值是："<<gama_2<<std::endl;

    //gama修改
    // gama_2.diagonal()<<1000,1000,5000,2000,5000,1000;
    // d_beita_hat = gama_2 * y_beita - 0.01 * beita_t;
    d_beita_hat=Proj(beita_t,gama_2*y_beita,2,0.1);
    beita_t = beita_t + d_beita_hat * _ctrlComp->dt;                           
    std::cout<<"d_beita_hat的值是："<<d_beita_hat.transpose()<<std::endl;
    std::cout<<"gama的值是："<<gama.transpose()<<std::endl;
    std::cout<<"y_beita的值是："<<y_beita.transpose()<<std::endl;
    std::cout<<"beita_t的值是："<<beita_t.transpose()<<std::endl;

    std::cout<<"**********************分割线**********************"<<std::endl;

    
    theta_hat = alpha_t * euclideanNorm(yita)+beita_t;
    std::cout<<"alpha_t的值是："<<alpha_t.transpose()<<std::endl;
    std::cout<<"beita_t的值是："<<beita_t.transpose()<<std::endl;
    std::cout<<"yita的值是："<<yita.transpose()<<std::endl;

    u_1 = -matrix_KP_KD * yita_hat;
    // u_2 = -second_filter(theta_hat);
    for (size_t i = 0; i < theta_hat.size(); i++)
    {
            u_2[i] = -LPF2(theta_hat[i]);

    }
    
    b_d = M * (u_1 + u_2 + theta_hat) + G;
    // b_d = M * (u_1) + G;
    std::cout<<"u_1的值是："<<u_1.transpose()<<std::endl;
    std::cout<<"u_2的值是："<<u_2.transpose()<<std::endl;
    std::cout<<"theta_hat的值是："<<theta_hat.transpose()<<std::endl;
    std::cout<<"b_d的值是："<<b_d.transpose()<<std::endl;

    std::cout<<"**********************分割线**********************"<<std::endl;
    static Vec6 Mu_1;
    static Vec6 Mu_2;
    static Vec6 Mtheta_hat;
    Mu_1=M*u_1;
    Mu_2=M*u_2;
    Mtheta_hat=M*theta_hat;
    
    std::cout<<"Mu_1的值是："<<Mu_1.transpose()<<std::endl;
    std::cout<<"Mu_2的值是："<<Mu_2.transpose()<<std::endl;
    std::cout<<"Mtheta_hat的值是："<<Mtheta_hat.transpose()<<std::endl;
    std::cout<<"G的值是："<<G.transpose()<<std::endl;
    std::cout<<"M的值是：\n"<<M<<std::endl;
    std::cout<<"M_KP_KD的值是：\n"<<matrix_KP_KD<<std::endl;
    std::cout<<"yita_hat的值是：\n"<<yita_hat.transpose()<<std::endl;
    



    // _forceFeetGlobal = -_balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact, F, c_tau_t);
    _forceFeetGlobal = -_balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact, -b_d.head(3), -b_d.tail(3));

    for (int i(0); i < 4; ++i)
    {
        if ((*_contact)(i) == 0)
        {
            _forceFeetGlobal.col(i) = _KpSwing * (_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing * (_velFeetGlobalGoal.col(i) - _velFeetGlobal.col(i));
        }
    }

    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal; // 足端力转换到身体系.
    _q = vec34ToVec12(_lowState->getQ());            // 关节角度从3*4矩阵专12维向量
    _tau = _robModel->getTau(_q, _forceFeetBody);    // 足端力与关节角度反求力矩

    if (count % 5 == 0)
    {
        _pub->pub_ESO_data(d_DeltaV_hat, DeltaV_hat, d_vi_hat, vi_hat, E_posError,
                           d_E_posError, E_velError, pos_error_integral, rv, d_vr, F);
        std::cout << "ESO_data已发布*******" << std::endl;

    _pub->pub_L1_adaptive_data(e,d_e,yita,yita_hat,yita_tilde,
    y_alpha,y_beita,alpha_t,d_beita_hat,beita_t,theta_hat,u_1,b_d,
    _vCmdGlobal,_velBody,_pcd[2],_posBody[2],_wCmdGlobal,_lowState->getGyroGlobal());

        // _pub->pub_RISE_c_data(c_error_1, c_error_2, c_integral, c_tau_t, c_template_sgn);
        // _pub->pub_RISE_data(error_1, error_2, integral, tau_t, template_sgn);
        // _pub->pub_RISE_error( _vCmdGlobal, _velBody, _wCmdGlobal, _lowState->getGyroGlobal());
        // _pub->pub_height(_posBody(2),_pcd(2));
        // std::cout<<"RISE已发布"<<std::endl;

        // FILE *input1;
        // input1 = fopen("AC_RISE.txt","a");
        // fprintf(input1,"%f %f %f %f %f %f %f %f \n",_posBody(2),_pcd(2),_velBody(0),_vCmdGlobal(0), _velBody(1),_vCmdGlobal(1),_dYawCmd,_lowState->getGyroGlobal()(2));
        // fclose(input1);

        // FILE *input3;
        // input3 = fopen("AC_RISE_error.txt","a");
        // fprintf(input3,"%f %f %f %f %f %f %f %f %f %f %f %f \n",_posError(0),_posError(1),_posError(2),_rotError(0), _rotError(1),_rotError(2),tau_t(0),tau_t(1),tau_t(2),c_tau_t(0),c_tau_t(1),c_tau_t(2));
        // fclose(input3);

            // 创建一个ofstream对象，用于写文件

    // std::ofstream outFile("L1_adaptive.txt",std::ios::app);

    //      // 检查文件是否成功打开
    // if (!outFile) {
    //     std::cerr << "无法打开文件" << std::endl;
    //     // return 1; // 返回错误码
    // }

    // // 向文件写入数据
    // // outFile << _posBody(2) " " <<_pcd(2) " "<<_velBody(0)" "<<_vCmdGlobal(0)" "<<_dYawCmd " "<< _lowState->getGyroGlobal()(2)" "<<std::endl;
    // outFile <<_posBody(2)<<" "<<_pcd(2)<<" "<<_velBody(0)<<" "<<_vCmdGlobal(0)<<" "<<_dYawCmd<<" "<< _lowState->getGyroGlobal()(2)<<std::endl;
    // outFile.close();
        FILE *input5;
        input5 = fopen("L1_adaptive_error.txt","a");
        fprintf(input5,"%f %f %f %f %f %f %f %f %f %f %f %f \n",_posBody(2),_pcd(2),_velBody(0),_vCmdGlobal(0),_dYawCmd,_lowState->getGyroGlobal()(2),_posError(0),_posError(1),_posError(2),c_error_1(0),c_error_1(1),c_error_1(2));
        fclose(input5);
    

    }
    count++;




}

void State_Zsl::calcQQd()
{

    Vec34 _posFeet2B;
    _posFeet2B = _robModel->getFeet2BPositions(*_lowState, FrameType::BODY);

    for (int i(0); i < 4; ++i)
    {
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody);
        // _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody - _B2G_RotMat * (skew(_lowState->getGyro()) * _posFeet2B.col(i)) );  //  c.f formula (6.12)
    }

    _qGoal = vec12ToVec34(_robModel->getQ(_posFeet2BGoal, FrameType::BODY));
    _qdGoal = vec12ToVec34(_robModel->getQd(_posFeet2B, _velFeet2BGoal, FrameType::BODY));
}


