#include "FSM/State_Zsl.h"
#include <iomanip>

// 这个头文件通常用于控制输出的格式，包括控制浮点数的小数位数、字段宽度、对齐方式等等。
// 在C++中，可以使用 std::setw、std::setprecision、std::fixed、std::scientific 等函数和标志，
// 以及 std::left、std::right 等对齐标志来格式化输出。


State_Zsl::State_Zsl(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::ZSL, "zsl"), 
              _est(ctrlComp->estimator), _phase(ctrlComp->phase), 
              _contact(ctrlComp->contact), _robModel(ctrlComp->robotModel), 
              _balCtrl(ctrlComp->balCtrl){
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

State_Zsl::~State_Zsl(){
    delete _gait;
}


void State_Zsl::enter(){
    _pcd = _est->getPosition();
    _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0);
    _vCmdBody.setZero();
    _yawCmd = _lowState->getYaw();
    _Rd = rotz(_yawCmd);
    _wCmdGlobal.setZero();

    _ctrlComp->ioInter->zeroCmdPanel();
    _gait->restart();

    //****************RISE**************    //力主要调节y，z方向
    K_s=Vec3(0.5, 0.5, 0.5).asDiagonal();
    beita=Vec3(0.5, 0.5, 0.5).asDiagonal();
    alphe_1=Vec3(0.5, 0.5, 0.5).asDiagonal();
    alphe_2=Vec3(1.0, 1.0, 1.0).asDiagonal();

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
    c_K_s=Vec3(0, 0, 0.2).asDiagonal();
    c_beita=Vec3(0, 0, 0.2).asDiagonal();
    c_alphe_1=Vec3(0, 0, 0.2).asDiagonal();
    c_alphe_2=Vec3(0, 0, 0.5).asDiagonal();

    c_error_1.setZero();
    c_error_2.setZero();
    c_integral.setZero();
    c_tau_t.setZero();
    c_template_sgn.setZero();

    I_1=Vec3(1,1,1).asDiagonal();
}

void State_Zsl::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
}

FSMStateName State_Zsl::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else if(_lowState->userCmd == UserCommand::START){
        return FSMStateName::TROTTING;
    }
    else{
        return FSMStateName::ZSL;
    }
}

void State_Zsl::run(){
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

    _gait->setGait(_vCmdGlobal.segment(0,2), _wCmdGlobal(2), _gaitHeight);
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

    calcTau();
    calcQQd();

    if(checkStepOrNot()){
        _ctrlComp->setStartWave();
    }else{
        _ctrlComp->setAllStance();
    }

    _lowCmd->setTau(_tau);
    _lowCmd->setQ(vec34ToVec12(_qGoal));
    _lowCmd->setQd(vec34ToVec12(_qdGoal));

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _lowCmd->setSwingGain(i);
        }else{
            _lowCmd->setStableGain(i);
        }
    }

}

bool State_Zsl::checkStepOrNot(){
    if( (fabs(_vCmdBody(0)) > 0.03) ||
        (fabs(_vCmdBody(1)) > 0.03) ||
        (fabs(_posError(0)) > 0.08) ||
        (fabs(_posError(1)) > 0.08) ||
        (fabs(_velError(0)) > 0.05) ||
        (fabs(_velError(1)) > 0.05) ||
        (fabs(_dYawCmd) > 0.20) ){
        return true;
    }else{
        return false;
    }
}

void State_Zsl::setHighCmd(double vx, double vy, double wz){
    _vCmdBody(0) = vx;
    _vCmdBody(1) = vy;
    _vCmdBody(2) = 0; 
    _dYawCmd = wz;
}

void State_Zsl::getUserCmd(){
    /* Movement */
    _vCmdBody(0) =  invNormalize(_userValue.ly, _vxLim(0), _vxLim(1));
    _vCmdBody(1) = -invNormalize(_userValue.lx, _vyLim(0), _vyLim(1));
    _vCmdBody(2) = 0;

    /* Turning */
    _dYawCmd = -invNormalize(_userValue.rx, _wyawLim(0), _wyawLim(1));
    _dYawCmd = 0.9*_dYawCmdPast + (1-0.9) * _dYawCmd;
    _dYawCmdPast = _dYawCmd;
}

void State_Zsl::calcCmd(){
    /* Movement */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody;

    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0)-0.2, _velBody(0)+0.2));
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1)-0.2, _velBody(1)+0.2));

    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
    _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _ctrlComp->dt, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));

    _vCmdGlobal(2) = 0;

    /* Turning */
    _yawCmd = _yawCmd + _dYawCmd * _ctrlComp->dt;

    _Rd = rotz(_yawCmd);
    _wCmdGlobal(2) = _dYawCmd;
}

//*************RISE************
    double left_min=-1;
    double right_max=1;
    int State_Zsl::sgn(double val){
    if (val > 0.001) {
        return 1;
    } else if (val < -0.001) {
        return -1;
    } else {
        return 0;
    }
    }
void State_Zsl::calcTau(){
    _posError = _pcd - _posBody;
    _velError = _vCmdGlobal - _velBody;

    _ddPcd = _Kpp * _posError + _Kdp * _velError;
    _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowState->getGyroGlobal());

    _ddPcd(0) = saturation(_ddPcd(0), Vec2(-3, 3));
    _ddPcd(1) = saturation(_ddPcd(1), Vec2(-3, 3));
    _ddPcd(2) = saturation(_ddPcd(2), Vec2(-5, 5));

    _dWbd(0) = saturation(_dWbd(0), Vec2(-40, 40));
    _dWbd(1) = saturation(_dWbd(1), Vec2(-40, 40));
    _dWbd(2) = saturation(_dWbd(2), Vec2(-10, 10));

    Vec3 _rotError =  rotMatToExp(_Rd*_G2B_RotMat);
    Vec3 _wError = -_lowState->getGyroGlobal() + _wCmdGlobal;        //getGyroGlobal计算角速度
    Mat3 _Ibody = Vec3(0.0792, 0.2085, 0.2265).asDiagonal(); 

        for (int i = 0; i < 3; i++)
    {
        template_sgn(i)=sgn(error_2(i));
    }

    //力主要调节y，z方向
    error_1=_posError;
    error_2=_velError+alphe_1*error_1;
    static Vec3 error_2_0=error_2;
    integral=integral+(K_s+I_1)*alphe_2*error_2*_ctrlComp->dt+beita*template_sgn*_ctrlComp->dt;           //相关于ks，alphe_2,beita
    tau_t=_Ibody*_ddPcd+_Ibody*alphe_1*_velError+_Ibody*alphe_2*error_2+
    (K_s+I_1)*error_2-(K_s+I_1)*error_2_0+integral;      //+integral    

    for (int i = 0; i < 3; i++)
    {
        c_template_sgn(i)=sgn(c_error_2(i));
    }
    //力矩，主要调节x，y轴的旋转
    c_error_1=_rotError;
    c_error_2=_wError+c_alphe_1*c_error_1;
    static Vec3 c_error_2_0 =c_error_2;
    c_integral=c_integral+(c_K_s+I_1)*c_alphe_2*c_error_2*_ctrlComp->dt+c_beita*c_template_sgn*_ctrlComp->dt;
    c_tau_t=_Ibody*_dWbd+_Ibody*c_alphe_1*_wError+_Ibody*c_alphe_2*c_error_2+
            (c_K_s+I_1)*c_error_2-(c_K_s+I_1)*c_error_2_0+c_integral;         //+c_integral

    static int count_1=0;
    if (count_1%5==0)
    {
    std::cout<<"ccc角速度是："<<_lowState->getGyroGlobal().transpose()<<std::endl;
    std::cout<<"ccc误差1："<<c_error_1.transpose()<<std::endl;
    std::cout<<"ccc误差2："<<c_error_2.transpose()<<std::endl;
    std::cout<<"ccc误差2的sgn："<<c_template_sgn.transpose()<<std::endl;
    std::cout<<"ccc误差2_0："<<c_error_2_0.transpose()<<std::endl;
    std::cout<<"ccc积分项："<<c_integral.transpose()<<std::endl;
    std::cout<<"ccc合力："<<c_tau_t.transpose()<<std::endl;
    std::cout<<"**************************"<<std::endl;
    }

    if (count_1%5==0)
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

    std::cout<<"FFF速度是："<<_velBody<<std::endl;
    std::cout<<"FFF误差1："<<error_1.transpose()<<std::endl;
    std::cout<<"FFF误差2："<<error_2.transpose()<<std::endl;
    std::cout<<"FFF误差2_0："<<error_2_0.transpose()<<std::endl;
    std::cout<<"FFF积分项："<<integral.transpose()<<std::endl;
    std::cout<<"FFF合力："<<tau_t.transpose()<<std::endl;
     
    }
    count_1++;
    

    //_forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);
    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact, tau_t, c_tau_t);

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _forceFeetGlobal.col(i) = _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
        }
    }

    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
    _q = vec34ToVec12(_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);


    
    // if (count%5 == 0)
    // {
    //    _pub->pub_RISE_data(error_1, error_2, c_error_1, c_error_2, tau_t, c_tau_t);
    // }
    // count++;
    

}

void State_Zsl::calcQQd(){

    Vec34 _posFeet2B;
    _posFeet2B = _robModel->getFeet2BPositions(*_lowState,FrameType::BODY);
    
    for(int i(0); i<4; ++i){
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody); 
        // _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody - _B2G_RotMat * (skew(_lowState->getGyro()) * _posFeet2B.col(i)) );  //  c.f formula (6.12) 
    }
    
    _qGoal = vec12ToVec34(_robModel->getQ(_posFeet2BGoal, FrameType::BODY));
    _qdGoal = vec12ToVec34(_robModel->getQd(_posFeet2B, _velFeet2BGoal, FrameType::BODY));
}

