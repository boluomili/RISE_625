#include "control/pub.h"

Pub::Pub(){
    
    pub_RISE = _nh.advertise<unitree_guide::RISE>("/RISE",20);
}

Pub::~Pub(){

}
// 发布估计负载质量

// 发布机身位置、速度、加速度、足端力之和等消息
void Pub::pub_RISE_data(Vec3 error_1, Vec3 error_2, Vec3 integral, Vec3 tau_t, Vec3 template_sgn){
    for (int i = 0; i < 3; i++)
    {
        RISE_data.error_1[i]   = error_1[i];     //实际位置
        RISE_data.error_2[i] = error_2[i];       //期望位置
        RISE_data.integral[i] = integral[i];       //位置误差
        RISE_data.tau_t[i]   = tau_t[i];         //实际速度  
        RISE_data.template_sgn[i] = template_sgn[i];       //期望速度

    }
    // RISE_data.sumfootforce = sumfootforce;
    // RISE_data.d_mp = d_mp;
    pub_RISE.publish(RISE_data);
    
}

void Pub::pub_RISE_c_data(Vec3 c_error_1, Vec3 c_error_2, Vec3 c_integral, Vec3 c_tau_t, Vec3 c_template_sgn){
    for (int i = 0; i < 3; i++)
    {
        RISE_data.c_error_1[i]   = c_error_1[i];     //实际位置
        RISE_data.c_error_2[i] = c_error_2[i];       //期望位置
        RISE_data.c_integral[i] = c_integral[i];       //位置误差
        RISE_data.c_tau_t[i]   = c_tau_t[i];         //实际速度  
        RISE_data.c_template_sgn[i] = c_template_sgn[i];       //期望速度

    }
    // RISE_data.sumfootforce = sumfootforce;
    // RISE_data.d_mp = d_mp;
    pub_RISE.publish(RISE_data);
    //发布_velBody, _vCmdGlobal, _lowState->getAccGlobal(), _ddPcd,

}

void Pub::pub_RISE_error(Vec3 _vCmdGlobal,Vec3 _velbody,Vec3 _wCmdGlobal,Vec3 _wGlobal){
        for (int i = 0; i < 3; i++)
    {
        RISE_data.vCmdGlobal[i]   = _vCmdGlobal[i];     //实际位置
        RISE_data.velbody[i] = _velbody[i];       //期望位置
        RISE_data.wCmdGlobal[i] = _wCmdGlobal[i];       //位置误差
        RISE_data.wGlobal[i]   = _wGlobal[i];         //实际速度  


    }
    pub_RISE.publish(RISE_data);

}

