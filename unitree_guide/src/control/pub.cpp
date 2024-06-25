#include "control/pub.h"

Pub::Pub(){
    
    pub_RISE = _nh.advertise<unitree_guide::RISE>("/RISE",20);
    pub_ESO  = _nh.advertise<unitree_guide::ESO>("/ESO",50);
    pub_L1_adaptive= _nh.advertise<unitree_guide::L1_adaptive>("/L1_adaptive",50);
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

void Pub::pub_trot(Vec3 _vCmdGlobal,Vec3 _velbody,Vec3 _wCmdGlobal,Vec3 _wGlobal){
        for (int i = 0; i < 3; i++)
    {
        RISE_data.vCmdGlobal[i]   = _vCmdGlobal[i];     //实际位置
        RISE_data.velbody[i] = _velbody[i];       //期望位置
        RISE_data.wCmdGlobal[i] = _wCmdGlobal[i];       //位置误差
        RISE_data.wGlobal[i]   = _wGlobal[i];         //实际速度  


    }
    pub_RISE.publish(RISE_data);
    
}

    void Pub::pub_height(double posBody_2,double pcd_2){
        RISE_data.pcd_2=pcd_2;
        RISE_data.posBody_2=posBody_2;
    }


    void Pub::pub_error_1_2(Vec3 error_1,Vec3 error_2){
        for(int i=0; i<3 ;i++)
        {
            RISE_data.error_1[i]=error_1[i];
            RISE_data.error_2[i]=error_2[i];
        }
    }


    void Pub::pub_force_tau(Vec3 tau_t,Vec3 c_tau_t){
                for(int i=0; i<3 ;i++)
        {
            RISE_data.tau_t[i]=tau_t[i];
            RISE_data.c_tau_t[i]=c_tau_t[i];
        }
    }


    void Pub::pub_ESO_data(Vec3 d_Deltav_hat,Vec3 Deltav_hat,Vec3 d_vi_hat,Vec3 vi_hat,Vec3 E_posError,
     Vec3 d_E_posError, Vec3 E_velError, Vec3 pos_error_integral, Vec3 rv,Vec3 d_vr,Vec3 F){
                for (int i = 0; i < 3; i++)
    {

        ESO_data.d_Deltav_hat[i]=d_Deltav_hat[i];
        ESO_data.Deltav_hat[i]=Deltav_hat[i];
        ESO_data.d_vi_hat[i]=d_vi_hat[i];
        ESO_data.vi_hat[i]=vi_hat[i];
        ESO_data.E_posError[i]=E_posError[i];
        ESO_data.d_E_posError[i]=d_E_posError[i];
        ESO_data.E_velError[i]=E_velError[i];
        ESO_data.pos_error_integral[i]=pos_error_integral[i];
        ESO_data.rv[i]=rv[i];
        ESO_data.d_vr[i]=d_vr[i];
        ESO_data.F[i]=F[i];

    }
    pub_ESO.publish(ESO_data);
    pub_RISE.publish(ESO_data);
     }


     void Pub::pub_L1_adaptive_data(Vec6 e,Vec6 d_e,Vec12 yita,Vec12 yita_hat,
     Vec12 yita_tilde,Vec6 y_alpha,Vec6 y_beita,Vec6 alpha_t,Vec6 d_beita_hat,
     Vec6 beita_t,Vec6 theta_hat,Vec6 u_1,Vec6 b_d,Vec3 vCmdGlobal,Vec3 velbody,double pcd_2,double posBody_2,
     Vec3 wCmdGlobal,Vec3 lowState_w)
     {

        for (size_t i = 0; i < 6; i++)
        {
            L1_adaptive_data.e[i]=e[i];
            L1_adaptive_data.d_e[i]=d_e[i];
            L1_adaptive_data.y_alpha[i]=y_alpha[i];
            L1_adaptive_data.y_beita[i]=y_beita[i];
            L1_adaptive_data.alpha_t[i]=alpha_t[i];
            L1_adaptive_data.d_beita_hat[i]=d_beita_hat[i];
            L1_adaptive_data.beita_t[i]=beita_t[i];
            L1_adaptive_data.theta_hat[i]=theta_hat[i];
            L1_adaptive_data.u_1[i]=u_1[i];
            L1_adaptive_data.b_d[i]=b_d[i];
        }
        
        for (size_t i = 0; i < 12; i++)
        {
            L1_adaptive_data.yita[i]=yita[i];
            L1_adaptive_data.yita_hat[i]=yita_hat[i];
            L1_adaptive_data.yita_tilde[i]=yita_tilde[i];

        }

        for (size_t i = 0; i < 3; i++)
        {
            L1_adaptive_data.vCmdGlobal[i]=vCmdGlobal[i];
            L1_adaptive_data.velbody[i]=velbody[i];
            L1_adaptive_data.wCmdGlobal[i]=wCmdGlobal[i];
            L1_adaptive_data.lowState_w[i]=lowState_w[i];
        }

        L1_adaptive_data.pcd_2=pcd_2;
        L1_adaptive_data.posBody_2=posBody_2;
        
        
    pub_L1_adaptive.publish(L1_adaptive_data);
     }
