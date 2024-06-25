#ifndef PUB_H
#define PUB_H

#include "ros/ros.h"
#include "std_msgs/String.h" 
#include <std_msgs/Float64.h>
#include "unitree_guide/RISE.h"
#include "common/mathTypes.h"
#include "unitree_guide/ESO.h"
#include "unitree_guide/L1_adaptive.h"

class Pub
{
private:
    ros::NodeHandle _nh;// NodeHandle
    //ros::Publisher pub_mp;// mp
    //ros::Publisher pub_movement;// Movement Data
    ros::Publisher pub_RISE;// Turning Data
    ros::Publisher pub_ESO;
    ros::Publisher pub_L1_adaptive;
    //std_msgs::Float64 data_mp;// mp msgs
    unitree_guide::RISE RISE_data;// movement msgs
    unitree_guide::ESO ESO_data;
    unitree_guide::L1_adaptive L1_adaptive_data;

public:
    // 构造函数，初始化发布者（确定话题名称及队列长度）
    Pub();
    // 析构函数，空实现
    ~Pub();
    // 发布估计负载质量
    //void pub_data_mp(double _mp);
    // 发布机身位置、速度、加速度、足端力之和等消息
    void pub_RISE_data(Vec3 error_1, Vec3 error_2, Vec3 integral, Vec3 tau_t, Vec3 template_sgn);
    void pub_RISE_c_data(Vec3 c_error_1, Vec3 c_error_2, Vec3 c_integral, Vec3 c_tau_t, Vec3 c_template_sgn);
    //发布_velBody, _vCmdGlobal, _lowState->getAccGlobal(), _ddPcd,
    void pub_RISE_error(Vec3 _vCmdGlobal,Vec3 _velbody,Vec3 _wCmdGlobal,Vec3 _wGlobal);
    void pub_trot(Vec3 _vCmdGlobal,Vec3 _velbody,Vec3 _wCmdGlobal,Vec3 _wGlobal);
    void pub_height(double posBody_2,double pcd_2);
    void pub_error_1_2(Vec3 error_1,Vec3 error_2);
    void pub_force_tau(Vec3 tau_t,Vec3 c_tau_t);

    //************ESO发布****************
    void pub_ESO_data(Vec3 d_Deltav_hat,Vec3 Deltav_hat,Vec3 d_vi_hat,Vec3 vi_hat,Vec3 E_posError,
     Vec3 d_E_posError, Vec3 E_velError, Vec3 pos_error_integral, Vec3 rv,Vec3 d_vr,Vec3 F);


    void pub_L1_adaptive_data(Vec6 e,Vec6 d_e,Vec12 yita,Vec12 yita_hat,
     Vec12 yita_tilde,Vec6 y_alpha,Vec6 y_beita,Vec6 alpha_t,Vec6 d_beita_hat,
     Vec6 beita_t,Vec6 theta_hat,Vec6 u_1,Vec6 b_d,Vec3 vCmdGlobal,Vec3 velbody,double pcd_2,double posBody_2,
     Vec3 wCmdGlobal,Vec3 lowState_w);



    // 发布机身方向、角速度、干扰项d、辅助变量更新律dkexi、转矩输出U
    //void pub_data_turning(Vec3 q, Vec3 q_d, Vec3 w, Vec3 w_d, Vec3 err_q, Vec3 err_w, Vec3 disturbance, Vec3 dkexi, Vec3 U);
    
    
    // float64[3] error_1
    // float64[3] error_2
    // float64[3] integral
    // float64[3] tau_t
    // float64[3] template_sgn

};

#endif
