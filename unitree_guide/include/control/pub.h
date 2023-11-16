#ifndef PUB_H
#define PUB_H

#include "ros/ros.h"
#include "std_msgs/String.h" 
#include <std_msgs/Float64.h>
#include "unitree_guide/RISE.h"
#include "common/mathTypes.h"

class Pub
{
private:
    ros::NodeHandle _nh;// NodeHandle
    //ros::Publisher pub_mp;// mp
    //ros::Publisher pub_movement;// Movement Data
    ros::Publisher pub_RISE;// Turning Data
    //std_msgs::Float64 data_mp;// mp msgs
    unitree_guide::RISE RISE_data;// movement msgs
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

    // 发布机身方向、角速度、干扰项d、辅助变量更新律dkexi、转矩输出U
    //void pub_data_turning(Vec3 q, Vec3 q_d, Vec3 w, Vec3 w_d, Vec3 err_q, Vec3 err_w, Vec3 disturbance, Vec3 dkexi, Vec3 U);
    
    
    // float64[3] error_1
    // float64[3] error_2
    // float64[3] integral
    // float64[3] tau_t
    // float64[3] template_sgn

};

#endif