// #include "control/pub.h"

// Pub::Pub(){
    
//     pub_RISE = _nh.advertise<unitree_guide::RISE>("/RISE",20);
// }

// Pub::~Pub(){

// }
// // 发布估计负载质量

// // 发布机身位置、速度、加速度、足端力之和等消息
// void Pub::pub_RISE_data(Vec3 error_1, Vec3 error_2, Vec3 integral, Vec3 tau_t, Vec3 template_sgn){
//     for (int i = 0; i < 3; i++)
//     {
//         // RISE_data.p[i]   = p[i];     //实际位置
//         // RISE_data.p_d[i] = p_d[i];       //期望位置
//         // RISE_data.err_p[i] = err_p[i];       //位置误差
//         // RISE_data.v[i]   = v[i];         //实际速度  
//         // RISE_data.v_d[i] = v_d[i];       //期望速度
//         // RISE_data.err_v[i] = err_v[i];   //速度误差
//         //RISE_data.a[i]   = a[i];         //加速度
//         //RISE_data.a_d[i] = a_d[i];       //期望加速度
//     }
//     // RISE_data.sumfootforce = sumfootforce;
//     // RISE_data.d_mp = d_mp;
//     pub_RISE.publish(RISE_data);
// }
// // 发布机身方向、角速度、干扰项d、辅助变量更新律dkexi、转矩输出U
// // void Pub::pub_data_turning(Vec3 q, Vec3 q_d, Vec3 w, Vec3 w_d, Vec3 err_q, Vec3 err_w, Vec3 disturbance, Vec3 dkexi, Vec3 U){
// //     for (int i = 0; i < 3; i++)
// //     {
// //         data_turning.q[i]      = q[i];
// //         data_turning.q_d[i]    = q_d[i];
// //         data_turning.w[i]      = w[i];
// //         data_turning.w_d[i]    = w_d[i];
// //         data_turning.err_q[i]  = err_q[i];
// //         data_turning.err_w[i]  = err_w[i];
// //         data_turning.disturbance[i]   = disturbance[i];
// //         data_turning.dkexi[i]  = dkexi[i];
// //         data_turning.U[i] = U[i];
// //     }
// //     pub_turning.publish(data_turning);
// // }
