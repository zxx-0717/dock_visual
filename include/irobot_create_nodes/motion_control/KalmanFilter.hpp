// 定义卡尔曼滤波
// 参考 https://blog.csdn.net/m0_56116736/article/details/123328989

// 原始方程伪代码
// P_now = P_last + Q_cov;
// K = P_now / (P_now + R_cov);
// x_last = x_last + K * (z - x_last);
// P_last = (1 - K) * P_now;

#ifndef KALMAN_FILTER__HPP
#define KALMAN_FILETER_HPP

typedef struct Kalman_Filter{
	float x_last;
	float P_now;
	float P_last;
	float K;
	float R_cov;
	float Q_cov;
    float x_out;
}KF_Struct; 

void KF_Struct_Init(KF_Struct* KFS)
{
	KFS->x_last	=0;
	KFS->P_now	=0;
	KFS->P_last	=0.02;
	KFS->K		=0;
	KFS->Q_cov	=0.005;//过程激励噪声协方差,参数可调
	KFS->R_cov	=0.5;//测量噪声协方差，与仪器测量的性质有关，参数可调
	KFS->x_out	=0;
}

/*
* @brief    卡尔曼滤波器
* @param    KFS:卡尔曼滤波器结构体指针
* @param    z:测量仪器的输入量
* @return   当前时刻的最优估计值
*/
float KMFilter(KF_Struct* KFS,float z)
{
	KFS->P_now = KFS->P_last + KFS->Q_cov;
    KFS->K = KFS->P_now / (KFS->P_now + KFS->R_cov );
    KFS->x_out = KFS->x_out + KFS->K * (z - KFS->x_out);
    KFS->P_last = (1.f - KFS->K)* KFS->P_now;
    
    return KFS->x_out;
}

#endif // KALMAN_FILTER_HPP