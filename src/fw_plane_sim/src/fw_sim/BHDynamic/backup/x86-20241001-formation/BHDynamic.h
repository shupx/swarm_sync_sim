#ifndef __BHDYNAMICS_H__
#define __BHDYNAMICS_H__

// BHDynamics.h : 头文件

#ifdef __cplusplus
extern "C" { 
#endif
/* 这种方式确保当头文件被 C++ 编译器编译时，函数按照 C 的方式导出。
当头文件被 C 编译器编译时，extern "C" 不会起作用，因为 C 编译器不识别这个关键字。*/


typedef struct
{
    double posiNInit; // 北向初始位置，m
    double posiEInit; // 东向初始位置，m
    double posiDInit; // 地向初始位置，m
    double velInit;   // 初始速度，m/s
    double pitchInit; // 初始俯仰角，rad
    double yawInit;   // 初始偏航角，rad
    double rollInit;  // 初始滚转角，rad
} State_Init;         // 初始化状态结构体

typedef struct
{
    double posiNState;    // 北向位置输出，m
    double posiEState;    // 东向位置输出，m
    double posiDState;    // 地向位置输出，m
    double pitchState;    // 俯仰角输出，rad
    double yawState;      // 偏航角输出，rad
    double rollState;     // 滚转角输出，rad
    double velState;      // 和速度输出，m/s
    double velalphaState; // 速度倾角，输出暂为0
    double velbeteState;  // 速度偏角，输出暂为0
    double alphaState;    // 攻角，输出暂为0
    double beteState;     // 侧滑角，输出暂为0
} State_Output;           // 输出状态结构体

// State_Init baseState;
// State_Output GS_state;

void InitState(State_Init baseState);
State_Output OutLoopCtrl_1(double High_input, double Vel_input, double Roll_input);
State_Output OutLoopCtrl_2(double High_input, double Vel_input, double Yaw_input);

#ifdef __cplusplus
}
#endif

#endif // __BHDYNAMICS_H__