// DynamicTest.cpp : 定义控制台应用程序的入口点。

#include <stdio.h>
#include <unistd.h>
#include "BHDynamics.h"
#include <dlfcn.h>
// #define LIB_CALCULATE_PATH "./lib/libBHDynamic.so"

int main(int argc, char const *argv[])
{
    // typedef void (*CALC_FUNC)(State_Init baseState);

    // typedef State_Output (*CALC_FUNC1)(double High_input, double Vel_input, double Roll_input);

    // typedef State_Output (*CALC_FUNC2)(double High_input, double Vel_input, double Yaw_input);

    // void *handle;
    // char *error;
    // CALC_FUNC InitStateFun = NULL;
    // CALC_FUNC1 OutLoopCtrl_1Fun = NULL;
    // CALC_FUNC2 OutLoopCtrl_2Fun = NULL;

    // handle = dlopen(LIB_CALCULATE_PATH, RTLD_LAZY);
    // if (!handle)
    // {
    //     fprintf(stderr, "%s\n", dlerror());
    //     return -1;
    // }

    // dlerror();

    // InitStateFun = dlsym(handle, "InitState");

    // if ((error = dlerror()) != NULL)
    // {
    //     fprintf(stderr, "%s\n", error);
    //     return -1;
    // }

    // OutLoopCtrl_1Fun = (CALC_FUNC1)dlsym(handle, "OutLoopCtrl_1");
    // if ((error = dlerror()) != NULL)
    // {
    //     fprintf(stderr, "%s\n", error);
    //     return -1;
    // }
    // OutLoopCtrl_2Fun = (CALC_FUNC2)dlsym(handle, "OutLoopCtrl_2");
    // if ((error = dlerror()) != NULL)
    // {
    //     fprintf(stderr, "%s\n", error);
    //     return -1;
    // }
    int a, b, c;
    double RollEXP, HighEXP, VelEXP, yawExp;

// 状态初始化

a = 0;
NEXT:
    printf("%s\n", "请输入1初始化：");
    scanf("%d", &a);
    if (a == 1)
    {

        baseState.posiNInit = 0;    // 北向初始位置，m
        baseState.posiEInit = 0;    // 东向初始位置，m
        baseState.posiDInit = -200; // 地向初始位置，m
        baseState.velInit = 30;     // 初始速度，m/s
        baseState.pitchInit = 0;    // 初始俯仰角，rad
        baseState.yawInit = 0;      // 初始偏航角，rad
        baseState.rollInit = 0;     // 初始滚转角，rad

        InitState(baseState);

        printf("飞行状态初始化完毕！\n");
    }
    else
    {
        printf("输入有误，请重新输入!\n");
        goto NEXT;
    }
    // 飞行控制
    RollEXP = 20;  // deg
    HighEXP = 100; // m
    VelEXP = 30;   // m/s
    yawExp = 50;   // deg
    printf("%s%.2f%s%.2f%s%.2f\n", "滚转期望=", RollEXP, "高度期望=", HighEXP, "速度期望=", VelEXP);

TEMP:
    printf("%s\n", "请输入2进行飞行控制：");
    scanf("%d", &b);
    if (b == 2)
    {
        printf("飞行状态初始化完毕！\n");
        for (int i = 0; i < 5000; i++)
        {
            // GS_state = OutLoopCtrl_1Fun( HighEXP, VelEXP,RollEXP);    // 北理控制1
            // GS_state = OutLoopCtrl_2Fun(HighEXP, VelEXP, yawExp); // 北理控制2
            GS_state = OutLoopCtrl_1( HighEXP, VelEXP,RollEXP);
            printf("%s%.2f%s%.2f%s%.2f%s%.2f\n", "滚转反馈=", GS_state.rollState * 180 / 3.1415926, "高度反馈=", GS_state.posiDState, "速度反馈=", GS_state.velState, "偏航反馈=", GS_state.yawState * 180 / 3.14);
        }
    }
    else
    {
        printf("输入有误，请重新输入!\n");
        goto TEMP;
    }

    printf("%s\n", "按任何键退出：");
    scanf("%d", &c);
    // getchar();
    return 0;
}
