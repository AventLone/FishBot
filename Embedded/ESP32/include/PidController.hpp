#pragma once

class PidController
{
public:
    PidController() = default;
    PidController(float kp, float ki, float kd)
    {
        reset();                 // 初始化控制器
        updatePID(kp, ki, kd);   // 更新PID参数
    }

    float update(float control);   // 更新输出值
    void reset();                  // 重置PID控制器

    /*** Update PID coeffient ***/
    void updatePID(float kp, float ki, float kd)
    {
        reset();   // Reset the status of the controller
        mKp = kp;
        mKi = ki;
        mKd = kd;
    }

    /*** Update the target ***/
    void updateTarget(float target)
    {
        mTarget = target;
    }

    /*** Limit the output ***/
    void limitOutput(float out_min, float out_max)
    {
        mOUTmin = out_min;   // Limit the minimum of output
        mOUTmax = out_max;   // Limit the maximum of output
    }

public:
    float mTarget;       // 目标值
    float mOUTmin;       // 输出下限
    float mOUTmax;       // 输出上限
    float mKp;           // 比例系数
    float mKi;           // 积分系数
    float mKd;           // 微分系数
    float mLastOutput;   // 上一次输出值

    float mErrorSum;                // 误差累积和
    float mDerror;                  // 误差变化率
    float mPreError;                // 上上次误差
    float mLastError;               // 上一次误差
    float mMaxIntergral{2500.0f};   // 积分上限
};

float PidController::update(float control)
{
    /*** 计算误差及其变化率 ***/
    float error = mTarget - control;   // 计算误差
    mDerror = mLastError - error;      // 计算误差变化率
    mLastError = error;

    /*** 计算积分项并进行积分限制 ***/
    mErrorSum += error;
    if (mErrorSum > mMaxIntergral) mErrorSum = mMaxIntergral;
    if (mErrorSum < -1 * mMaxIntergral) mErrorSum = -1 * mMaxIntergral;

    /*** 计算控制输出值 ***/
    float output = mKp * error + mKi * mErrorSum + mKd * mDerror;

    /*** 控制输出限幅 ***/
    if (output > mOUTmax) output = mOUTmax;
    if (output < mOUTmin) output = mOUTmin;

    /*** 保存上一次的控制输出值 ***/
    mLastOutput = output;

    return output;
}

void PidController::reset()
{
    // 重置控制器状态
    mLastOutput = 0.0f;   // 上一次的控制输出值
    mTarget = 0.0f;       // 控制目标值
    mOUTmin = 0.0f;       // 控制输出最小值
    mOUTmax = 0.0f;       // 控制输出最大值
    mKp = 0.0f;           // 比例项系数
    mKi = 0.0f;           // 积分项系数
    mKd = 0.0f;           // 微分项系数
    mErrorSum = 0.0f;     // 误差累计值
    mDerror = 0.0f;       // 误差变化率
    mLastError = 0.0f;    // 上一次的误差值
}