#pragma once

#include "motion_config.h"

struct EncoderCounts
{
    uint32_t left = 0;
    uint32_t right = 0;
};

struct WheelPwm
{
    int left = 0;
    int right = 0;
};

int64_t countDelta(uint32_t current, uint32_t previous);
int direction(float value);

class Pid
{
public:
    explicit Pid(PidGains gains) : gains_(gains) {}
    void reset() { integral_ = 0; }
    // errorRate is d(error)/dt, supplied by the caller to avoid derivative kick.
    float update(float error, float errorRate, float dtSec,
                 float lower, float upper, bool integrate = true);

private:
    PidGains gains_;
    float integral_ = 0;
};

class HeadingController
{
public:
    explicit HeadingController(const MotionConfig& config)
        : config_(config), pid_(config.headingPid) {}
    void reset();
    void setReference() { referenceDeg_ = angleDeg_; pid_.reset(); }
    void update(float gyroDegPerSec, float dtSec, bool turning);
    float angleDeg() const { return angleDeg_ - referenceDeg_; }
    float rateDegPerSec() const { return rateDegPerSec_; }
    float correction(float limitPwm, float dtSec);

private:
    const MotionConfig& config_;
    Pid pid_;
    float angleDeg_ = 0;
    float referenceDeg_ = 0;
    float rateDegPerSec_ = 0;
};

class DistanceController
{
public:
    explicit DistanceController(const MotionConfig& config) : config_(config) {}
    void start(float targetMm, EncoderCounts origin);
    float travelMm(EncoderCounts counts) const;
    float remainingMm(float travelMm) const;
    bool brakeRequested(float travelMm) const;
    bool atTarget(float travelMm) const;
    float demandPwm(float travelMm) const;

private:
    const MotionConfig& config_;
    EncoderCounts origin_{};
    float targetMm_ = 0;
};

// One instance per wheel. Only driven time without an expected edge counts as stall.
class WheelMonitor
{
public:
    enum class Result { Ok, Stalled, WrongDirection };
    void reset(uint32_t nowUs);
    Result update(uint32_t nowUs, int previousPwm, int64_t delta, const MotionConfig& config);

private:
    uint32_t lastProgressUs_ = 0;
    int64_t wrongCounts_ = 0;
};

class TurnController
{
public:
    explicit TurnController(const MotionConfig& config)
        : config_(config), pid_(config.turnPid) {}
    void start(float targetDeg);
    bool atTarget(float angleDeg) const;
    float demandPwm(float angleDeg, float rateDegPerSec, float dtSec);

private:
    const MotionConfig& config_;
    Pid pid_;
    float targetDeg_ = 0;
};

float slewPwm(float current, float target, float dtSec, const MotionConfig& config);
