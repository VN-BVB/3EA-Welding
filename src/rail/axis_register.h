#ifndef AXIS_REGISTER_H
#define AXIS_REGISTER_H
#include <QCoreApplication>
#include <QDebug>
#include <QNetworkProxy>
#include <QObject>
#include <QProcess>
#include <QThread>
#include <QTimer>
#include <QVector>
#include <cstdint>
/* =========================
 *  轴枚举
 * ========================= */
enum class Axis : uint8_t { X = 0, Y = 1, Z = 2 };

constexpr int axisIndex(Axis axis) { return static_cast<int>(axis); }

/* =========================
 *  每轴地址偏移定义
 * ========================= */
constexpr int AXIS_B_OFFSET = 0x000B;  // B 区：每轴 11 bit
constexpr int AXIS_R_OFFSET = 0x0018;  // R 区：每轴 12 word
constexpr int AXIS_S_OFFSET = 0x0020;  // S 区：每轴 32 bit
constexpr int AXIS_D_OFFSET = 0x000A;  // D 区：每轴 5~6 word

/* =========================
 *  B 区（线圈，IN）
 * ========================= */
enum class RegB : int {
    ServoEnable = 0x3000,         // IN_使能 (B0)
    Stop = 0x3001,                // IN_停止 (B1)
    Reset = 0x3002,               // IN_复位 (B2)
    JogForward = 0x3003,          // IN_正向点动 (B3)
    JogReverse = 0x3004,          // IN_反向点动 (B4)
    HomeCommand = 0x3005,         // IN_回原命令 (B5)
    AbsPositionCommand = 0x3006,  // IN_绝对定位命令 (B6)
    RelPositionCommand = 0x3007,  // IN_相对定位命令 (B7)
    TorqueCommand = 0x3008,       // IN_力矩定位命令 (B8)
    ImmediateStop = 0x3009,       // IN_急停 (B9)
    SetOverRide = 0x300A          // IN_运动超调使能 (B10)
};

constexpr int addr(Axis axis, RegB reg) { return static_cast<int>(reg) + axisIndex(axis) * AXIS_B_OFFSET; }

/* =========================
 *  R 区（输入寄存器，IN 参数）
 * ========================= */
enum class RegR : int {
    JogSpeed = 0x3000,        // IN_点动速度 (R0)
    AbsPosition = 0x3002,     // IN_绝对定位位置 (R2)
    AbsSpeed = 0x3004,        // IN_绝对定位速度 (R4)
    RelPosition = 0x3006,     // IN_相对定位位置 (R6)
    RelSpeed = 0x3008,        // IN_相对定位速度 (R8)
    TorqueForce = 0x300A,     // IN_力矩定位力 (R10)
    TorqueSpeed = 0x300C,     // IN_力矩定位速度 (R12)
    Acceleration = 0x300E,    // IN_加速度 (R14)
    Deceleration = 0x3010,    // IN_减速度 (R16)
    CurveType = 0x3012,       // IN_曲线类型 (R18)
    ORSpeed = 0x3014,         // IN_超调速度 (R20)
    ORAcceleration = 0x3016,  // IN_超调加速度 (R22)
};

constexpr int addr(Axis axis, RegR reg) { return static_cast<int>(reg) + axisIndex(axis) * AXIS_R_OFFSET; }

/* =========================
 *  S 区（状态位，OUT）
 * ========================= */
enum class RegS : int {
    PosLimitSignal = 0xE000,       // OUT_正极限位信号 (S0)
    PosLimitAlarm = 0xE001,        // OUT_正限位报警 (S1)
    NegLimitSignal = 0xE002,       // OUT_负极限位信号 (S2)
    NegLimitAlarm = 0xE003,        // OUT_负限位报警 (S3)
    HomeSignal = 0xE004,           // OUT_原点信号 (S4)
    EnableComplete = 0xE005,       // OUT_使能完成 (S5)
    StopComplete = 0xE006,         // OUT_停止完成 (S6)
    ResetComplete = 0xE007,        // OUT_复位完成 (S7)
    AbsPosComplete = 0xE008,       // OUT_在当前绝对定位位置 (S8)
    DriverAlarm = 0xE009,          // OUT_驱动器报警 (S9)
    AxisFault = 0xE00A,            // OUT_轴故障中 (S10)
    FBError = 0xE00B,              // OUT_FB执行错误 (S11)
    RelativeMoving = 0xE00C,       // OUT_相对定位执行中 (S12)
    RelativePosHolding = 0xE00D,   // OUT_相对定位完成保持 (S13)
    RelativePosComplete = 0xE00E,  // OUT_相对定位完成 (S14)
    AbsoluteMoving = 0xE00F,       // OUT_绝对定位执行中 (S15)
    AbsolutePosHolding = 0xE010,   // OUT_绝对定位完成保持 (S16)
    AbsolutePosComplete = 0xE011,  // OUT_绝对定位完成 (S17)
    HomingMoving = 0xE012,         // OUT_回原执行中 (S18)
    HomingHolding = 0xE013,        // OUT_回原完成保持 (S19)
    HomingComplete = 0xE014,       // OUT_回原完成 (S20)
    JogRunning = 0xE015,           // OUT_点动运行中 (S21)
    IStopDone = 0xE016,            // OUT_急停完成 (S22)
    IStopError = 0xE017,           // OUT_急停错误 (S23)
    OREnabled = 0xE018,            // OUT_运动超调有效 (S24)
    ORBusy = 0xE019,               // OUT_运动超调忙 (S25)
    ORError = 0xE01A,              // OUT_运动超调故障 (S26)
    Disabled = 0xE01B,             // OUT_去使能状态 (S27)
    StandStill = 0xE01C,           // OUT_使能非运行 (S28)
    ConstantVelocity = 0xE01D,     // OUT_恒速运动 (S29)
    Accelerating = 0xE01E,         // OUT_加速运动 (S30)
    Decelerating = 0xE01F,         // OUT_减速运动 (S31)
};

constexpr int addr(Axis axis, RegS reg) { return static_cast<int>(reg) + axisIndex(axis) * AXIS_S_OFFSET; }

/* =========================
 *  D 区（数据寄存器，OUT 数值）
 * ========================= */
enum class RegD : int {
    CurrentPosition = 0x0000,  // OUT_当前位置 (D0)
    CurrentSpeed = 0x0002,     // OUT_当前速度 (D2)
    CurrentTorque = 0x0004,    // OUT_当前转矩 (D4)
    DriverFaultCode = 0x0006,  // OUT_驱动器故障码 (D6)
    AxisFaultCode = 0x0007,    // OUT_轴故障码 (D7)
    CurrentValue = 0x0008,     // OUT_电流 (D8)
};

constexpr int addr(Axis axis, RegD reg) { return static_cast<int>(reg) + axisIndex(axis) * AXIS_D_OFFSET; }
#endif  // AXIS_REGISTER_H
