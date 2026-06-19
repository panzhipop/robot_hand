#ifndef ROBOTIC_HAND_HPP
#define ROBOTIC_HAND_HPP

#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <algorithm>
#include <unistd.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

// 手指設定檔
struct FingerConfig {
    uint8_t id;
    
    // --- 手動設定區 ---
    int32_t manual_min = 0;
    int32_t manual_max = 4095;
    
    // --- 模式切換 ---
    // 如果為 true，則執行 autoCalibrate 時會覆寫極限值
    // 如果為 false，則嚴格使用 manual_min / manual_max
    bool use_auto_calibration = false; 

    // --- 內部狀態 (請勿手動修改) ---
    int32_t active_min = 0; 
    int32_t active_max = 4095;
    bool is_ready = false; // 是否已經擁有可用的極限值
};

class RoboticHand {
public:
    // 建構子現在接收 FingerConfig 陣列
    RoboticHand(const std::string& port_name, int baudrate, const std::vector<FingerConfig>& finger_configs);
    ~RoboticHand();

    bool initialize();
    void enableTorque();
    void disableTorque();

    // 自動校正程序 (只會校正 use_auto_calibration == true 的手指)
    void autoCalibrate(int16_t calib_current_mA = 50, int32_t open_target = 0, int32_t close_target = 4095);

    // 控制函式
    void setFingerCurrent(uint8_t id, int16_t current_mA);
    void setFingerPosition(uint8_t id, int32_t position);
    void setAllFingersCurrent(int16_t current_mA);
    void setAllFingersPosition(int32_t position);

    void printMotorStatus();
    bool getFingerConfig(uint8_t id, FingerConfig& config) const;
private:
    std::string port_name_;
    int baudrate_;
    std::map<uint8_t, FingerConfig> fingers_; 

    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;

    // 硬體暫存器位址 (XC330)
    static const uint16_t ADDR_OPERATING_MODE   = 11;
    static const uint16_t ADDR_TORQUE_ENABLE    = 64;
    static const uint16_t ADDR_GOAL_CURRENT     = 102;
    static const uint16_t ADDR_GOAL_POSITION    = 116;
    static const uint16_t ADDR_PRESENT_CURRENT  = 126;
    static const uint16_t ADDR_PRESENT_POSITION = 132;
    static const uint8_t CURRENT_BASED_POSITION_MODE = 5;

    int32_t waitForStallAndGetPosition(uint8_t id);

    bool write1Byte(uint8_t id, uint16_t addr, uint8_t data);
    bool write2Byte(uint8_t id, uint16_t addr, uint16_t data);
    bool write4Byte(uint8_t id, uint16_t addr, uint32_t data);
};

#endif // ROBOTIC_HAND_HPP