#include "RoboticHand.hpp"

RoboticHand::RoboticHand(const std::string& port_name, int baudrate, const std::vector<FingerConfig>& finger_configs)
    : port_name_(port_name), baudrate_(baudrate) {
    
    for (const auto& config : finger_configs) {
        FingerConfig cfg = config;
        // 初始化時，先將手動值設為當前的活動邊界
        cfg.active_min = cfg.manual_min;
        cfg.active_max = cfg.manual_max;
        
        // 如果設定為手動模式，代表它已經準備好可以動了
        if (!cfg.use_auto_calibration) {
            cfg.is_ready = true;
        }
        
        fingers_[cfg.id] = cfg;
    }

    portHandler_ = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);
}

RoboticHand::~RoboticHand() {
    disableTorque();
    portHandler_->closePort();
}

bool RoboticHand::initialize() {
    if (!portHandler_->openPort() || !portHandler_->setBaudRate(baudrate_)) {
        std::cerr << "通訊埠設定失敗！" << std::endl; return false;
    }
    disableTorque();
    for (const auto& pair : fingers_) {
        write1Byte(pair.first, ADDR_OPERATING_MODE, CURRENT_BASED_POSITION_MODE);
    }
    enableTorque();
    return true;
}

bool RoboticHand::getFingerConfig(uint8_t id, FingerConfig& config) const {
    auto it = fingers_.find(id);
    if (it != fingers_.end()) {
        config = it->second;
        return true;
    }
    return false;
}

// 自動校正邏輯 (兼容手動與自動)
void RoboticHand::autoCalibrate(int16_t calib_current_mA, int32_t open_target, int32_t close_target) {
    std::cout << "\n[啟動校正程序] 使用安全電流: " << calib_current_mA << "mA" << std::endl;
    setAllFingersCurrent(calib_current_mA);

    for (auto& pair : fingers_) {
        uint8_t id = pair.first;
        FingerConfig& cfg = pair.second;

        // 如果設定檔指定要用「手動數值」，則直接跳過不校正
        if (!cfg.use_auto_calibration) {
            std::cout << ">> 手指 ID " << (int)id << " 使用手動設定 (Min: " 
                      << cfg.manual_min << " Max: " << cfg.manual_max << ")，跳過自動校正。" << std::endl;
            continue;
        }

        std::cout << ">> 正在自動校正手指 ID " << (int)id << " ..." << std::endl;

        write4Byte(id, ADDR_GOAL_POSITION, open_target);
        int32_t actual_min = waitForStallAndGetPosition(id);
        
        write4Byte(id, ADDR_GOAL_POSITION, close_target);
        int32_t actual_max = waitForStallAndGetPosition(id);

        // 寫入實際測量的數值 (含 +-20 安全邊距)
        cfg.active_min = std::min(actual_min, actual_max) + 20;
        cfg.active_max = std::max(actual_min, actual_max) - 20;
        cfg.is_ready = true;

        std::cout << "   完成! [自動測量範圍] Min: " << cfg.active_min 
                  << " ~ Max: " << cfg.active_max << std::endl;
        
        // write4Byte(id, ADDR_GOAL_POSITION, (cfg.active_min + cfg.active_max) / 2);
        usleep(10000); 
    }
    std::cout << "[校正程序結束] 所有手指已準備就緒！\n" << std::endl;
}

int32_t RoboticHand::waitForStallAndGetPosition(uint8_t id) {
    uint8_t dxl_error = 0;
    int32_t last_pos = -9999;
    int32_t current_pos = 0;
    int stall_count = 0;

    while (true) {
        packetHandler_->read4ByteTxRx(portHandler_, id, ADDR_PRESENT_POSITION, (uint32_t*)&current_pos, &dxl_error);
        if (abs(current_pos - last_pos) < 3) stall_count++;
        else stall_count = 0; 
        if (stall_count >= 5) return current_pos;
        last_pos = current_pos;
        usleep(50000); 
    }
}

// 保護機制：統一使用 active_min 和 active_max 來做夾緊/放鬆的極限
void RoboticHand::setFingerPosition(uint8_t id, int32_t position) {
    if (fingers_.find(id) != fingers_.end()) {
        int32_t safe_position = position;
        
        // 只有在手指 is_ready 時，才啟用極限保護
        if (fingers_[id].is_ready) {
            safe_position = std::clamp(position, fingers_[id].active_min, fingers_[id].active_max);
        }
        write4Byte(id, ADDR_GOAL_POSITION, safe_position);
    }
}

// (以下省略基本功能，與上一版相同...)
void RoboticHand::enableTorque() { for (const auto& pair : fingers_) write1Byte(pair.first, ADDR_TORQUE_ENABLE, 1); }
void RoboticHand::disableTorque() { for (const auto& pair : fingers_) write1Byte(pair.first, ADDR_TORQUE_ENABLE, 0); }
void RoboticHand::setFingerCurrent(uint8_t id, int16_t current_mA) { write2Byte(id, ADDR_GOAL_CURRENT, current_mA); }
void RoboticHand::setAllFingersCurrent(int16_t current_mA) { for (const auto& pair : fingers_) setFingerCurrent(pair.first, current_mA); }
void RoboticHand::setAllFingersPosition(int32_t position) { for (const auto& pair : fingers_) setFingerPosition(pair.first, position); }

void RoboticHand::printMotorStatus() {
    uint8_t dxl_error = 0;
    for (const auto& pair : fingers_) {
        uint8_t id = pair.first;
        int16_t present_current = 0;
        int32_t present_position = 0;
        packetHandler_->read2ByteTxRx(portHandler_, id, ADDR_PRESENT_CURRENT, (uint16_t*)&present_current, &dxl_error);
        packetHandler_->read4ByteTxRx(portHandler_, id, ADDR_PRESENT_POSITION, (uint32_t*)&present_position, &dxl_error);
        
        std::cout << "[ID:" << (int)id << "] Pos: " << present_position 
                  << " Cur: " << present_current << " | ";
    }
    std::cout << std::endl;
}

bool RoboticHand::write1Byte(uint8_t id, uint16_t addr, uint8_t data) {
    uint8_t error; return packetHandler_->write1ByteTxRx(portHandler_, id, addr, data, &error) == COMM_SUCCESS;
}
bool RoboticHand::write2Byte(uint8_t id, uint16_t addr, uint16_t data) {
    uint8_t error; return packetHandler_->write2ByteTxRx(portHandler_, id, addr, data, &error) == COMM_SUCCESS;
}
bool RoboticHand::write4Byte(uint8_t id, uint16_t addr, uint32_t data) {
    uint8_t error; return packetHandler_->write4ByteTxRx(portHandler_, id, addr, data, &error) == COMM_SUCCESS;
}