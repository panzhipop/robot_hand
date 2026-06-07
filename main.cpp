#include <iostream>
#include <unistd.h>
#include "RoboticHand.hpp"


int main() {
// 1. 混合模式的手指設定 (測試階段神器)
    // 參數順序: { ID, 手動Min, 手動Max, 是否啟用自動校正 }
    std::vector<FingerConfig> hand_config = {
        // true => 自動設定
        {1, 0, 4095, true}, 
        {2, 0, 4095, true}, 
        {3, 0, 4095, true}, 
        {4, 0, 4095, true}, 
        {5, 0, 4095, true}, 
        
        // false => 人工設定
        // {2, 0, 1800, false}, 
        // {3, 0, 1600, false},     
        // {4, 300, 2000, false},
        // {5, 400, 2000, false}
    };

    // 2. 初始化機械手
    RoboticHand myHand("/dev/ttyUSB0", 57600, hand_config);

    if (!myHand.initialize()) {
        return -1;
    }

    // 3. 呼叫校正程序 
    // (程式看到 ID 2, 3, 4, 5 會直接跳過，只會移動 ID 1 去找邊界)
    myHand.autoCalibrate(100, 0, 4095);

    std::cout << "--- 開始正常運作 ---" << std::endl;
    myHand.setAllFingersCurrent(100); 
    
    return 0;
}



// std::vector<FingerConfig> hand_config = {
//     {1, 0, 2100}, // 假設 ID 1 是大拇指 (行程較短)
//     {2, 0, 1800}, // 假設 ID 2 是食指
//     {3, 0, 1600}, // 假設 ID 3 是中指 (行程最長)
//     {4, 300, 2000}, // 假設 ID 4 是無名指
//     {5, 400, 2000}  // 假設 ID 5 是小拇指 (行程較短)
// };

// std::cout << "\n--- 測試 2: 單獨控制 (比出 OK 的手勢) ---" << std::endl;
//     // 拇指與食指閉合 (OK 圓圈)
//     myHand.setFingerPosition(1, 2048); 
//     myHand.setFingerPosition(2, 3000);
    
//     // 中、無名、小指張開
//     myHand.setFingerPosition(3, 1000);
//     myHand.setFingerPosition(4, 1000);
//     myHand.setFingerPosition(5, 1200);