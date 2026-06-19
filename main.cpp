#include <iostream>
#include <unistd.h>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <algorithm>
#include "RoboticHand.hpp"

// 定義與 ESP32 完全相同的資料結構
struct struct_message {
    int fingers[5];
};

long mapVal(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main() {
    // ===== 1. 網路設定 (UDP Server) =====
    int server_fd;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    int port = 12345; // 與 ESP32 相同的 Port

    // 建立 UDP Socket
    if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "UDP Socket 建立失敗！" << std::endl;
        return -1;
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY; // 監聽所有網路介面
    address.sin_port = htons(port);

    // 綁定 Port
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        std::cerr << "Socket 綁定 Port " << port << " 失敗！" << std::endl;
        close(server_fd);
        return -1;
    }

    // ===== 2. 機械手硬體設定 =====
    // 參數順序: { ID, 手動Min, 手動Max, 是否啟用自動校正 }
    std::vector<FingerConfig> hand_config = {
        {1, 0, 4095, true}, 
        {2, 0, 4095, true}, 
        {3, 0, 4095, true}, 
        {4, 0, 4095, true}, 
        {5, 0, 4095, true}
    };

    // 初始化機械手 (U2D2 在樹莓派通常是 /dev/ttyUSB0)
    RoboticHand myHand("/dev/ttyUSB0", 57600, hand_config);

    if (!myHand.initialize()) {
        std::cerr << "機器手初始化失敗，請檢查 U2D2 連線或執行 sudo chmod 666 /dev/ttyUSB0" << std::endl;
        close(server_fd);
        return -1;
    }

    // 3. 呼叫自動校正程序
    // 注意：開啟此程序時，機器手會動，請確保周圍安全
    myHand.autoCalibrate(100, 0, 4095);

    std::cout << "\n--- 校正結束，開始進行 Wi-Fi 即時同步控制 ---" << std::endl;
    std::cout << "正在監聽 Port " << port << " 接收手套數據..." << std::endl;
    
    // 設定正常運作時的馬達安全電流限制 (可根據需求微調)
    myHand.setAllFingersCurrent(100); 

    struct_message receivedData;
    FingerConfig cfg; // 用來暫存取出的手指設定

    // ===== 4. 即時控制主迴圈 =====
    struct_message temp_data;
    bool has_new_data = false;

    while (true) {
        has_new_data = false;

        // 核心修改：使用 MSG_DONTWAIT 不斷抽乾(Drain)緩衝區
        // 直到抽不到東西 (回傳 -1)，此時 receivedData 裡存的就是「最新」的封包
        while (true) {
            ssize_t len = recvfrom(server_fd, &temp_data, sizeof(temp_data), MSG_DONTWAIT, nullptr, nullptr);
            if (len == sizeof(struct_message)) {
                receivedData = temp_data;
                has_new_data = true;
            } else {
                break; // 緩衝區已清空，跳出內部迴圈
            }
        }

        // 只有當我們真正拿到最新數據時，才去驅動馬達
        if (has_new_data) {
            // 手指 1、2、3 (反向映射)
            // for (uint8_t id = 1; id <= 3; ++id) {
            //     if (myHand.getFingerConfig(id, cfg) && cfg.is_ready) {
            //         int32_t target = mapVal(receivedData.fingers[id - 1], 0, 4095, cfg.active_max, cfg.active_min);
            //         myHand.setFingerPosition(id, target);
            //     }
            // }

            // // 手指 4、5 (正向映射)
            // for (uint8_t id = 4; id <= 5; ++id) {
            //     if (myHand.getFingerConfig(id, cfg) && cfg.is_ready) {
            //         int32_t target = mapVal(receivedData.fingers[id - 1], 0, 4095, cfg.active_min, cfg.active_max);
            //         myHand.setFingerPosition(id, target);
            //     }
            // }

            if (myHand.getFingerConfig(1, cfg) && cfg.is_ready) {
                int32_t target = mapVal(receivedData.fingers[1 - 1], 0, 4095, cfg.active_max, cfg.active_min);
                myHand.setFingerPosition(1, target);
            }  
            if (myHand.getFingerConfig(2, cfg) && cfg.is_ready) {
                int32_t target = mapVal(receivedData.fingers[2 - 1], 2833, 4095, cfg.active_max, cfg.active_min);
                myHand.setFingerPosition(2, target);
            }  
            if (myHand.getFingerConfig(3, cfg) && cfg.is_ready) {
                int32_t target = mapVal(receivedData.fingers[3 - 1], 0, 3000, cfg.active_max, cfg.active_min);
                myHand.setFingerPosition(3, target);
            }              
            if (myHand.getFingerConfig(4, cfg) && cfg.is_ready) {
                int32_t target = mapVal(receivedData.fingers[4 - 1], 0, 2783, cfg.active_min, cfg.active_max);
                myHand.setFingerPosition(4, target);
            }               
            if (myHand.getFingerConfig(5, cfg) && cfg.is_ready) {
                int32_t target = mapVal(receivedData.fingers[5 - 1], 605, 2467, cfg.active_min, cfg.active_max);
                myHand.setFingerPosition(5, target);
            }     
        } else {
            // 如果沒收到封包，微小休眠 1 毫秒避免 CPU 100% 滿載
            usleep(1000); 
        }
    }
    
    // ===== 4. 即時控制主迴圈 =====
    // while (true) {
    //     ssize_t len = recvfrom(server_fd, &receivedData, sizeof(receivedData), 0, nullptr, nullptr);
        
    //     if (len == sizeof(struct_message)) {
    //         // 列印收到的原始數據 debug
    //         std::cout << "\r[收到數據] ID1: " << receivedData.fingers[0]
    //                   << " | ID2: " << receivedData.fingers[1]
    //                   << " | ID3: " << receivedData.fingers[2]
    //                   << " | ID4: " << receivedData.fingers[3]
    //                   << " | ID5: " << receivedData.fingers[4] << "    " << std::flush;

    //         // 👉 更新：依照各手指的正反向，動態帶入 active_min 與 active_max
            
    //         // 手指 1、2、3 (反向映射：手套 0 -> 馬達 active_max, 手套 4095 -> 馬達 active_min)
    //         for (uint8_t id = 1; id <= 3; ++id) {
    //             if (myHand.getFingerConfig(id, cfg) && cfg.is_ready) {
    //                 int32_t target = mapVal(receivedData.fingers[id - 1], 0, 4095, cfg.active_max, cfg.active_min);
    //                 myHand.setFingerPosition(id, target);
    //             }
    //         }

    //         // 手指 4、5 (正向映射：手套 0 -> 馬達 active_min, 手套 4095 -> 馬達 active_max)
    //         for (uint8_t id = 4; id <= 5; ++id) {
    //             if (myHand.getFingerConfig(id, cfg) && cfg.is_ready) {
    //                 int32_t target = mapVal(receivedData.fingers[id - 1], 0, 4095, cfg.active_min, cfg.active_max);
    //                 myHand.setFingerPosition(id, target);
    //             }
    //         }
    //     }
    // }

    close(server_fd);
    return 0;
}