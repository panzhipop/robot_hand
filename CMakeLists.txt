cmake_minimum_required(VERSION 3.10)
project(RoboticHandProject)

# 找到並加入 Dynamixel SDK (假設您已安裝)
find_package(dynamixel_sdk REQUIRED)

# 將 main.cpp 和您的 class 實作檔一起編譯成執行檔
add_executable(my_robot_exe 
    main.cpp 
    RoboticHand.cpp
)

# 連結 Dynamixel 函式庫
target_link_libraries(my_robot_exe dxl_cpp)