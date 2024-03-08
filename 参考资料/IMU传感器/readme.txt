1、IMU_x.xx.exe 为windows下的全功能上位机演示工具(绿色工具无需安装，双击即可运行)，可以使用串口或者蓝牙的方式连接传感器进行各种功能和性能的测试，以便用户快速评估产品的功能和性能是否达到自己项目的目标要求。传感器的集成功能很多，所以强烈推荐新用户先通过该工具来快速熟悉了解产品的功能和性能，有了对传感器的清晰认知后再来进行自己的产品开发。注意若windows下使用BLE蓝牙，需要带蓝牙功能的windows8以上系统(因为从windows8开始才原生支持BLE功能)，台式机可以用USB蓝牙适配器。
2、若IMU上位机打开时提示缺少Net Framework 运行库，那可能是系统精简过缺少Net Framework 组件，或组件版本太低。可以尝试安装 NET Framework 4.8运行库-x86-x64-allos-enu.exe 运行库解决。一般windows10以上电脑自带该运行库的了。

3、C#的蓝牙搜索连接示例源码_BleSolution
为VS开发环境下的C#蓝牙BLE示例代码，有蓝牙搜索、过滤设备、连接设备、断开设备、简单收发数据功能，给用户做为BLE的开发参考，更详细的示例请联系客服。

4、安卓测试_nrfconnect-V4.24.1.apk
为安卓下的通用BLE测试工具，可以方便看到广播细节、搜索发现设备、连接设备、断开设备、查看UUID属性、自定义收发数据等等，大多数从事BLE蓝牙开发的用户都喜欢使用该工具进行BLE相关的开发调试。

5、ROS_imu_ws.zip 为ROS机器人系统下的一个节点示例源码。

6、imu_bluetooth_Python_ForLinux.rar 为linux下的Python蓝牙连接使用示例
7、imu_bluetooth_Python_ForWindows.rar 为Windows10下的Python蓝牙连接使用示例
8、imu_Serial_Python_ForWindowsAndLinux.rar 为windows和linux下的python串口连接示例

9、IM948_SDK1.03_Stm32F103RET6.rar  这是嵌入式c语言相关的通过串口通信操作传感器的示例基于stm32标准库，需要的用户可以参考或移植使用。
   IM948_SDK1.03_Stm32F103C8T6_HAL  基于stm32的HAL库
   IM948_SDK1.03_Stm32F401_HAL      基于stm32的HAL库
   IM948_SDK1.03_Stm32F407VE_HAL    基于stm32的HAL库
   
10、IM948_ArduinoTest_V1.03.rar  这是arduino的参考源码

11、数据体解析方法示例  这个目录里有一些不同语言的数据体解析方法，以供大家参考或移植使用

12、imu_parse_V1.0.exe 这是一个填入imu主动上报的数据包，自动显示数据解析过程和结果的小工具，方便用户开发调试时更好理解数据协议

13、这是一个可以把上位机采集的文本数据，转换为excel或csv文件的脚本工具。