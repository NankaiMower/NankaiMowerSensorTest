// 基本数据类型用于简化书写
typedef signed char            S8;
typedef unsigned char          U8;
typedef signed short           S16;   
typedef unsigned short         U16;  
typedef signed long            S32;   
typedef unsigned long          U32;  
typedef float                  F32;   

// 传输时转换比例--------------
#define scaleAccel       0.00478515625f // 加速度 [-16g~+16g]    9.8*16/32768
#define scaleQuat        0.000030517578125f // 四元数 [-1~+1]         1/32768
#define scaleAngle       0.0054931640625f // 角度   [-180~+180]     180/32768
#define scaleAngleSpeed  0.06103515625f // 角速度 [-2000~+2000]    2000/32768
#define scaleMag         0.15106201171875f // 磁场 [-4950~+4950]   4950/32768
#define scaleTemperature 0.01f // 温度
#define scaleAirPressure 0.0002384185791f // 气压 [-2000~+2000]    2000/8388608
#define scaleHeight      0.0010728836f    // 高度 [-9000~+9000]    9000/8388608


#define pow2(x) ((x)*(x)) // 求平方

/**
 * 解析接收到报文的数据体并处理，用户根据项目需求，关注里面对应的内容即可--------------------
 * @param buf 把收到的数据包的数据体，传入到该指针
 */
static void Cmd_RxUnpack(U8 *buf)
{
    U16 ctl; // 数据订阅标识 标签0x11功能用到
    U8 L; // 标签0x11功能用到
    U8 tmpU8;   // 1个8位数，方便后续使用与解析数据
    U16 tmpU16; // 1个无符号16位数，方便后续使用与解析数据
    U32 tmpU32; // 1个无符号32位数，方便后续使用与解析数据
    F32 tmpX, tmpY, tmpZ, tmpAbs; // 4个单精度浮点数，方便后续使用于解析数据

    switch (buf[0]) // bug[0]为数据体的第1字节表示功能标签
    {
    case 0x02: // 传感器 已睡眠 回复
        printf("\t sensor off success\r\n");
        break;
    case 0x03: // 传感器 已唤醒 回复
        printf("\t sensor on success\r\n");
        break;
    case 0x32: // 磁力计 开始校准 回复
        printf("\t compass calibrate begin\r\n");
        break;
    case 0x04: // 磁力计 结束校准 回复
        printf("\t compass calibrate end\r\n");
        break;
    case 0x05: // z轴角 已归零 回复
        printf("\t z-axes to zero success\r\n");
        break;
    case 0x06: // 请求 xyz世界坐标系清零 回复
        printf("\t WorldXYZ-axes to zero success\r\n");
        break;
    case 0x07: // 加速计简单校准正在进行，将在9秒后完成  回复
        printf("\t acceleration calibration, Hold still for 9 seconds\r\n");
        break;
    case 0x08: // 恢复默认的自身坐标系Z轴指向及恢复默认的世界坐标系  回复
        printf("\t axesZ WorldXYZ-axes to zero success\r\n");
        break;
    case 0x10: // 模块当前的属性和状态 回复
        printf("\t still limit: %u\r\n", buf[1]);   // 字节1 惯导-静止状态加速度阀值 单位dm/s²
        printf("\t still to zero: %u\r\n", buf[2]); // 字节2 惯导-静止归零速度(单位mm/s) 0:不归零 255:立即归零
        printf("\t move to zero: %u\r\n", buf[3]);  // 字节3 惯导-动态归零速度(单位mm/s) 0:不归零
        printf("\t compass: %s\r\n", ((buf[4]>>0) & 0x01)? "on":"off" );     // 字节4 bit[0]: 1=已开启磁场 0=已关闭磁场
        printf("\t barometer filter: %u\r\n", (buf[4]>>1) & 0x03);           // 字节4 bit[1-2]: 气压计的滤波等级[取值0-3],数值越大越平稳但实时性越差
        printf("\t IMU: %s\r\n", ((buf[4]>>3) & 0x01)? "on":"off" );         // 字节4 bit[3]: 1=传感器已开启  0=传感器已睡眠
        printf("\t auto report: %s\r\n", ((buf[4]>>4) & 0x01)? "on":"off" ); // 字节4 bit[4]: 1=已开启传感器数据主动上报 0=已关闭传感器数据主动上报
        printf("\t FPS: %u\r\n", buf[5]); // 字节5 数据主动上报的传输帧率[取值0-250HZ], 0表示0.5HZ
        printf("\t gyro filter: %u\r\n", buf[6]);    // 字节6 陀螺仪滤波系数[取值0-2],数值越大越平稳但实时性越差
        printf("\t acc filter: %u\r\n", buf[7]);     // 字节7 加速计滤波系数[取值0-4],数值越大越平稳但实时性越差
        printf("\t compass filter: %u\r\n", buf[8]); // 字节8 磁力计滤波系数[取值0-9],数值越大越平稳但实时性越差
        printf("\t subscribe tag: 0x%04X\r\n", (U16)(((U16)buf[10]<<8) | buf[9])); // 字节[10-9] 功能订阅标识
        printf("\t charged state: %u\r\n", buf[11]); // 字节11 充电状态指示 0=未接电源 1=充电中 2=已充满
        printf("\t battery level: %u%%\r\n", buf[12]); // 字节12 当前剩余电量[0-100%]
        printf("\t battery voltage: %u mv\r\n", (U16)(((U16)buf[14]<<8) | buf[13])); // 字节[14-13] 电池的当前电压mv
        printf("\t Mac: %02X:%02X:%02X:%02X:%02X:%02X\r\n", buf[15],buf[16],buf[17],buf[18],buf[19],buf[20]); // 字节[15-20] MAC地址
        printf("\t version: %s\r\n", &buf[21]); // 字节[21-26] 固件版本 字符串
        printf("\t product model: %s\r\n", &buf[27]); // 字节[27-32] 产品型号 字符串
        break;
    case 0x11: // 获取订阅的功能数据 回复或主动上报
        ctl = ((U16)buf[2] << 8) | buf[1];// 字节[2-1] 为功能订阅标识，指示当前订阅了哪些功能
        printf("\t subscribe tag: 0x%04X\r\n", ctl);
        printf("\t ms: %u\r\n", (U32)(((U32)buf[6]<<24) | ((U32)buf[5]<<16) | ((U32)buf[4]<<8) | ((U32)buf[3]<<0))); // 字节[6-3] 为模块开机后的时间戳(单位ms)

        L =7; // 从第7字节开始根据 订阅标识tag来解析剩下的数据
        if ((ctl & 0x0001) != 0)
        {// 加速度xyz 去掉了重力 使用时需*scaleAccel m/s
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; printf("\taX: %.3f\r\n", tmpX); // x加速度aX
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; printf("\taY: %.3f\r\n", tmpY); // y加速度aY
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; printf("\taZ: %.3f\r\n", tmpZ); // z加速度aZ
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); printf("\ta_abs: %.3f\r\n", tmpAbs); // 3轴合成的绝对值
        }
        if ((ctl & 0x0002) != 0)
        {// 加速度xyz 包含了重力 使用时需*scaleAccel m/s
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; printf("\tAX: %.3f\r\n", tmpX); // x加速度AX
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; printf("\tAY: %.3f\r\n", tmpY); // y加速度AY
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; printf("\tAZ: %.3f\r\n", tmpZ); // z加速度AZ
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); printf("\tA_abs: %.3f\r\n", tmpAbs); // 3轴合成的绝对值
        }
        if ((ctl & 0x0004) != 0)
        {// 角速度xyz 使用时需*scaleAngleSpeed °/s
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngleSpeed; L += 2; printf("\tGX: %.3f\r\n", tmpX); // x角速度GX
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngleSpeed; L += 2; printf("\tGY: %.3f\r\n", tmpY); // y角速度GY
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngleSpeed; L += 2; printf("\tGZ: %.3f\r\n", tmpZ); // z角速度GZ
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); printf("\tG_abs: %.3f\r\n", tmpAbs); // 3轴合成的绝对值
        }
        if ((ctl & 0x0008) != 0)
        {// 磁场xyz 使用时需*scaleMag uT
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleMag; L += 2; printf("\tCX: %.3f\r\n", tmpX); // x磁场CX
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleMag; L += 2; printf("\tCY: %.3f\r\n", tmpY); // y磁场CY
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleMag; L += 2; printf("\tCZ: %.3f\r\n", tmpZ); // z磁场CZ
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); printf("\tC_abs: %.3f\r\n", tmpAbs); // 3轴合成的绝对值
        }
        if ((ctl & 0x0010) != 0)
        {// 温度 气压 高度
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleTemperature; L += 2; printf("\ttemperature: %.2f\r\n", tmpX); // 温度

            tmpU32 = (U32)(((U32)buf[L+2] << 16) | ((U32)buf[L+1] << 8) | (U32)buf[L]);
            tmpU32 = ((tmpU32 & 0x800000) == 0x800000)? (tmpU32 | 0xff000000) : tmpU32;// 若24位数的最高位为1则该数值为负数，需转为32位负数，直接补上ff即可
            tmpY = (S32)tmpU32 * scaleAirPressure; L += 3; printf("\tairPressure: %.3f\r\n", tmpY); // 气压

            tmpU32 = (U32)(((U32)buf[L+2] << 16) | ((U32)buf[L+1] << 8) | (U32)buf[L]);
            tmpU32 = ((tmpU32 & 0x800000) == 0x800000)? (tmpU32 | 0xff000000) : tmpU32;// 若24位数的最高位为1则该数值为负数，需转为32位负数，直接补上ff即可
            tmpZ = (S32)tmpU32 * scaleHeight; L += 3; printf("\theight: %.3f\r\n", tmpZ); // 高度
        }
        if ((ctl & 0x0020) != 0)
        {// 四元素 wxyz 使用时需*scaleQuat
            tmpAbs = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleQuat; L += 2; printf("\tw: %.3f\r\n", tmpAbs); // w
            tmpX =   (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleQuat; L += 2; printf("\tx: %.3f\r\n", tmpX); // x
            tmpY =   (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleQuat; L += 2; printf("\ty: %.3f\r\n", tmpY); // y
            tmpZ =   (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleQuat; L += 2; printf("\tz: %.3f\r\n", tmpZ); // z
        }
        if ((ctl & 0x0040) != 0)
        {// 欧拉角xyz 使用时需*scaleAngle
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngle; L += 2; printf("\tangleX: %.3f\r\n", tmpX); // x角度
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngle; L += 2; printf("\tangleY: %.3f\r\n", tmpY); // y角度
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngle; L += 2; printf("\tangleZ: %.3f\r\n", tmpZ); // z角度
        }
        if ((ctl & 0x0080) != 0)
        {// xyz 空间位移 单位mm 转为 m
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) / 1000.0f; L += 2; printf("\toffsetX: %.3f\r\n", tmpX); // x坐标
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) / 1000.0f; L += 2; printf("\toffsetY: %.3f\r\n", tmpY); // y坐标
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) / 1000.0f; L += 2; printf("\toffsetZ: %.3f\r\n", tmpZ); // z坐标
        }
        if ((ctl & 0x0100) != 0)
        {// 活动检测数据
            tmpU32 = (U32)(((U32)buf[L+3]<<24) | ((U32)buf[L+2]<<16) | ((U32)buf[L+1]<<8) | ((U32)buf[L]<<0)); L += 4; printf("\tsteps: %u\r\n", tmpU32); // 计步数
            tmpU8 = buf[L]; L += 1;
            printf("\t walking: %s\r\n", (tmpU8 & 0x01)?  "yes" : "no"); // 是否在走路
            printf("\t running: %s\r\n", (tmpU8 & 0x02)?  "yes" : "no"); // 是否在跑步
            printf("\t biking: %s\r\n",  (tmpU8 & 0x04)?  "yes" : "no"); // 是否在骑车
            printf("\t driving: %s\r\n", (tmpU8 & 0x08)?  "yes" : "no"); // 是否在开车
        }
        if ((ctl & 0x0200) != 0)
        {// 加速度xyz 去掉了重力且已转为导航系 使用时需*scaleAccel m/s
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; printf("\tasX: %.3f\r\n", tmpX); // x加速度asX
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; printf("\tasY: %.3f\r\n", tmpY); // y加速度asY
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; printf("\tasZ: %.3f\r\n", tmpZ); // z加速度asZ
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); printf("\tas_abs: %.3f\r\n", tmpAbs); // 3轴合成的绝对值
        }
        if ((ctl & 0x0400) != 0)
        {// ADC的值
            tmpU16 = (U16)(((U16)buf[L+1]<<8) | ((U16)buf[L]<<0)); L += 2; printf("\tadc: %u\r\n", tmpU16); // 10位精度ADC的电压值(0-VDDIO) mv
        }
        if ((ctl & 0x0800) != 0)
        {// GPIO1的值
            tmpU8 = buf[L]; L += 1;
            printf("\t GPIO1  M:%X, N:%X\r\n", (tmpU8>>4)&0x0f, (tmpU8)&0x0f);
        }
        break;
    case 0x12: // 设置参数 回复
        printf("\t set parameters success\r\n");
        break;
    case 0x13: // 惯导三维空间位置清零 回复
        printf("\t clear INS position success\r\n");
        break;
    case 0x14: // 恢复出厂校准参数 回复
        printf("\t Restore calibration parameters from factory mode success\r\n");
        break;
    case 0x15: // 保存当前校准参数为出厂校准参数 回复
        printf("\t Save calibration parameters to factory mode success\r\n");
        break;
    case 0x16: // 计步数清零 回复
        printf("\t clear steps success\r\n");
        break;
    case 0x17: // 加速计高精度校准 回复
        if (buf[1] == 255)
        {// 字节1 值255 表示采集完成，正在结束校准(设备需继续保持静止等待10秒钟)
            printf("\t calibration success, please wait 10 seconds\r\n");
        }
        else if (buf[1] == 254)
        {// 字节1 值254 表示陀螺仪自检失败
            printf("\t calibration fail, gyro error\r\n");
        }
        else if (buf[1] == 253)
        {// 字节1 值253 表示加速计自检失败
            printf("\t calibration fail, accelerometer error\r\n");
        }
        else if (buf[1] == 252)
        {// 字节1 值252 表示磁力计自检失败
            printf("\t calibration fail, compass error\r\n");
        }
        else if (buf[1] == 251)
        {// 字节1 值251 表示设备未在校准中
            printf("\t calibration fail, Hasn't started\r\n");
        }
        else if (buf[1] != 0)
        {// 值[1-250] 表示当前已采集的次数
            printf("\t calibration, Points collected is %u\r\n", buf[1]);
        }
        else
        {// 值0 表示模块已经在校准中
            printf("\t calibration is running\r\n");
        }
        break;
    case 0x18: // 已关闭主动上报 回复
        printf("\t auto report off\r\n");
        break;
    case 0x19: // 已打开主动上报 回复
        printf("\t auto report on\r\n");
        break;
    case 0x20: // 设置PCB安装方向矩阵 回复
        printf("\t set PCB direction success\r\n");
        break;
    case 0x21: // 是请求 读取安装方向矩阵
        Dbp_U8_buf("\t get PCB direction: 0x[", "]\r\n",
                   "%02x ",
                   &buf[1], 9); // 字节[1-9]     为加速计安装方向矩阵
        Dbp_U8_buf("\t get PCB direction: 0x[", "]\r\n",
                   "%02x ",
                   &buf[10], 9); // 字节[10-18] 为磁力计安装方向矩阵
        break;
    case 0x22: // 是请求 设置蓝牙广播名称
        printf("\t set BLE name success\r\n");
        break;
    case 0x23: // 读取蓝牙广播名称 回复
        printf("\t get BLE name: %s\r\n", &buf[1]); // 字节[1-16] 为蓝牙广播名称字符串
        break;
    case 0x24: // 设置关机电压和充电参数 回复
        printf("\t set PowerDownVoltage and charge parameters success\r\n");
        break;
    case 0x25: // 读取关机电压和充电参数 回复
        printf("\t PowerDownVoltageFlag: %u\r\n", buf[1]); // 字节1 关机电压选择标志 0表示3.4V, 1表示2.7V
        printf("\t charge_full_mV: %u\r\n", buf[2]); // 字节2 充电截止电压 0:3962mv 1:4002mv 2:4044mv 3:4086mv 4:4130mv 5:4175mv 6:4222mv 7:4270mv 8:4308mv 9:4349mv 10:4391mv
        printf("\t charge_full_mA: %u ma\r\n", buf[3]); // 字节3 充电截止电流 0:2ma 1:5ma 2:7ma 3:10ma 4:15ma 5:20ma 6:25ma 7:30ma
        printf("\t charge_mA: %u ma\r\n", buf[4]); // 字节3 充电电流 0:20ma 1:30ma 2:40ma 3:50ma 4:60ma 5:70ma 6:80ma 7:90ma 8:100ma 9:110ma 10:120ma 11:140ma 12:160ma 13:180ma 14:200ma 15:220ma
        break;
    case 0x27: // 设置用户的GPIO引脚 回复
        printf("\t set gpio success\r\n");
        break;
    case 0x2A: // 重启设备 回复
        printf("\t will reset\r\n");
        break;
    case 0x2B: // 设备关机 回复
        printf("\t will power off\r\n");
        break;
    case 0x2C: // 设置空闲关机时长 回复
        printf("\t set idleToPowerOffTime success\r\n");
        break;
    case 0x2D: // 读取空闲关机时长 回复
        printf("\t idleToPowerOffTime:%u minutes\r\n", buf[1]*10);
        break;
    case 0x2E: // 设置禁止蓝牙方式更改名称和充电参数标识 回复
        printf("\t set FlagForDisableBleSetNameAndCahrge success\r\n");
        break;
    case 0x2F: // 读取禁止蓝牙方式更改名称和充电参数标识 回复
        printf("\t FlagForDisableBleSetNameAndCahrge:%u\r\n", buf[1]);
        break;
    case 0x30: // 设置串口通信地址 回复
        printf("\t set address success\r\n");
        break;
    case 0x31: // 读取串口通信地址 回复
        printf("\t address:%u\r\n", buf[1]);
        break;
    case 0x33: // 设置加速计和陀螺仪量程 回复
        printf("\t set accelRange and gyroRange success\r\n");
        break;
    case 0x34: // 读取加速计和陀螺仪量程 回复
        printf("\t accelRange:%u gyroRange:%u\r\n", buf[1], buf[2]);
        break;
    case 0x35: // 设置陀螺仪自动校正标识 回复
        printf("\t set GyroAutoFlag success\r\n");
        break;
    case 0x36: // 读取陀螺仪自动校正标识 回复
        printf("\t GyroAutoFlag:%u\r\n", buf[1]);
        break;
    case 0x37: // 设置静止节能模式的触发时长 回复
        printf("\t set EcoTime success\r\n");
        break;
    case 0x38: // 读取静止节能模式的触发时长 回复
        printf("\t EcoTime:%u\r\n", buf[1]);
        break;

    default:
        break;
    }
}