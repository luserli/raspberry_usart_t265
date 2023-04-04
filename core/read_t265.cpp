/*
    使用Intel RealSense T265相机获取姿态数据，并通过串口将数据发送给外部设备。
    程序的主要流程如下：
        1. 初始化RealSense相机和GPIO。
        2. 获取RealSense相机姿态数据。
        3. 将姿态数据转换为整型数组，并在数组中插入针头针尾数据。
        4. 将整型数组的数据通过串口发送给外部设备。
        5. 可选地，使用OpenCV库显示左右目图像数据。
    其中使用了wiringPi库和wiringSerial库来进行GPIO和串口的初始化和操作。
    在获取姿态数据后，使用了一个自定义的float2int函数将数据转换为整型，并使用insert_element函数在数组中插入针头针尾数据。
    最后，使用了serialPrintf函数将整型数组的数据以十六进制格式发送给外部设备。
    如果需要显示左右目图像数据，则可以使用OpenCV库来进行显示。
*/
#include<iostream>
#include<string>

#include <stdio.h>
#include <stdlib.h>

#include <wiringPi.h>//gpio
#include <wiringSerial.h>//usart

#include <librealsense2/rs.hpp>//t265

#define ENABLE_VISION false //是否调用图像数据进行显示，默认关闭
// 如果需要调用图像数据进行测试则设为true，并且将cmake文件中的USE_OPENCV设为true

// 需要显示图像数据则使用cv库
#if ENABLE_VISION
    #include <opencv2/opencv.hpp>
    #include<opencv2/core/core.hpp>
    #include<opencv2/highgui/highgui.hpp>
    using namespace cv;
#endif

#define ARR_N 18 //ARR_N其值为数据数组元素数加6（6为插入的针头针尾数据数）
int* data_conversion(float* num, int n);// 定义函数对原始数据进行处理，转换成需要的数据格式
void data_print(const char* data_name[12], float* imu_data, int mode);

using namespace std;

int main(int argc,char** argv)
{
    rs2::config cfg;

    // 使能 左右目图像数据
    // 就算不使用opencv查看图像数据也要开启使能，因为t265的数据是根据图像数据进行计算得到的
    cfg.enable_stream(RS2_STREAM_FISHEYE,1, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_FISHEYE,2, RS2_FORMAT_Y8);

    // 使能 传感器的POSE和6DOF IMU数据
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

    rs2::pipeline pipe;
    pipe.start(cfg);

    rs2::frameset data;

    // GPIO初始化
    int fd; //定义一个变量与串口进行链接，对其进行操作实际上就是对串口进行操作
    if(wiringPiSetup()<0) {
        return 1;
    }

    // Linux中每一个硬件设备都会有其相对应的节点文件，对相应的节点文件进行操作实际上就是在对硬件设备进行操作
    // 事先已经将GPIO串口修改为硬件串口了，故这里使用硬件串口
    //if((fd=serialOpen("/dev/ttyS0",115200))<0) { // gpio 使用mini串口
    if((fd=serialOpen("/dev/ttyAMA0",115200))<0) { // gpio 使用硬件串口, 波特率115200
        return 1;
    }
    cout<<"Transfer the data of t265 to the serial port for printing and output:"<<endl;
    //创建数组存储用于各个数据的名称
    const char* data_name[]={ 
        "\npx: ", "\tpy: ", "\tpz: ",
        "\tvx: ", "\tvy: ", "\tvz: ",
        "\nax: ", "\tay: ", "\taz: ",
        "\tgx: ", "\tgy: ", "\tgz: "        
    };
    while(true){
        data = pipe.wait_for_frames();
        // Get a frame from the pose stream
        auto f = data.first_or_default(RS2_STREAM_POSE);
        auto pose = f.as<rs2::pose_frame>().get_pose_data();
        
        //创建数组存储位置、速度、加速度、角速度数据
        float imu_data[ARR_N]={
            pose.translation.x, pose.translation.y, pose.translation.z, //位置
            pose.velocity.x, pose.velocity.y, pose.velocity.z, //速度
            pose.acceleration.x, pose.acceleration.y, pose.acceleration.z, //加速度 
            pose.angular_velocity.x, pose.angular_velocity.y, pose.angular_velocity.z //角速度
        };

        int* data_int;
        data_int = data_conversion(imu_data, ARR_N);
        for(int i = 0; i < ARR_N; i++){
            if(i<5||i>=ARR_N-2)
                serialPrintf(fd, "%02x ", data_int[i]); // 使用%#02x则会输出带0x表示的十六进制数，并且高位的空位会用0补全
            else 
                serialPrintf(fd, "%02x %02x ", (data_int[i]>>8)&0xff, data_int[i]&0xff);//分别输出数据高八位和低八位
        }
        free(data_int); //释放动态分配的内存空间
        
        //打印数据
        int mode = 0;//0：打印原始数据；1：打印化为整数cm单位后的数据；2：打印化为整数cm单位后的16进制数据
        data_print(data_name, imu_data, mode);
        
        // 电脑端调试调用摄像头数据并使用cv库显示
        #if ENABLE_VISION
            rs2::frame image_left = data.get_fisheye_frame(1);
            rs2::frame image_right = data.get_fisheye_frame(2);

            if (!image_left || !image_right)
                break;

            cv::Mat cv_image_left(cv::Size(848, 800), CV_8U, (void*)image_left.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat cv_image_right(cv::Size(848, 800), CV_8U, (void*)image_right.get_data(), cv::Mat::AUTO_STEP);

            cv::imshow("left", cv_image_left);
            cv::imshow("right", cv_image_right);
            cv::waitKey(1);
        #endif
    }

    return 0;
}

//定义函数将数组数据转为int类型存储
int* data_conversion(float* num, int n){
    /*
        对数组数据进行格式转换:
            1. 将数组中的数据类型由float转为int;
            2. 将数据的单位由m转为cm
            3. 对负数数据进行判断，如果是负数则加上0xff转换为正数
    */    
    int* arr_int = (int*)malloc(sizeof(int)*n); //动态分配内存空间
    for(int i = 0; i < n; i++){
        int tmux = (int)(num[i]*100); //t265的默认距离单位是m，这里乘以一百，将单位化为cm
        arr_int[i] = (tmux<0)?tmux+0xffff:tmux; // 如果小于0则加上0xffff，大于0则不进行操作
    }

    /*
        在数组中插入针头针尾
            在转换好后的十六进制数据数组中插入针头针尾组成特定数据传输格式
            数据传输格式定义：
            <0xaa 0x29 0x05 0xff 0x06 高八位 低八位 高八位 低八位 ____ ____ ...... 0x00>
    */
    int insertion[7] = {0xaa, 0x29, 0x05, 0xff, 0x06, 0x00};//定义针头和针尾的内容，前五个为针头，最后一个为针尾
    int pos[7] = {0, 1, 2, 3, 4, n-1};// 定义针头针尾要插入的位置

    for(int i = 0; i < n; i++){
        for(int j = 0; j < 6; j++)//只有6个数据需要插入
            if(pos[j] == i){
                for(int k = n - 1; k > i; k--){
                    arr_int[k] = arr_int[k - 1];
                }
                arr_int[i] = insertion[j];
            }
    }
    return arr_int;
}

void data_print(const char* data_name[12], float* imu_data, int mode){
    cout.precision(4);//设置cout默认打印4位有效数字
    int i = 0;
    switch (mode){
        case 0: for(i=0;i<12;i++)cout<<data_name[i]<<imu_data[i];break;// 打印原始数据
        case 1: for(i=0;i<12;i++)cout<<data_name[i]<<(int)(imu_data[i]*100);break;// 打印化为整数cm单位后的数据
        case 2: for(i=0;i<12;i++)cout<<data_name[i]<<hex<<(int)(imu_data[i]*100);break;// 打印化为整数cm单位后的16进制数据
        default: for(i=0;i<12;i++)cout<<data_name[i]<<imu_data[i];break;
    }
    return ;
}