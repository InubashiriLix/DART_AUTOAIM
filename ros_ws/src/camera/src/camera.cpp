//Include CAMERA DRIVER
#include <xinerCAM.h>
#include <CapturePub.h>

//Include ROS DRIVER
#include <xinerROS.h>

//Change to Ros
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

using namespace cv;
using namespace dnn;
using namespace std;
using namespace chrono_literals;

//Ros
sensor_msgs::msg::Image rosImage;
//相机参数
sensor_msgs::msg::CameraInfo cam_p;
// 控制图像捕获线程的运行
mutex frame_mutex;
queue<cv::Mat> frame_queue;
bool g_bAcquisitionFlag = true; 
size_t closest_index = 999;

//原始相机帧率计算器
std::chrono::steady_clock::time_point lastTime1 = std::chrono::steady_clock::now();
int frameCount1 = 0;

//动态模型的优越性
int img_width = 640;
int img_height = 512;

Camera* Camera::instance = nullptr;

Camera::Camera() {
    instance = this;
}

Camera::~Camera() {
    close();
}

bool Camera::open() {
    GX_STATUS status = GX_STATUS_SUCCESS;
    uint32_t deviceNumber = 0;
    //重要，初始化相机，否则无法调用。
    status = GXInitLib();
    status = GXUpdateDeviceList(&deviceNumber, 1000);
    if (deviceNumber <= 0) {
        printf("找不到你这该死相机。\n");
        return false;
    }

    if (status != GX_STATUS_SUCCESS) {
        std::cerr << "垃圾相机列表更新失败。\n" << status << std::endl;
        return false;
    }
    if (deviceNumber == 0) {
        std::cerr << "沙壁，没接相机！\n" << std::endl;
        return false;
    }

    GX_OPEN_PARAM openParam;
    openParam.accessMode = GX_ACCESS_EXCLUSIVE;
    openParam.openMode = GX_OPEN_INDEX;
    openParam.pszContent = "1";
    status = GXOpenDevice(&openParam, &hDevice);
    if (status != GX_STATUS_SUCCESS) {
        std::cerr << "打不开你这破相机，被占用了！\n " << status << std::endl;
        return false;
    }
    //status = GXImportConfigFile(hDevice, "../config/config.txt");
    status = GXSetBool(hDevice, GX_BOOL_REVERSE_X, true);
    status = GXSetBool(hDevice, GX_BOOL_REVERSE_Y, true);
    return true;
}

bool Camera::close() {
    if (hDevice != nullptr) {
        GXCloseDevice(hDevice);
        hDevice = nullptr;
    }
    return true;
}

bool Camera::startCapture() {
    if (hDevice != nullptr) {
        GX_STATUS status = GXRegisterCaptureCallback(hDevice, nullptr, OnFrameCallbackFun);
        if (status != GX_STATUS_SUCCESS) {
            return false;
        }

        status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);
        return status == GX_STATUS_SUCCESS;
    }
    return false;
}

bool Camera::stopCapture() {
    if (hDevice != nullptr) {
        GX_STATUS status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_STOP);
        GXUnregisterCaptureCallback(hDevice);
        return status == GX_STATUS_SUCCESS;
    }
    return false;
}

void ShowImg() {
    while (g_bAcquisitionFlag) {
        cv::Mat frame;
        // 从相机或队列中获取图像帧
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            if (!frame_queue.empty()) {
                frame = frame_queue.front();
                frame_queue.pop();
            }
        }
        if (!frame.empty()) {
            cv::imshow("Detection", frame);
            cv::waitKey(1);
        }
    }
}

void GX_STDC Camera::OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame) {
    if (pFrame->status == GX_FRAME_STATUS_SUCCESS) {
        // 创建一个OpenCV的Mat，用于存放从相机收到的原始BayerRG格式图像
        void* nonConstBuffer = const_cast<void*>(pFrame->pImgBuf);
        cv::Mat rawImage(pFrame->nHeight, pFrame->nWidth, CV_8UC1, nonConstBuffer);

        cv::Mat bgrImage;
        // 使用cv::cvtColor将BayerRG格式图像转换为BGR格式
        cv::cvtColor(rawImage, bgrImage, cv::COLOR_BayerRG2BGR);

        // 设置目标图像大小
        cv::Size targetSize(img_width, img_height);
        cv::Mat resizedImage;

        // 缩放图像到目标大小
        cv::resize(bgrImage, resizedImage, targetSize);
        rosImage = *(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", resizedImage).toImageMsg());
/*
        frameCount1++; // 增加原始相机帧数计数器
        auto now1 = std::chrono::steady_clock::now(); // 获取当前时间
        double elapsedSeconds = std::chrono::duration_cast<std::chrono::duration<double>>(now1 - lastTime1).count();

        if (elapsedSeconds >= 1.0) { // 如果自上次更新以来已经过了至少一秒
            double fps1 = frameCount1 / elapsedSeconds; // 计算FPS
            std::cout << "原始相机取流FPS: " << fps1 << std::endl;

            lastTime1 = now1; // 重置计时器
            frameCount1 = 0;} // 重置帧数计数器
*/

        // 使用锁确保线程安全地向队列添加图像
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            // 清空队列，只保留最新的一帧图像
            std::queue<cv::Mat> empty;
            std::swap(frame_queue, empty);
            frame_queue.push(resizedImage.clone());
        }
    }
}


CapturePub::CapturePub(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub1)
    : img_width(640), img_height(512), // 初始化列表应放在冒号后面
      pub_(pub),
      pub1_(pub1)
{
        while(1) {
                    cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 1419.3, 0, 611.3, 0, 1418.2, 533.1, 0, 0, 1);
                    cv::Mat distCoeffs = (cv::Mat_<double>(1,5) << -0.1295, 0.0804, 4.85E-04, 6.37E-04, 0.2375);
                    cv::Mat dst = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(img_width, img_height), 0);
                    cam_p.height = img_height;
                    cam_p.width = img_width;
                    cam_p.distortion_model = "plumb_bob";
                    cam_p.k = {
                        cameraMatrix.at<double>(0,0), cameraMatrix.at<double>(0,1), cameraMatrix.at<double>(0,2),
                        cameraMatrix.at<double>(1,0), cameraMatrix.at<double>(1,1), cameraMatrix.at<double>(1,2),
                        cameraMatrix.at<double>(2,0), cameraMatrix.at<double>(2,1), cameraMatrix.at<double>(2,2),
                    };
                    cam_p.d = {
                        distCoeffs.at<double>(0,0), distCoeffs.at<double>(0,1), distCoeffs.at<double>(0,2),
                        distCoeffs.at<double>(1,0), distCoeffs.at<double>(1,1), distCoeffs.at<double>(1,2),
                        distCoeffs.at<double>(2,0), distCoeffs.at<double>(2,1), distCoeffs.at<double>(2,2),
                    };
                    cam_p.p = {
                        dst.at<double>(0,0), dst.at<double>(0,1), dst.at<double>(0,2),
                        dst.at<double>(1,0), dst.at<double>(1,1), dst.at<double>(1,2),
                        dst.at<double>(2,0), dst.at<double>(2,1), dst.at<double>(2,2),
                    };
                    cam_p.r = {1,0,0,0,1,0,0,0,1};
                    cam_p.binning_x = 0;
                    cam_p.binning_y = 0;
                    pub_->publish(rosImage);
                    pub1_->publish(cam_p);
                } 
}



int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("capture_pub_node");
    auto pub = node->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    auto pub1 = node->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);

    system("clear");
    std::cout << "\n";
    std::cout << "---------------------------------------------------------------------------\n";
    std::cout << "---------------------------------------------------------------------------\n";
    std::cout << "\n";
    std::cout << " ooo    pppp    eeeee   n   n      V     V  iii   ssss   iii   ooo   n   n \n";
    std::cout << "o   o   p   p   e       nn  n       V   V    i   s        i   o   o  nn  n \n";
    std::cout << "o   o   pppp    eeeee   n n n        V V     i    ssss    i   o   o  n n n \n";
    std::cout << "o   o   p       e       n  nn         V      i        s   i   o   o  n  nn \n";
    std::cout << " ooo    p       eeeee   n   n         V     iii   ssss   iii   ooo   n   n \n";
    std::cout << "\n";
    std::cout << "\n";
    std::cout << "---------------------------------------------------------------------------\n";
    std::cout << "---------------------------------------------------------------------------\n";

    // 创建Camera实例并尝试打开相机
    Camera camera;
    if (!camera.open()) {
        std::cerr << "Failed to open camera.\n";
        return -1;
    }
    camera.startCapture();
    std::cout << "Camera opened successfully.\n";
    
    std::thread capture_thread([&]() { 
        CapturePub CapturePub(pub,pub1);
    });

    std::thread showimg_thread([&]() {
        ShowImg();
    });    
    std::thread subscription_thread([&]() {
        rclcpp::spin(node); // 在单独的线程中执行 ROS 节点的事件循环
    });

    std::cout << "Multi Thread Boosted.Press Enter to stop...\n";
    std::cin.get();
    return 0;
    g_bAcquisitionFlag = false;
    capture_thread.join();
    showimg_thread.join();
    subscription_thread.join();
    rclcpp::shutdown();
    std::cout << "ROS closed.\n";
    camera.close();
    std::cout << "Camera capture stopped and camera closed.\n";
    return 0;
}



//                            _ooOoo_  
//                           o8888888o  
//                           88" . "88  
//                           (| -_- |)  
//                            O\ = /O  
//                        ____/`---'\____  
//                      .   ' \\| |// `.  
//                       / \\||| : |||// \  
//                     / _||||| -:- |||||- \  
//                       | | \\\ - /// | |  
//                     | \_| ''\---/'' | |  
//                        \ .-\__ `-` ___/-. /  
//                   ___`. .' /--.--\ `. . __  
//                ."" '< `.___\_<|>_/___.' >'"".  
//               | | : `- \`.;`\ _ /`;.`/ - ` : | |  
//                 \ \ `-. \_ __\ /__ _/ .-` / /  
//         ======`-.____`-.___\_____/___.-`____.-'======  
//                            `=---='  
//  
//         .............................................  
//                  佛祖镇楼                 BUG辟易  
//                         以下是函数，请勿乱改

//人生若只如初见，
//何事秋风悲画扇。
//等闲变却故人心，
//却道故人心易变。
//骊山语罢清宵半，
//泪雨霖铃终不怨。
//何如薄幸锦衣郎，
//比翼连枝当日愿。
