#include "ArmorDetection.hpp"
#include "LatestChannel.hpp"
#include "LocalCam.hpp"

std::shared_ptr<detector::MatChannel> g_mat_channel = std::make_shared<detector::MatChannel>();
auto chan = std::make_shared<localcam::MatChannel>();
detector::ArmorDetection armor_detector(g_mat_channel);

int main() {
    localcam::DHLocalCamConfig cfg;
    cfg.SN = "";  // 为空=默认相机；或填入具体 SN
    cfg.FPS = 120;
    cfg.frame_refresh_rate = 120;
    cfg.IS_ROTATE = false;
    cfg.DEEP_COPY_ON_PUBLISH = false;  // 如有底层缓冲复用问题可改 true

    localcam::DHLocalCam cam(chan, cfg);
    if (!cam.start()) {
        std::cerr << "Failed to start DHLocalCam\n";
        return 1;
    }
    if (!armor_detector.start()) {
        std::cout << "detector start failed" << std::endl;
    }

    return 0;
}
