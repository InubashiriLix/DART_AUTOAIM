# 稳定完整版本 <br>
由于大恒API截取图像函数的问题，每当第一次插入相机时会出现返回错误“-11 用户写入值越届问题”，不会改，所以采用ros2 launch功能respawn,重启延迟可以在camera.launch.py里设置
