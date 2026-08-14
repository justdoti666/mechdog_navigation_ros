/**
 * 底盘通信抽象层 (ChassisBridge)
 *
 * 职责: 把速度指令 (linear, angular) 发送到机械狗底盘。
 * 设计: 可插拔接口 —— 换底盘通信方式只需换一个实现类, 节点代码零改动。
 *
 * 实现:
 *   - SimulatedChassisBridge (默认): 打印日志, 无硬件依赖, PC 模拟跑通链路
 *   - Stm32ChassisBridge: 通过串口 (UART) 发送 21 字节帧到 STM32H723 达妙板,
 *       协议与师兄 quadruped_ws wheel_board_bridge_node.py 一致:
 *         AA 55 | 0x01(设四轮RPM) | 0x10(16字节payload) |
 *         FL_rpm f32LE | FR_rpm | RR_rpm | RL_rpm | Checksum(前20字节和&0xFF)
 *       差速运动学: left=vx−wz·base/2, right=vx+wz·base/2, rpm=v/(2πr)·60
 *       轮径 0.0645m, 轮距 0.256m
 *
 * 使用 (chassis_bridge_node):
 *   ros2 run mechdog_navigation_ros chassis_bridge_node --ros-args -p bridge_type:=simulated
 *   # 真机: bridge_type:=stm32  (serial_port / baudrate 参数可配)
 */
#pragma once

// MSVC 默认不定义 M_PI (需 _USE_MATH_DEFINES), 自定义兜底保证跨平台编译 (nit)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>

#ifdef _WIN32
#include <windows.h>
#else
#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace mechdog_ros {

/**
 * 底盘通信抽象接口
 * 所有底盘实现只需实现 send_velocity, 接收线速度(m/s)与角速度(rad/s)。
 */
class ChassisBridge {
public:
    virtual ~ChassisBridge() = default;

    /** 发送速度指令到底盘 */
    virtual void send_velocity(double linear, double angular) = 0;

    /** 按名称创建桥接实现 (simulated / stm32) */
    static std::unique_ptr<ChassisBridge> create(const std::string& type);
};

/**
 * 模拟实现 (默认): 打印日志, 验证链路用, 无硬件依赖。
 */
class SimulatedChassisBridge : public ChassisBridge {
public:
    void send_velocity(double linear, double angular) override {
        std::cout << "[ChassisBridge:simulated] vx=" << linear
                  << " m/s, wz=" << angular << " rad/s" << std::endl;
    }
};

/**
 * STM32 真机实现 (串口, 21 字节帧)
 *
 * 协议 (与师兄 quadruped_ws 一致, 见 docs/10_下位机通信协议设计.md + 代码):
 *   AA 55 | 0x01 | 0x10 | FL_rpm LE f32 | FR_rpm | RR_rpm | RL_rpm | CS
 *   CS = (字节0..19 累加和) & 0xFF
 * 差速运动学: 轮径 0.0645m / 轮距 0.256m, 与师兄 wheel_board_bridge_node 一致。
 *
 * 注意: 若师兄栈已运行 wheel_board_bridge_node (它直接管串口),
 *       则本桥应禁用 (bridge_type:=simulated), 避免双写串口。
 */
class Stm32ChassisBridge : public ChassisBridge {
public:
    explicit Stm32ChassisBridge(const std::string& port = "/dev/ttyACM0",
                                int baudrate = 115200,
                                double wheel_radius_m = 0.0645,
                                double wheel_base_m = 0.256)
        : port_(port), baudrate_(baudrate),
          wheel_radius_m_(wheel_radius_m), wheel_base_m_(wheel_base_m) {
        open_serial();
    }

    ~Stm32ChassisBridge() override {
        if (fd_ >= 0) {
#ifdef _WIN32
            CloseHandle(reinterpret_cast<HANDLE>(fd_));
#else
            ::close(fd_);
#endif
            fd_ = -1;
        }
    }

    void send_velocity(double linear, double angular) override {
        // 限幅 (与师兄 wheel_board_bridge_node 层2一致; 与 config.h PlannerConfig 上限对齐, FIX-8)
        if (linear > 0.20) linear = 0.20;
        if (linear < -0.20) linear = -0.20;
        if (angular > 0.60) angular = 0.60;
        if (angular < -0.60) angular = -0.60;

        // 差速运动学 → 各轮 m/s → RPM
        double left_m_s  = linear - angular * wheel_base_m_ / 2.0;
        double right_m_s = linear + angular * wheel_base_m_ / 2.0;
        double left_rpm  = mps_to_rpm(left_m_s);
        double right_rpm = mps_to_rpm(right_m_s);

        // 4×float32 LE: FL, FR, RR, RL (两侧同速)
        uint8_t frame[21];
        frame[0] = 0xAA;
        frame[1] = 0x55;
        frame[2] = 0x01;   // 命令: 设四轮 RPM
        frame[3] = 0x10;   // payload 长度 16
        encode_f32_le(frame + 4,  (float)left_rpm);   // FL
        encode_f32_le(frame + 8,  (float)right_rpm);  // FR
        encode_f32_le(frame + 12, (float)right_rpm);  // RR
        encode_f32_le(frame + 16, (float)left_rpm);   // RL
        uint8_t cs = 0;
        for (int i = 0; i < 20; ++i) cs = (uint8_t)(cs + frame[i]);
        frame[20] = cs;

        if (fd_ >= 0) {
#ifdef _WIN32
            DWORD written = 0;
            WriteFile(reinterpret_cast<HANDLE>(fd_), frame, sizeof(frame), &written, nullptr);
#else
            ::write(fd_, frame, sizeof(frame));
#endif
            std::cout << "[ChassisBridge:stm32] vx=" << linear
                      << " wz=" << angular
                      << " RPM(L=" << left_rpm << ",R=" << right_rpm << ")"
                      << " frame=" << (int)frame[2] << "/" << (int)frame[20]
                      << std::endl;
        } else {
            std::cerr << "[ChassisBridge:stm32] 串口未打开, 丢弃指令" << std::endl;
        }
    }

private:
    double mps_to_rpm(double v) const {
        if (wheel_radius_m_ <= 0.0) return 0.0;
        return v / (2.0 * M_PI * wheel_radius_m_) * 60.0;
    }

    static void encode_f32_le(uint8_t* dst, float val) {
        uint32_t bits;
        std::memcpy(&bits, &val, sizeof(bits));
        dst[0] = (uint8_t)(bits & 0xFF);
        dst[1] = (uint8_t)((bits >> 8) & 0xFF);
        dst[2] = (uint8_t)((bits >> 16) & 0xFF);
        dst[3] = (uint8_t)((bits >> 24) & 0xFF);
    }

    void open_serial() {
#ifdef _WIN32
        // Windows: COMx 串口 (测试用)
        std::string win_port = port_;
        if (win_port.rfind("/dev/tty", 0) == 0) win_port = "COM3";  // 默认映射
        HANDLE h = CreateFileA(
            win_port.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
            OPEN_EXISTING, 0, nullptr);
        if (h == INVALID_HANDLE_VALUE) {
            std::cerr << "[ChassisBridge:stm32] 无法打开串口 " << win_port << std::endl;
            fd_ = -1;
            return;
        }
        // 配置 115200 8N1 无流控 (与 21 字节帧协议一致, FIX-7/ROS-2)
        DCB dcb{};
        dcb.DCBlength = sizeof(DCB);
        if (GetCommState(h, &dcb)) {
            dcb.BaudRate = static_cast<DWORD>(baudrate_);
            dcb.ByteSize = 8;
            dcb.Parity = NOPARITY;
            dcb.StopBits = ONESTOPBIT;
            dcb.fParity = FALSE;
            dcb.fOutxCtsFlow = FALSE;
            dcb.fOutxDsrFlow = FALSE;
            dcb.fDtrControl = DTR_CONTROL_DISABLE;
            dcb.fDsrSensitivity = FALSE;
            dcb.fOutX = FALSE;
            dcb.fInX = FALSE;
            dcb.fRtsControl = RTS_CONTROL_DISABLE;
            if (!SetCommState(h, &dcb))
                std::cerr << "[ChassisBridge:stm32] SetCommState 失败 (baudrate="
                          << baudrate_ << ")" << std::endl;
        }
        // 写操作不无限阻塞
        COMMTIMEOUTS timeouts{};
        timeouts.WriteTotalTimeoutConstant = 100;
        timeouts.WriteTotalTimeoutMultiplier = 10;
        SetCommTimeouts(h, &timeouts);
        fd_ = reinterpret_cast<intptr_t>(h);
        std::cout << "[ChassisBridge:stm32] 串口 " << win_port << " 已打开 @ "
                  << baudrate_ << " 8N1" << std::endl;
#else
        fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            std::cerr << "[ChassisBridge:stm32] 无法打开串口 " << port_
                      << " (errno=" << errno << ")" << std::endl;
            return;
        }
        // 配置 115200 8N1 无流控, 关 canonical/echo (FIX-7/ROS-2)
        struct termios tio;
        if (tcgetattr(fd_, &tio) == 0) {
            speed_t speed = B115200;  // 默认 115200; 常用备选档位
            if (baudrate_ == 57600)      speed = B57600;
            else if (baudrate_ == 38400) speed = B38400;
            else if (baudrate_ == 19200) speed = B19200;
            else if (baudrate_ == 9600)  speed = B9600;
            cfsetispeed(&tio, speed);
            cfsetospeed(&tio, speed);
            tio.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
            tio.c_cflag |= (CS8 | CLOCAL | CREAD);
            tio.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
            tio.c_lflag &= ~(ICANON | ECHO | ECHONL | ISIG);
            tio.c_oflag &= ~OPOST;
            tcsetattr(fd_, TCSANOW, &tio);
        }
        std::cout << "[ChassisBridge:stm32] 串口 " << port_ << " 已打开 @ "
                  << baudrate_ << " 8N1" << std::endl;
#endif
    }

    std::string port_;
    int baudrate_;
    double wheel_radius_m_;
    double wheel_base_m_;
    intptr_t fd_ = -1;
};

inline std::unique_ptr<ChassisBridge> ChassisBridge::create(const std::string& type) {
    if (type == "stm32") {
        return std::make_unique<Stm32ChassisBridge>();
    }
    // 默认 simulated
    return std::make_unique<SimulatedChassisBridge>();
}

} // namespace mechdog_ros
