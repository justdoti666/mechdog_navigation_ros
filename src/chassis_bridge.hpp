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
#include <thread>
#include <chrono>

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

    // v2.9.21 (T-B4, 复审批 B12): 链路故障语义 —— 连续写失败(或串口未打开) ⇒ true。
    //   节点侧可据此上报; Stm32 实现同时会尝试一帧零速兜底 (见下)。Simulated 恒 false。
    virtual bool fault() const { return false; }
    // ROS-8 (v2.2): 删除未用的 create() 工厂方法 —— chassis_bridge_node 直接构造具体实现
    // (create 不传 port/baud, 且全库无调用方); 如需工厂可在节点层包一层带参构造
};

/**
 * 模拟实现 (默认): 打印日志, 验证链路用, 无硬件依赖。
 */
class SimulatedChassisBridge : public ChassisBridge {
public:
    void send_velocity(double linear, double angular) override {
        // ROS-9 (v2.2): 每帧 cout 刷屏 -> 每 25 帧 (约 5s @5Hz) 打一次
        if (++tick_ % 25 == 0) {
            std::cout << "[ChassisBridge:simulated] vx=" << linear
                      << " m/s, wz=" << angular << " rad/s" << std::endl;
        }
    }
private:
    unsigned int tick_ = 0;
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
        // 注意 (M5): 正常链路中闸门层1 (±0.08/±0.25) 先限幅, 本桥限幅 (±0.20/±0.60)
        // 在直连模式 (cmd_vel_topic:=/cmd_vel 绕过闸门) 下才实际生效 —— 排查限幅问题时
        // 先确认当前链路是哪一层在限。
        if (linear > 0.20) linear = 0.20;
        if (linear < -0.20) linear = -0.20;
        if (angular > 0.60) angular = 0.60;
        if (angular < -0.60) angular = -0.60;

        // 差速运动学 → 各轮 m/s → RPM
        double left_m_s  = linear - angular * wheel_base_m_ / 2.0;
        double right_m_s = linear + angular * wheel_base_m_ / 2.0;
        double left_rpm  = mps_to_rpm(left_m_s);
        double right_rpm = mps_to_rpm(right_m_s);

        // 4×float32 LE: FL, FR, RR, RL (两侧同速); T-B4: 组帧抽成 build_rpm_frame 供零速兜底复用
        uint8_t frame[21];
        build_rpm_frame(frame, left_rpm, right_rpm);

        if (fd_ >= 0) {
            // H4 修复: 串口短写/失败必须重试, 否则 STM32 收到半个帧。
            // 旧实现单次 write/WriteFile, 短写 (或 O_NONBLOCK 下 EAGAIN/TX 满) 仅打日志。
            if (!write_frame_all(frame, sizeof(frame))) {
                ++fail_streak_;
                // v2.9.21 (T-B4, 复审批 B12): 连续 kFaultStreak 帧写失败 ⇒ 置故障 +
                //   best-effort 发一帧**零速** (半帧/缓存残留场景让底盘尽快停下)。
                //   故障后**不停止发布**: 每次调用仍继续尝试 (链路恢复自动解除)。
                if (fail_streak_ >= kFaultStreak && !fault_) {
                    fault_ = true;
                    uint8_t zframe[21];
                    build_rpm_frame(zframe, 0.0, 0.0);
                    if (write_frame_all(zframe, sizeof(zframe))) ++zero_sent_;
                    std::cerr << "[ChassisBridge:stm32] 连续 " << fail_streak_
                              << " 帧写失败 ⇒ 链路故障: 已尝试零速兜底帧, "
                              << "请检查串口接线/供电 (期间指令不保证到达底盘)" << std::endl;
                } else if (!fault_) {
                    std::cerr << "[ChassisBridge:stm32] 串口写失败/短写 (重试后仍失败): "
                              << sizeof(frame) << " 字节 (连续 " << fail_streak_ << ")" << std::endl;
                }
            } else {
                if (fault_) {
                    std::cerr << "[ChassisBridge:stm32] 串口写恢复 ⇒ 故障解除" << std::endl;
                }
                fail_streak_ = 0;
                fault_ = false;
                // ROS-9 (v2.2): 成功路径每帧 cout 刷屏 -> 每 25 帧 (约 5s @5Hz) 打一次
                if (++ok_tick_ % 25 == 0) {
                    std::cout << "[ChassisBridge:stm32] vx=" << linear
                              << " wz=" << angular
                              << " RPM(L=" << left_rpm << ",R=" << right_rpm << ")"
                              << " frame=" << (int)frame[2] << "/" << (int)frame[20]
                              << std::endl;
                }
            }
        } else {
            // Low 修复: 串口未打开时避免每帧 (5Hz) cerr 刷屏 -- 每 25 帧打一次 (约 5s)
            // v2.9.21 (T-B4): 未打开同样计入故障链 (连续 kFaultStreak 条未送达 ⇒ fault);
            //   此时无端口可发零速兜底, 只能置位 + 日志。
            ++not_open_count_;
            ++fail_streak_;
            if (fail_streak_ >= kFaultStreak && !fault_) {
                fault_ = true;
                std::cerr << "[ChassisBridge:stm32] 连续 " << fail_streak_
                          << " 条指令因串口未打开被丢弃 ⇒ 链路故障: 指令未到达底盘, "
                          << "且无法发零速兜底(无端口) —— 请检查串口设备" << std::endl;
            }
            if (not_open_tick_ >= 25) {
                not_open_tick_ = 0;
                std::cerr << "[ChassisBridge:stm32] 串口未打开, 丢弃指令"
                          << " (已连续丢弃 " << not_open_count_ << " 条)" << std::endl;
            }
            ++not_open_tick_;
        }
    }

    // v2.9.21 (T-B4): 链路故障标志 (连续写失败/串口未打开 ⇒ true; 写恢复自动解除)
    bool fault() const override { return fault_; }
    // T-B4: 测试观测 —— 已成功发出的零速兜底帧数
    unsigned long zero_frame_sent() const { return zero_sent_; }

private:
    // T-B4: 21 字节帧组装 (AA 55 | 0x01 | 0x10 | 4×f32 LE | 累加和); 零速兜底同用此函数。
    // 注: 轮序 FL=left/FR=right/RR=right/RL=left 为差速假定, 待 STM32 实机联调验证。
    static void build_rpm_frame(uint8_t frame[21], double left_rpm, double right_rpm) {
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
    }

    // H4 修复: 全量写入 + 重试。串口短写 (O_NONBLOCK 下 EAGAIN/TX 满, Windows 下
    // 缓冲满) 会导致 STM32 收到半个帧。循环补齐未写字节, 有限重试 3 轮。
    bool write_frame_all(const uint8_t* data, size_t len) {
        const int kMaxRetries = 3;
        size_t sent = 0;
        for (int attempt = 0; attempt < kMaxRetries && sent < len; ++attempt) {
#ifdef _WIN32
            DWORD written = 0;
            BOOL ok = WriteFile(reinterpret_cast<HANDLE>(fd_), data + sent,
                                (DWORD)(len - sent), &written, nullptr);
            if (!ok) return false;
            sent += written;
#else
            ssize_t n = ::write(fd_, data + sent, len - sent);
            if (n < 0) {
                // O_NONBLOCK 下 TX 满 -> EAGAIN, 稍等重试; 其他错误直接放弃
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                    continue;
                }
                return false;
            }
            sent += (size_t)n;
#endif
            if (sent < len) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }
        return sent == len;
    }

    // Low: 串口未打开时的节流刷屏计数
    unsigned int not_open_tick_ = 0;
    unsigned long not_open_count_ = 0;
    unsigned int ok_tick_ = 0;  // ROS-9: 成功路径节流计数
    // v2.9.21 (T-B4): 链路故障语义 —— 连续失败计数 / 故障位 / 零速兜底成功数
    static constexpr unsigned int kFaultStreak = 3;
    unsigned int fail_streak_ = 0;
    bool fault_ = false;
    unsigned long zero_sent_ = 0;

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
            // M1 修复: 波特率白名单外不再静默降级 B115200 —— 否则联调时以为用了目标
            // 波特率实际是 115200, 帧时序完全错乱。非法波特率直接报错退出。
            speed_t speed;
            switch (baudrate_) {
                case 115200: speed = B115200; break;
                case 57600:  speed = B57600;  break;
                case 38400:  speed = B38400;  break;
                case 19200:  speed = B19200;  break;
                case 9600:   speed = B9600;   break;
                default:
                    std::cerr << "[ChassisBridge:stm32] 非法波特率 " << baudrate_
                              << " (支持 115200/57600/38400/19200/9600)" << std::endl;
                    ::close(fd_); fd_ = -1; return;
            }
            // M1: 检查 cfsetispeed/cfsetospeed/tcsetattr 返回值 —— 失败仍以残旧参数运行为静默故障
            if (cfsetispeed(&tio, speed) != 0 || cfsetospeed(&tio, speed) != 0) {
                std::cerr << "[ChassisBridge:stm32] cfset speed 失败 (baudrate="
                          << baudrate_ << ")" << std::endl;
            }
            tio.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
            tio.c_cflag |= (CS8 | CLOCAL | CREAD);
            tio.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
            tio.c_lflag &= ~(ICANON | ECHO | ECHONL | ISIG);
            tio.c_oflag &= ~OPOST;
            if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
                std::cerr << "[ChassisBridge:stm32] tcsetattr 失败 (errno="
                          << errno << ")" << std::endl;
            }
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

// ROS-8 (v2.2): ChassisBridge::create() 已删除 (未用, 且不传 port/baud);
// 节点 (chassis_bridge_node.cpp) 直接按 bridge_type 参数构造具体实现

} // namespace mechdog_ros
