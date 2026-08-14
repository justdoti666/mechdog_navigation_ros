/**
 * rgb_stream: Astra Pro RGB 回传服务 (Windows/树莓派通用, 仅平台差异在编译)
 *
 * 功能: 读取 Astra Pro ColorStream (RGB888) → 编码 JPEG → HTTP MJPEG 流
 *       电脑浏览器直接打开 http://<ip>:8080/stream 即可看实时画面
 *
 * 依赖: Astra SDK (astra:: API) + stb_image_write.h (JPEG 编码, 单头文件)
 *
 * 构建 (Windows MSVC):
 *   cl /nologo /EHsc /utf-8 /std:c++20 /I<sdk>/include rgb_stream.cpp \
 *       /Fe:rgb_stream.exe /link /LIBPATH:<sdk>/lib astra_core.lib astra_core_api.lib astra.lib
 *   运行前: PATH 需含 <sdk>/bin (astra.dll)
 *
 * 运行:
 *   rgb_stream.exe              # 默认 8080 端口
 *   rgb_stream.exe 9090         # 指定端口
 *   浏览器: http://localhost:8080/stream
 */
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <vector>

#include <winsock2.h>
#include <ws2tcpip.h>
#include <gdiplus.h>
#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "gdiplus.lib")

#include <astra/astra.hpp>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// ============ JPEG 编码 (stb_image_write 回调到内存) ============
struct JpegBuffer {
    unsigned char* data = nullptr;
    int size = 0;
};

static void jpeg_write_cb(void* context, void* data, int size) {
    auto* buf = static_cast<JpegBuffer*>(context);
    auto* old = buf->data;
    buf->data = static_cast<unsigned char*>(realloc(old, buf->size + size));
    if (!buf->data) { free(old); return; }
    memcpy(buf->data + buf->size, data, size);
    buf->size += size;
}

// 把 RGB888 像素编码成 JPEG (返回 true 成功)
static bool encode_jpeg(const uint8_t* rgb, int w, int h, JpegBuffer& out) {
    out.size = 0;
    int ok = stbi_write_jpg_to_func(jpeg_write_cb, &out, w, h, 3, rgb, 80);
    return ok != 0 && out.size > 0;
}

// 计算深度图中央区域的平均距离 (mm) 和最近障碍距离 (mm)
// 中央 1/4 区域, avg<0 表示无效; min 返回该区域最近有效值
static double central_depth_mm(const int16_t* depth, int w, int h,
                               double* min_mm_out) {
    int x0 = w / 4, x1 = w * 3 / 4;
    int y0 = h / 4, y1 = h * 3 / 4;
    double sum = 0;
    int count = 0;
    double min_mm = 1e9;
    for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
            int16_t v = depth[y * w + x];
            if (v > 0 && v < 8000) {  // 有效范围 0~8m
                sum += v;
                ++count;
                if (v < min_mm) min_mm = v;
            }
        }
    }
    if (min_mm_out) *min_mm_out = (count > 0) ? min_mm : -1.0;
    return count > 0 ? sum / count : -1.0;
}

// ============ GDI+ 文字叠加 (Windows 系统字体, 清晰可靠) ============
// 在 RGB 缓冲上绘制文字: 创建 GDI+ Bitmap -> Graphics -> DrawString
static void draw_text_gdiplus(uint8_t* rgb, int w, int h,
                              const char* text, int x, int y, int font_size,
                              uint8_t cr = 255, uint8_t cg = 40, uint8_t cb = 40) {
    // 用 GDI+ 从 RGB 缓冲创建位图并画字 (每次调用较慢, 但叠加距离每帧一次可接受)
    // 注意: 需在 main 中 GdiplusStartup 初始化
    Gdiplus::Bitmap bmp(w, h, w * 3, PixelFormat24bppRGB, rgb);
    Gdiplus::Graphics g(&bmp);
    g.SetTextRenderingHint(Gdiplus::TextRenderingHintAntiAlias);
    Gdiplus::SolidBrush brush(Gdiplus::Color(255, cr, cg, cb));  // ARGB
    Gdiplus::Font font(L"Arial", (Gdiplus::REAL)font_size, Gdiplus::FontStyleBold);
    std::wstring wtext(text, text + strlen(text));
    Gdiplus::PointF origin((Gdiplus::REAL)x, (Gdiplus::REAL)y);
    g.DrawString(wtext.c_str(), (INT)wtext.size(), &font, origin, &brush);
    // Bitmap 析构时会将像素写回 rgb (因为是内存映射)
}

// ============ 极简 HTTP MJPEG 服务器 ============
static std::atomic<bool> g_running{true};

// 控制台 Ctrl+C / 关闭窗口: 置退出标志 (FIX-15/ROS-5)
static BOOL WINAPI console_ctrl_handler(DWORD ctrl_type) {
    if (ctrl_type == CTRL_C_EVENT || ctrl_type == CTRL_CLOSE_EVENT ||
        ctrl_type == CTRL_BREAK_EVENT) {
        g_running = false;
        return TRUE;
    }
    return FALSE;
}

// 全局互斥锁: 保护 StreamReader 的 open_frame/close_frame (多连接线程共享 reader)
static std::mutex g_frame_mutex;

// 读取一帧彩色+深度 (非阻塞轮询, 修复: 单次 astra_update 后新帧未必就绪)
// 返回 true 表示成功拿到帧 (color 有效); 失败返回 false
static bool read_frame(astra::StreamReader& reader,
                       std::vector<uint8_t>& rgb, int w, int h,
                       double* dist_mm_out, double* min_mm_out) {
    std::lock_guard<std::mutex> lock(g_frame_mutex);

    // 短等待重试 astra_update(), 直到 has_new_frame
    for (int attempt = 0; attempt < 5; ++attempt) {
        astra_update();
        if (reader.has_new_frame()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (!reader.has_new_frame()) return false;

    // timeout=0 非阻塞取帧 (默认 ASTRA_TIMEOUT_FOREVER 会永久阻塞)
    astra::Frame frame = reader.get_latest_frame(0);
    astra::ColorFrame color = frame.get<astra::ColorFrame>();
    if (!color.is_valid() || color.width() <= 0) return false;

    int len = color.width() * color.height();
    if (len != w * h) return false;

    color.copy_to(reinterpret_cast<astra::RgbPixel*>(rgb.data()));

    // 深度: 中央区域平均距离 + 最近障碍 (mm)
    astra::DepthFrame depth = frame.get<astra::DepthFrame>();
    if (depth.is_valid() && depth.width() == w) {
        std::vector<int16_t> depth_buf(w * h);
        depth.copy_to(depth_buf.data());
        if (dist_mm_out) *dist_mm_out = central_depth_mm(depth_buf.data(), w, h, min_mm_out);
    } else {
        if (dist_mm_out) *dist_mm_out = -1.0;
        if (min_mm_out) *min_mm_out = -1.0;
    }
    return true;
}

// 发送完整 HTTP 响应; 返回实际发送字节数 (FIX-15: 调用方据此检测客户端断开)
static int send_all(SOCKET s, const char* data, int len) {
    int sent = 0;
    while (sent < len) {
        int n = send(s, data + sent, len - sent, 0);
        if (n <= 0) break;
        sent += n;
    }
    return sent;
}

// MJPEG 流处理器: 持续发 JPEG 帧 (multipart/x-mixed-replace)
static void handle_stream(SOCKET client, astra::StreamReader& reader) {
    // 响应头
    const char* header =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n"
        "\r\n";
    if (send_all(client, header, (int)strlen(header)) != (int)strlen(header)) {
        return;  // 客户端已断开
    }

    auto colorStream = reader.stream<astra::ColorStream>();
    auto depthStream = reader.stream<astra::DepthStream>();
    const int width = 640, height = 480;
    std::vector<uint8_t> rgb(width * height * 3);

    while (g_running) {
        // 读取一帧 (非阻塞轮询 + 互斥锁保护)
        double dist_mm = -1.0, min_mm = -1.0;
        if (!read_frame(reader, rgb, width, height, &dist_mm, &min_mm)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // 深度: 中央区域平均距离 + 最近障碍 (mm), 叠加到 RGB 画面
        if (dist_mm >= 0) {
            char text[40];
            // 主: 中央平均距离 (字号 18, 红)
            snprintf(text, sizeof(text), "DIST %.2fm", dist_mm / 1000.0);
            draw_text_gdiplus(rgb.data(), width, height, text, 10, 10, 18);
            // 副: 最近障碍 (字号 13, 黄), 第二行
            if (min_mm >= 0) {
                snprintf(text, sizeof(text), "NEAR %.2fm", min_mm / 1000.0);
                draw_text_gdiplus(rgb.data(), width, height, text, 10, 34, 13, 255, 200, 0);
            }
        }

        JpegBuffer jpeg;
        if (encode_jpeg(rgb.data(), width, height, jpeg)) {
            // 组装 MJPEG 帧
            std::string boundary =
                "--frame\r\nContent-Type: image/jpeg\r\n"
                "Content-Length: " + std::to_string(jpeg.size) + "\r\n\r\n";
            int sent_boundary = send_all(client, boundary.c_str(), (int)boundary.size());
            int sent_jpeg = (sent_boundary == (int)boundary.size())
                ? send_all(client, (const char*)jpeg.data, jpeg.size) : 0;
            if (sent_jpeg == jpeg.size) {
                send_all(client, "\r\n", 2);
            }
            free(jpeg.data);
            if (sent_boundary != (int)boundary.size() || sent_jpeg != jpeg.size) {
                return;  // 客户端断开, 停止本连接
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));  // ~30fps
    }
}

// 简单 HTTP 处理: 只响应 /stream
static void handle_client(SOCKET client, astra::StreamReader& reader) {
    char buf[1024];
    int n = recv(client, buf, sizeof(buf) - 1, 0);
    if (n > 0) {
        buf[n] = 0;
        std::string req(buf);
        if (req.find("GET /stream") == 0) {
            handle_stream(client, reader);
        } else {
            const char* notfound =
                "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n";
            send_all(client, notfound, (int)strlen(notfound));
        }
    }
    closesocket(client);
}

int main(int argc, char** argv) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCtrlHandler(console_ctrl_handler, TRUE);  // FIX-15: Ctrl+C 优雅退出
#endif
    int port = 8080;
    if (argc > 1) port = atoi(argv[1]);

    // 初始化 Astra
    astra::initialize();
    astra::StreamSet streamSet;
    astra::StreamReader reader = streamSet.create_reader();
    auto colorStream = reader.stream<astra::ColorStream>();
    colorStream.start();
    auto depthStream = reader.stream<astra::DepthStream>();
    depthStream.start();
    std::cout << "[RGB] Astra 彩色流+深度流已启动, 端口 " << port << std::endl;

    // 初始化 GDI+ (文字叠加)
    Gdiplus::GdiplusStartupInput gdiplusStartupInput;
    ULONG_PTR gdiplusToken = 0;
    Gdiplus::GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, nullptr);

    // 初始化 Winsock
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
    SOCKET server = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((u_short)port);
    if (bind(server, (sockaddr*)&addr, sizeof(addr)) != 0) {
        std::cerr << "[RGB] bind 失败 (端口被占用?)" << std::endl;
        return 1;
    }
    listen(server, 5);
    std::cout << "[RGB] 就绪: 浏览器打开 http://localhost:" << port << "/stream" << std::endl;

    // FIX-15: 保存连接线程, 退出时 join (原 detach 会在 astra::terminate() 后访问已释放 reader)
    std::vector<std::thread> clients;
    while (g_running) {
        SOCKET client = accept(server, nullptr, nullptr);
        if (client != INVALID_SOCKET) {
            clients.emplace_back(handle_client, client, std::ref(reader));
        }
    }

    for (auto& t : clients) {
        if (t.joinable()) t.join();
    }
    closesocket(server);
    WSACleanup();
    astra::terminate();
    return 0;
}
