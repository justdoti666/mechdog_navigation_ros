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

#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

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

// ============ 极简 HTTP MJPEG 服务器 ============
static std::atomic<bool> g_running{true};

// 发送完整 HTTP 响应
static void send_all(SOCKET s, const char* data, int len) {
    int sent = 0;
    while (sent < len) {
        int n = send(s, data + sent, len - sent, 0);
        if (n <= 0) break;
        sent += n;
    }
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
    send_all(client, header, (int)strlen(header));

    auto colorStream = reader.stream<astra::ColorStream>();
    const int width = 640, height = 480;
    std::vector<uint8_t> rgb(width * height * 3);

    while (g_running) {
        astra_update();
        astra::Frame frame = reader.get_latest_frame();
        astra::ColorFrame color = frame.get<astra::ColorFrame>();
        if (color.is_valid() && color.width() > 0) {
            // RGB888 -> 直接拷到 rgb 缓冲 (RgbPixel 是 {r,g,b} 紧凑排列)
            int len = color.width() * color.height();
            if (len == width * height) {
                color.copy_to(reinterpret_cast<astra::RgbPixel*>(rgb.data()));
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
                continue;
            }

            JpegBuffer jpeg;
            if (encode_jpeg(rgb.data(), width, height, jpeg)) {
                // 组装 MJPEG 帧
                std::string boundary =
                    "--frame\r\nContent-Type: image/jpeg\r\n"
                    "Content-Length: " + std::to_string(jpeg.size) + "\r\n\r\n";
                send_all(client, boundary.c_str(), (int)boundary.size());
                send_all(client, (const char*)jpeg.data, jpeg.size);
                send_all(client, "\r\n", 2);
                free(jpeg.data);
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
#endif
    int port = 8080;
    if (argc > 1) port = atoi(argv[1]);

    // 初始化 Astra
    astra::initialize();
    astra::StreamSet streamSet;
    astra::StreamReader reader = streamSet.create_reader();
    auto colorStream = reader.stream<astra::ColorStream>();
    colorStream.start();
    std::cout << "[RGB] Astra 彩色流已启动, 端口 " << port << std::endl;

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

    while (g_running) {
        SOCKET client = accept(server, nullptr, nullptr);
        if (client != INVALID_SOCKET) {
            // 每个连接一个线程 (简单处理)
            std::thread t(handle_client, client, std::ref(reader));
            t.detach();
        }
    }

    closesocket(server);
    WSACleanup();
    astra::terminate();
    return 0;
}
