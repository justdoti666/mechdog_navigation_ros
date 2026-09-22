
// mount_calib.cpp — 安装角三参数离线标定 (2026-09-21)
// 目标: 求 (pitch, roll, yaw) 使**多帧真实地板**拟合平面的倾角最小
//   · 多帧联合: 同一组外参必须同时把每帧地板都摆平 ⇒ 排除"用非地面面凑低倾角"的假解
//   · 判据 = 拟合平面 tilt (acos(nz)); 全程离线, 不需要相机
// 用法: mount_calib --intr intr.txt [--step 8] [--z 0 --x 0] [--prior 0.35]
//                   [--p0 -2] [--p1 18] [--r0 -8] [--r1 8] [--y0 -12] [--y1 12]
//                   [--coarse 2] [--fine 0.25] [--cell] frames...
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>
#include "config.h"
#include "ground_segmentation.h"
#include "heightmap_2d5.h"
#include "point_cloud.h"
using namespace mechdog;

static bool load_raw(const std::string& p, int W, int H, std::vector<uint16_t>& out) {
    std::ifstream f(p, std::ios::binary); if (!f) return false;
    out.resize((size_t)W * H);
    f.read(reinterpret_cast<char*>(out.data()), (std::streamsize)(out.size() * sizeof(uint16_t)));
    return f.gcount() == (std::streamsize)(out.size() * sizeof(uint16_t));
}
static double tilt_deg(const GroundPlane& pl) {
    return std::acos(std::min(1.0, std::max(-1.0, pl.nz))) / 0.01745329251994329576;
}

int main(int argc, char** argv) {
    const int W = 640, H = 480;
    std::string intr_path;
    int step = 8;
    double ext_z = 0.0, ext_x = 0.0, prior = 0.35, prior_z = -0.9, max_tilt = 15.0;
    double p0 = -2, p1 = 18, r0 = -8, r1 = 8, y0 = -12, y1 = 12;
    double coarse = 2.0, fine = 0.25;
    bool use_cell = false;
    std::vector<std::string> frames;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto nx = [&](double d) { return (i + 1 < argc) ? atof(argv[++i]) : d; };
        if (a == "--intr") intr_path = (i + 1 < argc) ? argv[++i] : "";
        else if (a == "--step") step = (int)nx(step);
        else if (a == "--z") ext_z = nx(ext_z);
        else if (a == "--x") ext_x = nx(ext_x);
        else if (a == "--prior") prior = nx(prior);
        else if (a == "--prior-z") prior_z = nx(prior_z);
        else if (a == "--tilt") max_tilt = nx(max_tilt);
        else if (a == "--p0") p0 = nx(p0); else if (a == "--p1") p1 = nx(p1);
        else if (a == "--r0") r0 = nx(r0); else if (a == "--r1") r1 = nx(r1);
        else if (a == "--y0") y0 = nx(y0); else if (a == "--y1") y1 = nx(y1);
        else if (a == "--coarse") coarse = nx(coarse);
        else if (a == "--fine") fine = nx(fine);
        else if (a == "--cell") use_cell = true;
        else frames.push_back(a);
    }
    if (intr_path.empty() || frames.empty()) { printf("参数不全\n"); return 2; }
    CameraIntrinsics K; {
        std::ifstream f(intr_path); double w,h,fx,fy,cx,cy;
        if (!(f >> w >> h >> fx >> fy >> cx >> cy) || fx <= 0) { printf("内参失败\n"); return 2; }
        K.fx = fx; K.fy = fy; K.cx = cx; K.cy = cy;
    }
    // 预加载 + 一次反投影 (与 E 无关, 只做一次)
    std::vector<PointCloud> opt(frames.size());
    for (size_t i = 0; i < frames.size(); ++i) {
        std::vector<uint16_t> raw;
        if (!load_raw(frames[i], W, H, raw)) { printf("读帧失败 %s\n", frames[i].c_str()); return 2; }
        depth_to_cloud(raw.data(), W, H, K, opt[i]);
    }
    GroundSegParams gp; gp.ground_prior_z = prior_z; gp.prior_window = prior;
    gp.plane_max_tilt_deg = max_tilt; gp.use_cell_min_fit = use_cell;
    const double D2R = 0.01745329251994329576;

    auto eval = [&](double pdeg, double rdeg, double ydeg, double* mean_tilt, double* h0_out, double* inl_out) {
        CameraExtrinsics E; E.x = ext_x; E.z = ext_z;
        E.pitch = pdeg * D2R; E.roll = rdeg * D2R; E.yaw = ydeg * D2R;
        double sum = 0; int nv = 0; double h0s = 0, inls = 0;
        for (size_t fi = 0; fi < opt.size(); ++fi) {
            PointCloud ds, base; ds.seq = opt[fi].seq; ds.stamp = opt[fi].stamp; ds.frame_id = opt[fi].frame_id;
            for (size_t i = 0; i < opt[fi].points.size(); i += (size_t)step) ds.points.push_back(opt[fi].points[i]);
            transform_to_base(ds, E, base);
            GroundPlane pl; bool ok = false;
            if (use_cell) ok = fit_ground_plane_cells(base, gp, pl);
            else { GroundSegResult sg; segment_ground(base, gp, sg); pl = sg.plane; ok = sg.plane.valid; }
            if (!ok) return false;                       // 任一杯不接受 ⇒ 该候选作废 (多帧联合刚性)
            sum += tilt_deg(pl); h0s += pl.height_at_origin();
            inls += (double)pl.inliers; ++nv;
        }
        if (nv == 0) return false;
        *mean_tilt = sum / nv; *h0_out = h0s / nv; *inl_out = inls / nv;
        return true;
    };

    struct Cand { double p, r, y, t, h0, inl; };
    std::vector<Cand> all;
    auto scan = [&](double p0_, double p1_, double dp, double r0_, double r1_, double dr,
                    double y0_, double y1_, double dy) {
        for (double p = p0_; p <= p1_ + 1e-9; p += dp)
        for (double r = r0_; r <= r1_ + 1e-9; r += dr)
        for (double y = y0_; y <= y1_ + 1e-9; y += dy) {
            double t, h0, inl;
            if (eval(p, r, y, &t, &h0, &inl)) all.push_back({p, r, y, t, h0, inl});
        }
    };
    printf("帧数=%zu step=%d prior=%.2f@%.2f cell=%d\n", frames.size(), step, prior, prior_z, (int)use_cell);
    scan(p0, p1, coarse, r0, r1, coarse, y0, y1, coarse);
    printf("粗扫完成: %zu 个有效候选\n", all.size());
    if (all.empty()) { printf("无有效候选 —— 放宽 --prior 或扩大范围\n"); return 1; }
    auto best_of = [](std::vector<Cand>& v) {
        return *std::min_element(v.begin(), v.end(), [](const Cand& a, const Cand& b) { return a.t < b.t; });
    };
    Cand b1 = best_of(all);
    printf("粗扫最优: pitch=%.2f roll=%.2f yaw=%.2f  tilt=%.3f  h0=%.3f  inl=%.0f\n", b1.p, b1.r, b1.y, b1.t, b1.h0, b1.inl);
    std::vector<Cand> fine_list;
    for (double p = b1.p - 1.5; p <= b1.p + 1.5 + 1e-9; p += fine)
    for (double r = b1.r - 1.5; r <= b1.r + 1.5 + 1e-9; r += fine)
    for (double y = b1.y - 1.5; y <= b1.y + 1.5 + 1e-9; y += fine) {
        double t, h0, inl;
        if (eval(p, r, y, &t, &h0, &inl)) fine_list.push_back({p, r, y, t, h0, inl});
    }
    if (!fine_list.empty()) {
        Cand bf = best_of(fine_list);
        printf("精扫最优: pitch=%.3f roll=%.3f yaw=%.3f  tilt=%.3f  h0=%.3f  inl=%.0f\n", bf.p, bf.r, bf.y, bf.t, bf.h0, bf.inl);
        b1 = bf;
    }
    // top-8 (粗+精合并)
    all.insert(all.end(), fine_list.begin(), fine_list.end());
    std::sort(all.begin(), all.end(), [](const Cand& a, const Cand& b) { return a.t < b.t; });
    printf("--- 最平的 8 个候选 (排除假解请看 h0 是否与实测镜头高一致) ---\n");
    for (size_t i = 0; i < all.size() && i < 8; ++i)
        printf("  #%zu pitch=%+.2f roll=%+.2f yaw=%+.2f  tilt=%.3f  h0=%.3f  inl=%.0f\n",
               i + 1, all[i].p, all[i].r, all[i].y, all[i].t, all[i].h0, all[i].inl);
    printf("=== 建议参数 (度): pitch=%.2f roll=%.2f yaw=%.2f ===\n", b1.p, b1.r, b1.y);
    return 0;
}
