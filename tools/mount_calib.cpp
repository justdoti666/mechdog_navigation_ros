
// mount_calib.cpp — 安装角三参数离线标定 (2026-09-21 首版; 2026-10-04 双平面 + 自检)
// 目标: 求 (pitch, roll, yaw) 使**多帧真实地板**拟合平面的倾角最小
//   · 多帧联合: 同一组外参必须同时把每帧地板都摆平 ⇒ 排除"用非地面面凑低倾角"的假解
//   · 判据 = 拟合平面 tilt (acos(nz)); 全程离线, 不需要相机
// 2026-10-04 追加 (离线待办 #1: 标定器双平面 + 自造帧真值验证):
//   · --wall: 目标函数追加"竖直面(墙)项" —— 对地面以上的点做受约束 RANSAC 拟竖直面,
//     目标 = 平均地板 tilt + wall_weight × 平均墙法向水平偏离(度); 墙拟合失败 ⇒ 候选作废。
//   · --selftest: 自造帧真值验证 —— 合成"地板+墙"深度帧 → 走同一条搜索链 → 真值对照断言;
//     每个场景同时跑"地板-only"与"地板+墙"两条链并打印对比 (验收: 姿态误差 <0.5°, h0 误差 <5cm)。
//   · 可辨识性 (2026-10-04 数值实验钉死): (pitch, roll) 由地面唯一确定(与 yaw 无关);
//     墙项沿"yaw 一维族"恒定 ⇒ **不增加可辨识性**, 是第二个结构面的复核项;
//     yaw 仍不可辨识 (单水平面/含墙均观测不出; 需机械对齐或竖直面已知朝向)。
//   · 抗密度偏置 (2026-10-04, 合成真值实测): cell 拟合逐格取"最低表面", 近密远疏的点密度
//     使格最小值统计的下偏量随距离变化 ⇒ 拟合面"近低远高", 把俯仰最优点推离真值 +0.25~1.0°。
//     标定器内部先把点云按 (x,y) 格均匀抽稀 (每格 1 点) 再拟合, 消除该偏置 (节点侧不动)。
//     抽稀取样语义 (2026-10-04, 两轮真机帧 + 合成帧探针修正): 每格取**首末端点中点** ——
//     单端点取样被实测钉出边界噪声选择性偏差: 格顶/格底像素坐在格边界上, 邻域噪声把点
//     挤出/挤进格时被条件化 (ε 偏 ±0.8σ), 经 z = -H·(1+ε/s) 的 1/s 杠杆放大后, 格顶与
//     格底各自给 ~∓0.29° 的反向斜率 (近段偏差 ±7~9mm, 远段 ≈±0.8mm); 中点两点平均对消。
//     多表面堆叠格 (墙压地板等, |Δz|>0.10m) 取**较低者** = 可见最低表面。纯"保首点"会在
//     真机复杂场景取到柱顶高点, 被拟合高度带整格淘汰 (floor_1617: 带内格数 14 → 0,
//     全参数 0 候选; 保末端点可复活但带入上述 +0.25° 偏差)。
//   · 残余特征 (合成真值实测, 仍在验收 0.5° 内): 浅地板(≲1.5m)+噪声时残差 −0.25° 量级,
//     随噪声线性 (0 噪声 = 精确归零); 深地板(≳3m)与侧墙位形不出现; 墙项在浅景下可把解拉回真值。
// 用法: mount_calib --intr intr.txt [--step 8] [--z 0 --x 0] [--prior 0.35]
//                   [--p0 -2] [--p1 18] [--r0 -8] [--r1 8] [--y0 -12] [--y1 12]
//                   [--coarse 2] [--fine 0.25] [--cell]
//                   [--wall [--wall-weight 1.0] [--wall-min-pts 60] [--wall-max-dev 10] [--wall-rms 0.05]]
//                   frames...
//        mount_calib --selftest        # 自造帧真值验证 (无需帧文件; 退出码 >0 = 有断言失败)
// v2.9.18 稳健性: 数值参数缺值/非法 ⇒ 报错退出(不再静默吞下一个参数; 旧版 `--z --cell`
//   会把 `--cell` 当 0 吃掉); `--step` 必须 >=1; 载入时剔除坏帧(有效像素 <25%);
//   yaw 仅在可辨识时给建议值(单水平面观测不出, 详见结尾输出)。
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "config.h"
#include "ground_segmentation.h"
#include "heightmap_2d5.h"
#include "point_cloud.h"
using namespace mechdog;

namespace {

constexpr double kD2R = 0.01745329251994329576;

bool load_raw(const std::string& p, int W, int H, std::vector<uint16_t>& out) {
    std::ifstream f(p, std::ios::binary); if (!f) return false;
    out.resize((size_t)W * H);
    f.read(reinterpret_cast<char*>(out.data()), (std::streamsize)(out.size() * sizeof(uint16_t)));
    return f.gcount() == (std::streamsize)(out.size() * sizeof(uint16_t));
}
double tilt_deg(const GroundPlane& pl) {
    return std::acos(std::min(1.0, std::max(-1.0, pl.nz))) / kD2R;
}
// v2.9.18: 有效像素计数 (口径同 sensor_astra.h: 600~8000mm) —— 供坏帧预过滤
size_t count_valid_raw(const std::vector<uint16_t>& d) {
    size_t n = 0;
    for (uint16_t v : d) if (v >= 600 && v <= 8000) ++n;
    return n;
}

// ============================================================
// 竖直面(墙)拟合 (2026-10-04) —— 与 ground_segmentation 的受约束 RANSAC 同风格
//   输入: base 系中"地面以上"的点; 输出: 法向近水平的平面 (|nz| 小 = 墙面竖直)。
//   约束: 采样平面 |nz| <= sin(max_dev_deg); 原点到平面距离在 [min_dist, max_dist];
//   内点距离 <= inlier_dist; 内点数 >= min_inliers; 精修后 RMS <= rms_max。
// ============================================================
struct WallPlane {
    bool   valid = false;
    double nx = 0.0, ny = 0.0, nz = 0.0, d = 0.0;   // nx*x+ny*y+nz*z+d=0, |n|=1, nz>=0
    int    inliers = 0;
    double rms = 0.0;
};
struct WallParams {
    double max_dev_deg = 10.0;   // 采样约束: 法向对水平面的最大偏离角
    double inlier_dist = 0.03;   // 内点距离 (m)
    double min_dist = 0.35;      // 原点到墙距离范围 (m) —— 太近是盲区噪声, 太远超量程
    double max_dist = 6.0;
    int    max_iters = 80;
    int    min_inliers = 60;     // 抽稀后最低支撑点数
    double rms_max = 0.05;       // 精修后 RMS 守门 (与地板同口径 5cm)
    int    max_points = 800;     // 拟合前抽稀上限 (控搜索耗时)
    unsigned seed = 20260830u;   // 固定种子: 复现确定性
};

/** 平面方程过三点; 返回 false = 三点近共线 (退化采样, RANSAC 中跳过) */
bool plane_from_three(const Point3D& a, const Point3D& b, const Point3D& c,
                      double& nx, double& ny, double& nz, double& d) {
    const double ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;
    const double vx = c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;
    nx = uy * vz - uz * vy;
    ny = uz * vx - ux * vz;
    nz = ux * vy - uy * vx;
    const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len < 1e-6) return false;
    nx /= len; ny /= len; nz /= len;
    d = -(nx * a.x + ny * a.y + nz * a.z);
    return true;
}

/** 沿主轴的最小二乘平面拟合 (axis = 因变量轴; 与 ground_segmentation 的 lsq_plane_zy 同手法) */
bool lsq_plane_axis(const std::vector<Point3D>& q, int axis,
                    double& nx, double& ny, double& nz, double& d) {
    if (q.size() < 3) return false;
    double mu[3] = {0.0, 0.0, 0.0};
    for (const auto& p : q) {
        const double c[3] = {p.x, p.y, p.z};
        for (int k = 0; k < 3; ++k) mu[k] += c[k];
    }
    for (double& m : mu) m /= (double)q.size();
    const int iu = (axis + 1) % 3, iv = (axis + 2) % 3;
    double Suu = 0, Suv = 0, Svv = 0, Suw = 0, Svw = 0;
    for (const auto& p : q) {
        const double c[3] = {p.x - mu[0], p.y - mu[1], p.z - mu[2]};
        Suu += c[iu] * c[iu]; Suv += c[iu] * c[iv]; Svv += c[iv] * c[iv];
        Suw += c[iu] * c[axis]; Svw += c[iv] * c[axis];
    }
    const double det = Suu * Svv - Suv * Suv;
    if (std::abs(det) < 1e-12) return false;
    const double a = (Suw * Svv - Suv * Svw) / det;
    const double b = (Suu * Svw - Suv * Suw) / det;
    // 平面: X_axis = a·X_iu + b·X_iv + c (c 取质心 ⇒ 平面过质心); 法向 g 未归一
    double g[3] = {0.0, 0.0, 0.0};
    g[axis] = 1.0; g[iu] = -a; g[iv] = -b;
    const double gn = std::sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
    if (!(gn > 1e-12)) return false;
    nx = g[0] / gn; ny = g[1] / gn; nz = g[2] / gn;
    d = -(nx * mu[0] + ny * mu[1] + nz * mu[2]);
    if (nz < 0) { nx = -nx; ny = -ny; nz = -nz; d = -d; }
    return std::isfinite(nx) && std::isfinite(ny) && std::isfinite(nz) && std::isfinite(d);
}

bool fit_wall_plane(const std::vector<Point3D>& pts, const WallParams& wp, WallPlane& out) {
    out = WallPlane{};
    std::vector<Point3D> all;
    all.reserve(pts.size());
    for (const auto& p : pts) {
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) all.push_back(p);
    }
    if ((int)all.size() < wp.min_inliers) return false;
    // 抽稀 (上限): 等步长取样, 保序
    const int cap = (wp.max_points > 0) ? wp.max_points : 1000000;
    const int stride = std::max(1, (int)(all.size() / (size_t)cap));
    std::vector<Point3D> q;
    q.reserve(std::min<size_t>(all.size(), (size_t)cap) + 1);
    for (size_t i = 0; i < all.size(); i += (size_t)stride) q.push_back(all[i]);
    const int n = (int)q.size();
    if (n < wp.min_inliers) return false;

    const double sin_gate = std::sin(wp.max_dev_deg * kD2R);
    std::mt19937 rng(wp.seed);
    std::uniform_int_distribution<int> pick(0, n - 1);
    int best_inl = 0; double bnx = 0, bny = 0, bnz = 0, bd = 0;
    for (int it = 0; it < wp.max_iters; ++it) {
        const int i1 = pick(rng), i2 = pick(rng), i3 = pick(rng);
        if (i1 == i2 || i2 == i3 || i1 == i3) continue;
        double nx, ny, nz, d;
        if (!plane_from_three(q[i1], q[i2], q[i3], nx, ny, nz, d)) continue;
        if (nz < 0) { nx = -nx; ny = -ny; nz = -nz; d = -d; }
        if (std::abs(nz) > sin_gate) continue;                    // 竖直面约束
        const double dist = std::abs(d);
        if (dist < wp.min_dist || dist > wp.max_dist) continue;   // 距离范围
        int inl = 0;
        for (const auto& p : q) {
            if (std::abs(nx * p.x + ny * p.y + nz * p.z + d) <= wp.inlier_dist) ++inl;
        }
        if (inl > best_inl) {
            best_inl = inl; bnx = nx; bny = ny; bnz = nz; bd = d;
        }
        if (best_inl >= (int)(0.9 * (double)n)) break;            // 支撑已足, 提前收敛
    }
    if (best_inl < wp.min_inliers) return false;

    // 精修: 旧平面内点 → 沿主轴 LSQ → 全量重统计
    std::vector<Point3D> inl_pts;
    inl_pts.reserve((size_t)best_inl);
    for (const auto& p : q) {
        if (std::abs(bnx * p.x + bny * p.y + bnz * p.z + bd) <= wp.inlier_dist) inl_pts.push_back(p);
    }
    int axis = 0;
    {
        const double aa[3] = {std::abs(bnx), std::abs(bny), std::abs(bnz)};
        if (aa[1] > aa[axis]) axis = 1;
        if (aa[2] > aa[axis]) axis = 2;
    }
    double nx, ny, nz, d;
    if (!lsq_plane_axis(inl_pts, axis, nx, ny, nz, d)) return false;
    if (std::abs(nz) > sin_gate) return false;                    // 精修后仍须竖直
    int finl = 0; double ss = 0.0;
    for (const auto& p : q) {
        const double r = nx * p.x + ny * p.y + nz * p.z + d;
        if (std::abs(r) <= wp.inlier_dist) { ++finl; ss += r * r; }
    }
    if (finl < wp.min_inliers) return false;
    const double rms = std::sqrt(ss / (double)std::max(1, finl));
    if (rms > wp.rms_max) return false;
    const double dist2 = std::abs(d);
    if (dist2 < wp.min_dist || dist2 > wp.max_dist) return false;
    out.valid = true;
    out.nx = nx; out.ny = ny; out.nz = nz; out.d = d;
    out.inliers = finl; out.rms = rms;
    return true;
}

// ============================================================
// 候选评估 / 扫描 (与 v2.9.18 行为一致; 2026-10-04 抽出以便 --selftest 复用同一链)
// ============================================================
struct Cand { double p = 0, r = 0, y = 0, t = 0, wd = 0, h0 = 0, inl = 0, score = 0; };

struct CalibCfg {
    int step = 8;
    double ext_x = 0.0, ext_z = 0.0;
    GroundSegParams gp;
    bool use_cell = false;
    bool use_wall = false;
    double wall_weight = 1.0;      // 墙项权重 (度/度)
    WallParams wp;
    double p0 = -2, p1 = 18, r0 = -8, r1 = 8, y0 = -12, y1 = 12;
    double coarse = 2.0, fine = 0.25;
};

/** 单候选评估: 全帧地板摆平 + (可选)全帧墙竖直; 任一帧失败 ⇒ 该候选作废 (多帧联合刚性) */
bool eval_cand(const std::vector<PointCloud>& opt, const CalibCfg& cfg,
               double pdeg, double rdeg, double ydeg, Cand& out) {
    CameraExtrinsics E; E.x = cfg.ext_x; E.z = cfg.ext_z;
    E.pitch = pdeg * kD2R; E.roll = rdeg * kD2R; E.yaw = ydeg * kD2R;
    double sum = 0, h0s = 0, inls = 0, wsum = 0;
    int nv = 0;
    for (size_t fi = 0; fi < opt.size(); ++fi) {
        PointCloud ds;
        ds.seq = opt[fi].seq; ds.stamp = opt[fi].stamp; ds.frame_id = opt[fi].frame_id;
        ds.points.reserve(opt[fi].points.size() / (size_t)cfg.step + 1);
        for (size_t i = 0; i < opt[fi].points.size(); i += (size_t)cfg.step) ds.points.push_back(opt[fi].points[i]);
        PointCloud base;
        transform_to_base(ds, E, base);
        GroundPlane pl; bool ok = false;
        if (cfg.use_cell) {
            // 2026-10-04: 抗密度偏置均匀抽稀 + 边界噪声条件化修正 —— 见文件头说明。
            //   每 (x,y) 格取**首末端点中点** (两点平均对消格边界 ε 条件化偏差);
            //   多表面堆叠格 (|Δz|>0.10m) 取较低者 = 可见最低表面 (真机墙压地板场景必须)。
            //   标定器专用; 节点侧拟合不动。
            PointCloud uni;
            uni.seq = base.seq; uni.stamp = base.stamp; uni.frame_id = base.frame_id;
            {
                struct FL { Point3D f, l; };
                std::unordered_map<int64_t, FL> fl;
                const double cs = (cfg.gp.cell_size > 0.01) ? cfg.gp.cell_size : 0.05;
                fl.reserve(base.points.size() / 2 + 16);
                for (const auto& q : base.points) {
                    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) continue;
                    const int64_t ix = (int64_t)std::floor(q.x / cs) + 1000000;
                    const int64_t iy = (int64_t)std::floor(q.y / cs) + 1000000;
                    const int64_t k = ix * 1000003 + iy;
                    auto it = fl.find(k);
                    if (it == fl.end()) fl.emplace(k, FL{q, q});
                    else it->second.l = q;               // 保序更新 ⇒ l = 光栅序最后点
                }
                uni.points.reserve(fl.size());
                for (const auto& kv : fl) {
                    const Point3D& f = kv.second.f;
                    const Point3D& l = kv.second.l;
                    Point3D s;
                    if (std::fabs(f.z - l.z) > 0.10) {
                        s = (f.z <= l.z) ? f : l;        // 多表面堆叠 ⇒ 取较低者 (最低表面)
                    } else {
                        s.x = 0.5 * (f.x + l.x);         // 单表面 ⇒ 中点 (噪声偏差对消)
                        s.y = 0.5 * (f.y + l.y);
                        s.z = 0.5 * (f.z + l.z);
                    }
                    uni.points.push_back(s);
                }
            }
            ok = fit_ground_plane_cells(uni, cfg.gp, pl);
        }
        else { GroundSegResult sg; segment_ground(base, cfg.gp, sg); pl = sg.plane; ok = sg.plane.valid; }
        if (!ok) return false;                       // 任一杯不接受 ⇒ 该候选作废 (多帧联合刚性)
        sum += tilt_deg(pl); h0s += pl.height_at_origin();
        inls += (double)pl.inliers; ++nv;
        if (cfg.use_wall) {
            std::vector<Point3D> above;              // 地面以上点 → 墙候选
            above.reserve(base.points.size() / 4);
            for (const auto& q : base.points) {
                if (pl.nx * q.x + pl.ny * q.y + pl.nz * q.z + pl.d > 0.06) above.push_back(q);
            }
            WallPlane wl;
            if (!fit_wall_plane(above, cfg.wp, wl)) return false;
            wsum += std::asin(std::min(1.0, std::fabs(wl.nz))) / kD2R;
        }
    }
    if (nv == 0) return false;
    out.p = pdeg; out.r = rdeg; out.y = ydeg;
    out.t = sum / nv; out.h0 = h0s / nv; out.inl = inls / nv;
    out.wd = cfg.use_wall ? (wsum / nv) : 0.0;
    out.score = out.t + (cfg.use_wall ? cfg.wall_weight * out.wd : 0.0);
    return true;
}

struct SearchOut {
    bool ok = false;
    Cand coarse_best, fine_best;
    bool fine_ran = false;
    size_t n_coarse = 0, ncand = 0;
    bool yaw_ident = false;
    double yaw_span = -1.0;
    std::vector<Cand> top;    // 按 score 升序, 至多 8 个
};

SearchOut run_search(const std::vector<PointCloud>& opt, const CalibCfg& cfg) {
    SearchOut so;
    auto score_less = [](const Cand& a, const Cand& b) { return a.score < b.score; };
    std::vector<Cand> all;
    for (double p = cfg.p0; p <= cfg.p1 + 1e-9; p += cfg.coarse)
    for (double r = cfg.r0; r <= cfg.r1 + 1e-9; r += cfg.coarse)
    for (double y = cfg.y0; y <= cfg.y1 + 1e-9; y += cfg.coarse) {
        Cand c;
        if (eval_cand(opt, cfg, p, r, y, c)) all.push_back(c);
    }
    so.n_coarse = all.size();
    if (all.empty()) return so;
    so.coarse_best = *std::min_element(all.begin(), all.end(), score_less);

    std::vector<Cand> fine_list;
    const Cand& b1 = so.coarse_best;
    for (double p = b1.p - 1.5; p <= b1.p + 1.5 + 1e-9; p += cfg.fine)
    for (double r = b1.r - 1.5; r <= b1.r + 1.5 + 1e-9; r += cfg.fine)
    for (double y = b1.y - 1.5; y <= b1.y + 1.5 + 1e-9; y += cfg.fine) {
        Cand c;
        if (eval_cand(opt, cfg, p, r, y, c)) fine_list.push_back(c);
    }
    so.fine_ran = !fine_list.empty();
    so.fine_best = so.fine_ran ? *std::min_element(fine_list.begin(), fine_list.end(), score_less)
                               : so.coarse_best;
    // yaw 可辨识性检查 (v2.9.18; 地板 tilt 口径): 单水平面观测不出 yaw (绕竖轴转不改变平面倾角)。
    {
        Cand cp, cm;
        const Cand& bf = so.fine_best;
        if (eval_cand(opt, cfg, bf.p, bf.r, bf.y + 3.0, cp) &&
            eval_cand(opt, cfg, bf.p, bf.r, bf.y - 3.0, cm)) {
            so.yaw_span = std::max(std::fabs(cp.t - bf.t), std::fabs(cm.t - bf.t));
            so.yaw_ident = so.yaw_span > 0.05;
        }
    }
    all.insert(all.end(), fine_list.begin(), fine_list.end());
    std::sort(all.begin(), all.end(), score_less);
    so.ncand = all.size();
    so.top.assign(all.begin(), all.begin() + std::min<size_t>(8, all.size()));
    so.ok = true;
    return so;
}

// ============================================================
// --selftest: 自造帧真值验证 (2026-10-04)
//   合成"地板 + 竖直墙"深度帧 (约定与 cell_synth_check.cpp 的 render_floor 一致) →
//   与真机完全同一条搜索链 (eval_cand/run_search) → 与真值对照断言。
// ============================================================
int g_assert_ok = 0;
int g_assert_fail = 0;
#define ACHECK(cond)                                                          \
    do {                                                                      \
        if (cond) { ++g_assert_ok; }                                          \
        else {                                                                \
            ++g_assert_fail;                                                  \
            printf("  [ASSERT-FAIL] %s:%d  %s\n", __FILE__, __LINE__, #cond); \
        }                                                                     \
    } while (0)

/** 渲染"地板+墙"深度帧 (16UC1 mm; 无效=0)。
 *  相机原点 (0,0,0); 地板 z=-cam_h; 墙: 法向 u=(cosψ,sinψ,0) 朝机器人, 面在 u·X=wall_dist;
 *  与 cell_synth_check.cpp 的 render_floor 同约定 (optical→link→Rx→Ry→Rz), 噪声 N(0,noise_mm), dropout 丢点。 */
std::vector<uint16_t> render_floor_wall(int W, int H_px, double fx, double fy, double cx, double cy,
                                        double cam_h,
                                        double pitch_deg, double roll_deg, double yaw_deg,
                                        double wall_psi_deg, double wall_dist,
                                        double noise_mm, double dropout, unsigned seed) {
    std::vector<uint16_t> depth((size_t)W * H_px, 0);
    const double r = roll_deg * kD2R, pch = pitch_deg * kD2R, y = yaw_deg * kD2R;
    const double cr = std::cos(r), sr = std::sin(r);
    const double cp = std::cos(pch), sp = std::sin(pch);
    const double cyw = std::cos(y), syw = std::sin(y);
    auto rot = [&](double x, double yy, double z, double& ox, double& oy, double& oz) {
        // ★ 先做 optical→link 固定旋转 (与 point_cloud.cpp 同约定): X_link=Z_opt, Y_link=-X_opt, Z_link=-Y_opt
        { const double xo = z, yo = -x, zo = -yy; x = xo; yy = yo; z = zo; }
        // 再做外参 Rz(yaw)·Ry(pitch)·Rx(roll)
        const double x1 = x, y1 = cr * yy - sr * z, z1 = sr * yy + cr * z;
        const double x2 = cp * x1 + sp * z1, y2 = y1, z2 = -sp * x1 + cp * z1;
        ox = cyw * x2 - syw * y2; oy = syw * x2 + cyw * y2; oz = z2;
    };
    const double psi = wall_psi_deg * kD2R;
    const double ux = std::cos(psi), uy = std::sin(psi);
    std::mt19937 rng(seed);
    std::normal_distribution<double> nn(0.0, noise_mm);
    std::uniform_real_distribution<double> un(0.0, 1.0);
    for (int v = 0; v < H_px; ++v) {
        for (int u = 0; u < W; ++u) {
            double ox, oy, oz;
            rot((u - cx) / fx, (v - cy) / fy, 1.0, ox, oy, oz);         // 光学: x右 y下 z前
            double s = -1.0;                                            // 地板: z=-cam_h
            if (oz < -1e-6) s = (-cam_h) / oz;
            const double ud = ux * ox + uy * oy;                        // 墙: u·X = wall_dist
            if (ud > 1e-6) {
                const double sw = wall_dist / ud;
                if (sw > 1e-6 && (s < 0.0 || sw < s)) s = sw;
            }
            if (!(s > 0.0) || s > 8.0) continue;                        // 量程
            if (un(rng) < dropout) continue;                            // 丢点(反光/无回波)
            const double mm = s * 1000.0 + nn(rng);
            if (mm < 300.0 || mm > 8000.0) continue;
            depth[(size_t)v * W + u] = (uint16_t)mm;
        }
    }
    return depth;
}

struct SfCase {
    const char* name;
    double H, tp, tr, ty;
    std::vector<double> psis;
    double dist, noise, drop;
};

void selftest_case(const SfCase& c, int idx) {
    const int W = 640, HP = 480;
    const double fx = 570.3422047415297129, fy = fx, ccx = 319.5, ccy = 239.5;
    std::vector<PointCloud> opt;
    for (size_t k = 0; k < c.psis.size(); ++k) {
        const std::vector<uint16_t> depth = render_floor_wall(
            W, HP, fx, fy, ccx, ccy, c.H, c.tp, c.tr, c.ty,
            c.psis[k], c.dist, c.noise, c.drop, 20261004u + 1000u * (unsigned)k);
        CameraIntrinsics K; K.fx = fx; K.fy = fy; K.cx = ccx; K.cy = ccy;
        K.min_depth_m = 0.3; K.max_depth_m = 8.0;
        PointCloud pc;
        depth_to_cloud(depth.data(), W, HP, K, pc);
        opt.push_back(pc);
    }
    CalibCfg cfg;                      // 与真机同一条链: step=8, cell 拟合, 原点相机
    cfg.step = 8; cfg.ext_x = 0.0; cfg.ext_z = 0.0;
    cfg.use_cell = true;
    cfg.gp.ground_prior_z = -c.H; cfg.gp.prior_window = 0.10; cfg.gp.plane_max_tilt_deg = 15.0;
    cfg.gp.use_cell_min_fit = true;
    CalibCfg cf = cfg; cf.use_wall = false;
    CalibCfg cw = cfg; cw.use_wall = true; cw.wall_weight = 1.0;
    const SearchOut s0 = run_search(opt, cf);
    const SearchOut s1 = run_search(opt, cw);

    std::string psistr;
    for (size_t k = 0; k < c.psis.size(); ++k) {
        char b[16];
        std::snprintf(b, sizeof b, "%s%.0f", k ? "/" : "", c.psis[k]);
        psistr += b;
    }
    printf("[case %d] %s | H=%.2f 真值 p=%.2f r=%.2f y=%.2f | 墙ψ=%s d=%.2f | 噪声 %.0fmm/%.0f%%\n",
           idx + 1, c.name, c.H, c.tp, c.tr, c.ty, psistr.c_str(), c.dist, c.noise, c.drop * 100.0);
    if (s0.ok)
        printf("  地板-only: p=%7.3f r=%7.3f  tilt=%.3f°  h0=%.3f  (候选 %zu)\n",
               s0.fine_best.p, s0.fine_best.r, s0.fine_best.t, s0.fine_best.h0, s0.ncand);
    else
        printf("  地板-only: 无有效候选\n");
    if (s1.ok)
        printf("  地板+墙:   p=%7.3f r=%7.3f  tilt=%.3f°  wdev=%.3f°  h0=%.3f  (候选 %zu)\n",
               s1.fine_best.p, s1.fine_best.r, s1.fine_best.t, s1.fine_best.wd,
               s1.fine_best.h0, s1.ncand);
    else
        printf("  地板+墙:   无有效候选\n");

    // ── 诊断 (证据留存): 真值点评估 + p 剖面 (r=真值, y=真值) ──
    {
        Cand ct, cfx;
        const bool tf = eval_cand(opt, cf, c.tp, c.tr, c.ty, cfx);
        const bool tw = eval_cand(opt, cw, c.tp, c.tr, c.ty, ct);
        printf("  真值点评估: 地板-only=");
        if (tf) printf("t=%.3f  ", cfx.t); else printf("无效   ");
        printf("| 地板+墙=");
        if (tw) printf("t=%.3f wd=%.3f sum=%.3f", ct.t, ct.wd, ct.score); else printf("无效");
        printf("\n  p-剖面: ");
        for (double pp = c.tp - 1.0; pp <= c.tp + 1.0 + 1e-9; pp += 0.5) {
            Cand a, b;
            const bool va = eval_cand(opt, cf, pp, c.tr, c.ty, a);
            const bool vb = eval_cand(opt, cw, pp, c.tr, c.ty, b);
            printf("p=%.2f[", pp);
            if (va) printf("t_f=%.3f", a.t); else printf("t_f=无效");
            if (vb) printf(" t_w=%.3f wd=%.3f sum=%.3f", b.t, b.wd, b.score); else printf(" t_w=无效");
            printf("] ");
        }
        printf("\n");
    }

    ACHECK(s1.ok);
    if (s1.ok) {
        const Cand& bf = s1.fine_best;
        printf("  误差: Δp=%+.3f° Δr=%+.3f° (验收 <0.5°) | Δh0=%+.3fm (验收 <0.05m)\n",
               bf.p - c.tp, bf.r - c.tr, bf.h0 + c.H);
        ACHECK(std::fabs(bf.p - c.tp) <= 0.5);
        ACHECK(std::fabs(bf.r - c.tr) <= 0.5);
        ACHECK(bf.t <= 0.5);
        ACHECK(std::fabs(bf.h0 + c.H) <= 0.05);
        ACHECK(bf.wd <= 0.5);
        if (s0.ok) {
            const bool same = std::fabs(s0.fine_best.p - bf.p) < 0.25 &&
                              std::fabs(s0.fine_best.r - bf.r) < 0.25;
            printf("  对照: 地板-only 与 地板+墙 解 %s (Δp=%.3f° Δr=%.3f°)\n",
                   same ? "一致" : "不一致!",
                   s0.fine_best.p - bf.p, s0.fine_best.r - bf.r);
        }
        if (idx == 0) {   // yaw 族结构演示 (仅 case 1): 沿 yaw 移动, tilt 恒定 ⇒ 不可辨识
            Cand cy1, cy2;
            const bool ok1 = eval_cand(opt, cw, bf.p, bf.r, bf.y + 10.0, cy1);
            const bool ok2 = eval_cand(opt, cw, bf.p, bf.r, bf.y - 10.0, cy2);
            if (ok1 && ok2) {
                printf("  yaw±10° 微扰: tilt=%.3f°/%.3f°  wdev=%.3f°/%.3f°  (族内恒定 ⇒ yaw 不可辨识)\n",
                       cy1.t, cy2.t, cy1.wd, cy2.wd);
            }
            ACHECK(ok1 && cy1.t <= 0.5);
            ACHECK(ok2 && cy2.t <= 0.5);
        }
    }
}

int run_selftest() {
    printf("=== 自造帧真值验证 (--selftest): 地板+墙合成深度帧 → 标定搜索 → 真值对照 ===\n");
    printf("    内参 fx=fy=570.342 cx=319.5 cy=239.5 | step=8 cell=1 | 搜索 p[-2,18] r[-8,8] y[-12,12] coarse=2 fine=0.25\n");
    printf("    合成约定: 相机原点(0,0,0); 地板 z=-H; 墙法向 u=(cosψ,sinψ,0), 面在 u·X=d (法向朝机器人)\n");
    const std::vector<SfCase> cases = {
        {"装机15° ψ=0",  0.18, 15.0,  0.0,   0.0, {0.0},             1.5,  8.0, 0.10},
        {"yaw=+10°",     0.18, 15.0,  0.0, +10.0, {0.0},             1.5,  8.0, 0.10},
        {"yaw=-10°",     0.18, 15.0,  0.0, -10.0, {0.0},             1.5,  8.0, 0.10},
        {"roll=+3°",     0.18, 15.0, +3.0,   0.0, {0.0},             1.5,  8.0, 0.10},
        {"台架0.75m",    0.75,  0.0,  0.0,   0.0, {0.0},             3.5,  8.0, 0.10},
        {"墙斜60°",      0.18, 15.0,  0.0,   0.0, {60.0},            1.5,  8.0, 0.10},
        {"三帧联合",     0.18, 15.0,  0.0,  +3.0, {0.0, 45.0, -60.0}, 2.0,  8.0, 0.10},
        {"高噪声三帧",   0.18, 15.0,  0.0,   0.0, {0.0, 45.0, -60.0}, 1.5, 16.0, 0.20},
        {"极低噪0.5mm",  0.18, 15.0,  0.0,   0.0, {0.0},             1.5,  0.5, 0.00},
        {"低噪4mm",      0.18, 15.0,  0.0,   0.0, {0.0},             1.5,  4.0, 0.10},
        {"深地板3m",     0.18, 15.0,  0.0,   0.0, {0.0},             3.0,  8.0, 0.10},
        {"侧墙90°",      0.18, 15.0,  0.0,   0.0, {90.0},            1.5,  8.0, 0.10},
    };
    for (size_t i = 0; i < cases.size(); ++i) selftest_case(cases[i], (int)i);
    printf("\n=== 自造帧真值验证汇总: 断言通过 %d, 失败 %d ===\n", g_assert_ok, g_assert_fail);
    return g_assert_fail > 0 ? 1 : 0;
}

} // namespace

int main(int argc, char** argv) {
    const int W = 640, H = 480;
    std::string intr_path;
    int step = 8;
    double ext_z = 0.0, ext_x = 0.0, prior = 0.35, prior_z = -0.9, max_tilt = 15.0;
    double p0 = -2, p1 = 18, r0 = -8, r1 = 8, y0 = -12, y1 = 12;
    double coarse = 2.0, fine = 0.25;
    bool use_cell = false;
    bool use_wall = false, do_selftest = false;
    double wall_weight = 1.0, wall_max_dev = 10.0, wall_rms = 0.05;
    int wall_min_pts = 60;
    std::vector<std::string> frames;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto nx = [&](const char* name, double d) -> double {
            // v2.9.18 (T4): 缺值/非法值 ⇒ 立即报错。旧版 atof 静默吞参:
            //   `--z --cell` 会把 `--cell` 当 0.0 吃掉并输出一组错误外参。
            if (i + 1 >= argc) { printf("%s 缺参数值\n", name); exit(2); }
            char* end = nullptr;
            const double v = std::strtod(argv[++i], &end);
            if (end == argv[i] || *end != '\0') {
                printf("%s 参数值无效: '%s'\n", name, argv[i]); exit(2);
            }
            return v;
        };
        if (a == "--intr") {
            if (i + 1 >= argc) { printf("--intr 缺参数值\n"); return 2; }
            intr_path = argv[++i];
        }
        else if (a == "--step") step = (int)nx("--step", step);
        else if (a == "--z") ext_z = nx("--z", ext_z);
        else if (a == "--x") ext_x = nx("--x", ext_x);
        else if (a == "--prior") prior = nx("--prior", prior);
        else if (a == "--prior-z") prior_z = nx("--prior-z", prior_z);
        else if (a == "--tilt") max_tilt = nx("--tilt", max_tilt);
        else if (a == "--p0") p0 = nx("--p0", p0); else if (a == "--p1") p1 = nx("--p1", p1);
        else if (a == "--r0") r0 = nx("--r0", r0); else if (a == "--r1") r1 = nx("--r1", r1);
        else if (a == "--y0") y0 = nx("--y0", y0); else if (a == "--y1") y1 = nx("--y1", y1);
        else if (a == "--coarse") coarse = nx("--coarse", coarse);
        else if (a == "--fine") fine = nx("--fine", fine);
        else if (a == "--cell") use_cell = true;
        else if (a == "--wall") use_wall = true;
        else if (a == "--wall-weight") wall_weight = nx("--wall-weight", wall_weight);
        else if (a == "--wall-max-dev") wall_max_dev = nx("--wall-max-dev", wall_max_dev);
        else if (a == "--wall-rms") wall_rms = nx("--wall-rms", wall_rms);
        else if (a == "--wall-min-pts") wall_min_pts = (int)nx("--wall-min-pts", wall_min_pts);
        else if (a == "--selftest") do_selftest = true;
        else frames.push_back(a);
    }
    if (do_selftest) return run_selftest();   // 自检模式: 不需要 --intr / 帧文件
    if (intr_path.empty() || frames.empty()) { printf("参数不全\n"); return 2; }
    if (step < 1) { printf("--step 必须 >= 1 (推荐 4~16); 收到 %d\n", step); return 2; }
    if (coarse <= 0.0 || fine <= 0.0) { printf("--coarse/--fine 必须 > 0\n"); return 2; }
    if (use_wall && (wall_weight <= 0.0 || wall_max_dev <= 0.0 || wall_max_dev > 45.0 ||
                     wall_rms <= 0.0 || wall_min_pts < 3)) {
        printf("--wall-* 参数非法 (weight>0, max-dev∈(0,45], rms>0, min-pts>=3)\n"); return 2;
    }
    CameraIntrinsics K; {
        std::ifstream f(intr_path); double w,h,fx,fy,cx,cy;
        if (!(f >> w >> h >> fx >> fy >> cx >> cy) || fx <= 0) { printf("内参失败\n"); return 2; }
        K.fx = fx; K.fy = fy; K.cx = cx; K.cy = cy;
    }
    // 预加载 + 一次反投影 (与 E 无关, 只做一次)
    // v2.9.18 (T4): 坏帧(全零/低有效, 真机已知现象)载入时剔除 —— 旧版"任一帧拟合失败
    //   即整体作废"会把正确外参否掉; 有效帧之间仍保持严格多帧联合(防假解设计不变)。
    std::vector<PointCloud> opt;
    size_t dropped = 0;
    for (size_t i = 0; i < frames.size(); ++i) {
        std::vector<uint16_t> raw;
        if (!load_raw(frames[i], W, H, raw)) { printf("读帧失败 %s\n", frames[i].c_str()); return 2; }
        const size_t nv = count_valid_raw(raw);
        if (nv * 4 < raw.size()) {   // <25% (与节点深度守门口径一致)
            printf("剔除坏帧 %s (有效像素 %zu/%zu = %.1f%%)\n",
                   frames[i].c_str(), nv, raw.size(), 100.0 * (double)nv / (double)raw.size());
            ++dropped;
            continue;
        }
        PointCloud pc;
        depth_to_cloud(raw.data(), W, H, K, pc);
        opt.push_back(pc);
    }
    if (opt.empty()) { printf("全部帧均无效 —— 检查帧文件 / 内参\n"); return 2; }
    if (dropped) printf("注意: 已剔除 %zu/%zu 帧\n", dropped, frames.size());

    CalibCfg cfg;
    cfg.step = step; cfg.ext_x = ext_x; cfg.ext_z = ext_z;
    cfg.gp.ground_prior_z = prior_z; cfg.gp.prior_window = prior;
    cfg.gp.plane_max_tilt_deg = max_tilt; cfg.gp.use_cell_min_fit = use_cell;
    cfg.use_cell = use_cell;
    cfg.use_wall = use_wall; cfg.wall_weight = wall_weight;
    cfg.wp.max_dev_deg = wall_max_dev; cfg.wp.min_inliers = wall_min_pts; cfg.wp.rms_max = wall_rms;
    cfg.p0 = p0; cfg.p1 = p1; cfg.r0 = r0; cfg.r1 = r1; cfg.y0 = y0; cfg.y1 = y1;
    cfg.coarse = coarse; cfg.fine = fine;

    printf("帧数=%zu step=%d prior=%.2f@%.2f cell=%d wall=%d (剔除坏帧 %zu)\n",
           opt.size(), step, prior, prior_z, (int)use_cell, (int)use_wall, dropped);
    const SearchOut so = run_search(opt, cfg);
    printf("粗扫完成: %zu 个有效候选\n", so.n_coarse);
    if (!so.ok) {
        if (use_wall) printf("无有效候选 —— 放宽 --prior 或扩大范围; --wall 模式还须帧内可见竖直墙\n");
        else printf("无有效候选 —— 放宽 --prior 或扩大范围\n");
        return 1;
    }
    {
        const Cand& b1 = so.coarse_best;
        if (use_wall)
            printf("粗扫最优: pitch=%.2f roll=%.2f yaw=%.2f  tilt=%.3f  wdev=%.3f  h0=%.3f  inl=%.0f\n",
                   b1.p, b1.r, b1.y, b1.t, b1.wd, b1.h0, b1.inl);
        else
            printf("粗扫最优: pitch=%.2f roll=%.2f yaw=%.2f  tilt=%.3f  h0=%.3f  inl=%.0f\n",
                   b1.p, b1.r, b1.y, b1.t, b1.h0, b1.inl);
    }
    const Cand& b1 = so.fine_best;
    if (so.fine_ran) {
        if (use_wall)
            printf("精扫最优: pitch=%.3f roll=%.3f yaw=%.3f  tilt=%.3f  wdev=%.3f  h0=%.3f  inl=%.0f\n",
                   b1.p, b1.r, b1.y, b1.t, b1.wd, b1.h0, b1.inl);
        else
            printf("精扫最优: pitch=%.3f roll=%.3f yaw=%.3f  tilt=%.3f  h0=%.3f  inl=%.0f\n",
                   b1.p, b1.r, b1.y, b1.t, b1.h0, b1.inl);
    }
    // v2.9.18 (T4): yaw 可辨识性检查 —— 单水平面观测不出 yaw (绕竖轴转不改变平面倾角)。
    //   ±3° 微扰下平均 tilt 几乎不变 ⇒ 打警告, 且"建议参数"里不再照抄一个纯噪声值。
    //   (2026-10-04: 含墙亦然 —— 墙项沿 yaw 一维族恒定, 数值实验已证实)
    printf("--- 最平的 8 个候选 (排除假解请看 h0 是否与实测镜头高一致) ---\n");
    for (size_t i = 0; i < so.top.size(); ++i) {
        const Cand& c = so.top[i];
        if (use_wall)
            printf("  #%zu pitch=%+.2f roll=%+.2f yaw=%+.2f  tilt=%.3f  wdev=%.3f  h0=%.3f  inl=%.0f\n",
                   i + 1, c.p, c.r, c.y, c.t, c.wd, c.h0, c.inl);
        else
            printf("  #%zu pitch=%+.2f roll=%+.2f yaw=%+.2f  tilt=%.3f  h0=%.3f  inl=%.0f\n",
                   i + 1, c.p, c.r, c.y, c.t, c.h0, c.inl);
    }
    if (so.yaw_ident) {
        printf("=== 建议参数 (度): pitch=%.2f roll=%.2f yaw=%.2f ===\n", b1.p, b1.r, b1.y);
        printf("    (yaw 可辨识性: ±3° 微扰 tilt 变化 %.3f°, 通过)\n", so.yaw_span);
    } else {
        printf("=== 建议参数 (度): pitch=%.2f roll=%.2f ===\n", b1.p, b1.r);
        if (so.yaw_span >= 0.0)
            printf("    !! yaw 不可辨识: ±3° 微扰下 tilt 变化仅 %.3f° (<0.05°) —— 单水平面观测不出,\n"
                   "       不要照抄候选表里的 yaw; 装夹对齐机体后按 0 处理 (见 launch camera_yaw_rad 注)\n", so.yaw_span);
        else
            printf("    !! yaw 不可辨识: 微扰帧拟合失败无法评估; 单水平面观测不出, 请勿照抄 yaw\n");
    }
    return 0;
}
