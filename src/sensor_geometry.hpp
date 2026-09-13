/**
 * v2.6: 相机几何 → 地面高度先验 (纯函数, 便于单测)。
 *
 * 背景 (真机实测 2026-09-13): `GroundSegConfig::ground_prior_z`(-0.18) 是**装机值**
 * ——它假设相机镜头离地 18cm。台架/手持时相机常架在 0.4~0.6m, 于是**真地面落在
 * 先验窗之外**, 拟合器只能锁到别的面, 2.5D 的 known/traversable/up/down 全部失去
 * 物理意义。同一帧实测 (相机离地 0.657m, 见 transform_to_base 推算):
 *     prior_z = -0.18 → 可通行 0 / 已知 35 格 (全是"凸起", 无意义)
 *     prior_z = -0.60 → 可通行 22 / 已知 35 格 (真实地板被正确识别) ✓
 *
 * 推导: `transform_to_base` 里 `d.z = R·z_cam + E.z`
 *   ⇒ base 系下的地面高度 = -(相机离地高 - E.z)
 *   (E.z = 相机相对 base 原点的高度; 台架把 base 原点放在相机上时 E.z = 0)
 *
 * 用法: 台架 0.6m → camera_height_m=0.6, cloud_z=0 → prior_z=-0.6
 *       装机 0.2m → camera_height_m=0.2, cloud_z 按量测 → prior_z ≈ -0.2
 */
#pragma once

namespace mechdog_ros {

inline double derived_ground_prior_z(double camera_height_m, double ext_z_m) {
    return -(camera_height_m - ext_z_m);
}

} // namespace mechdog_ros
