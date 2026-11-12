#pragma once
// 20261107回传board2只作为回归夹具, 不自动部署为现场标定.
#include "sentinel_kinematics.hpp"
namespace ec::sentinel::test {
inline constexpr std::optional<app::SerialGeometry> heading_geometry{
    app::SerialGeometry{
        {-0.086802847f, 0.031076549f, 0.995740686f},
        {0.014223269f, -0.999415767f, 0.031077689f},
        {-0.097089624f, 0.027570401f, 0.994893702f},
        {0.997511309f, 0.016329022f, 0.068589730f},
        {0.001770788f, -0.000808838f, 0.000463396f},
        estimation::identity_rotation(),
        -0.099708751f, -0.052262315f,
        {0, 0}, {0, 0},
        0.01f, 0.02f, 0.8f
    }
};
} // 命名空间 ec::sentinel::test
