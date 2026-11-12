#pragma once
// 10310221现场接线. 顺序为四轮LF/RF/LB/RB、main/fire、摩擦左/右、拨弹、pitch.
#include "../../../robots/sentinel/capture/capture.hpp"
namespace ec::sentinel::board {
inline constexpr std::array<capture::Install,10> current_harness{{
    {1,capture::Kind::c620,0x203,3},{1,capture::Kind::c620,0x201,1},
    {1,capture::Kind::c620,0x204,4},{1,capture::Kind::c620,0x202,2},
    {1,capture::Kind::gm6020,0x208,4},{2,capture::Kind::gm6020,0x20a,6},
    {2,capture::Kind::c620,0x205,5},{2,capture::Kind::c620,0x206,6},
    {2,capture::Kind::c610,0x207,7},{2,capture::Kind::dm,0x011,1}}};
} // 命名空间 ec::sentinel::board
