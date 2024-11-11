/**
 * @file Types.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-11-11
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <array>
#include <cstddef>

namespace ocs2
{
    namespace hexapod_robot
    {
        template <typename T>
        using feet_array_t = std::array<T, 4>;
        using contact_flag_t = feet_array_t<bool>;
    } // namespace hexapod_robot
} // namespace ocs2