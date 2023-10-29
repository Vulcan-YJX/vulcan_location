/**
* This file is part of vulcan_location.
*
* Copyright (C) 2023 Vulcan YJX <vulcanyjx@163.com>
* For more information see <https://github.com/Vulcan-YJX/vulcan_location>
*
* vulcan_location is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* vulcan_location is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with stairs_detection. If not, see <http://www.gnu.org/licenses/>.
*/

#include "differential_wheel.hpp"

namespace vulcan_wheel
{
// 更新里程计，提供左右轮的编码器计数
void DifferentialDriveOdometry::update(int left_encoder_counts, int right_encoder_counts)
{
  double left_distance = countToDistance(left_encoder_counts);
  double right_distance = countToDistance(right_encoder_counts);

  // 计算平均距离和角度变化
  double delta_distance = (right_distance + left_distance) / 2.0;
  double delta_theta = (right_distance - left_distance) / wheel_base_;

  // 更新位置和方向
  x_ += delta_distance * cos(theta_);
  y_ += delta_distance * sin(theta_);
  theta_ += delta_theta;
}

}  // namespace vulcan_wheel
