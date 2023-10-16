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

#ifndef DIFFERENTIAL_WHEEL_HPP__
#define DIFFERENTIAL_WHEEL_HPP__

#include <math.h>

namespace vulcan_wheel
{

class DifferentialDriveOdometry {
public:
    DifferentialDriveOdometry(double wheel_diameter, int encoder_counts_per_revolution, double wheel_base)
        : wheel_radius_(wheel_diameter / 2.0)
        , counts_per_revolution_(encoder_counts_per_revolution)
        , wheel_base_(wheel_base)
        , x_(0.0)
        , y_(0.0)
        , theta_(0.0)
    {}

    void update(int left_encoder_counts, int right_encoder_counts);

    // 获取当前位置和方向
    double getX() const {
        return x_;
    }

    double getY() const {
        return y_;
    }

    double getTheta() const {
        return theta_;
    }

private:
    // 将编码器计数转换为距离
    double countToDistance(int counts) {
        double revolutions = static_cast<double>(counts) / counts_per_revolution_;
        return revolutions * 2.0 * M_PI * wheel_radius_;
    }

    double wheel_radius_;
    int counts_per_revolution_;
    double wheel_base_;
    double x_;
    double y_;
    double theta_;
};
} // namespace vulcan_wheel


#endif /*DIFFERENTIAL_WHEEL_HPP__*/
