
#pragma once

#include <cmath>

namespace soft_body {

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vec3 operator+(const Vec3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }

    Vec3 operator-(const Vec3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }

    Vec3 operator*(double scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }

    Vec3 operator/(double scalar) const {
        return {x / scalar, y / scalar, z / scalar};
    }

    double dot(const Vec3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    
    Vec3 cross(const Vec3& other) const {
        return {
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        };
    }


    double length() const {
        return std::sqrt(dot(*this));
    }
};

} // namespace soft_body
