#pragma once

#include <array>
#include <cstddef>
#include <cmath>

class Vector {
public:
    Vector(){};
    Vector(double x, double y, double z) : data_({x, y, z}){};

    double& operator[](size_t ind) {
        return data_[ind];
    }

    double operator[](size_t ind) const {
        return data_[ind];
    };

    void Normalize() {
        double l = std::sqrt(data_[0] * data_[0] + data_[1] * data_[1] + data_[2] * data_[2]);
        data_[0] /= l;
        data_[1] /= l;
        data_[2] /= l;
    }

    Vector operator+(const Vector& rhs) const {
        double x = rhs.data_[0] + data_[0];
        double y = rhs.data_[1] + data_[1];
        double z = rhs.data_[2] + data_[2];
        return Vector(x, y, z);
    }

    Vector operator-(const Vector& rhs) const {
        double x = data_[0] - rhs.data_[0];
        double y = data_[1] - rhs.data_[1];
        double z = data_[2] - rhs.data_[2];
        return Vector(x, y, z);
    }

    Vector operator*(double rhs) const {
        double x = data_[0] * rhs;
        double y = data_[1] * rhs;
        double z = data_[2] * rhs;
        return Vector(x, y, z);
    }

private:
    std::array<double, 3> data_;
};

double DotProduct(const Vector& a, const Vector& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vector CrossProduct(const Vector& a, const Vector& b) {
    Vector c(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
    return c;
}

double Length(const Vector& v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}
