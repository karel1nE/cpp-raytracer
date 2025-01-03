#pragma once

#include "vector.h"

#include <cstddef>

class Triangle {
public:
    Triangle(const Vector& a, const Vector& b, const Vector& c) {
        data_[0] = a;
        data_[1] = b;
        data_[2] = c;
    }

    const Vector& operator[](size_t ind) const {
        return data_[ind];
    }
    double Area() const {
        return Length(CrossProduct(data_[2] - data_[0], data_[1] - data_[0])) / 2.;
    }

private:
    std::array<Vector, 3> data_;
};
