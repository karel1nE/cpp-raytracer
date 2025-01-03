#pragma once

#include "vector.h"

class Intersection {
public:
    double distance_;
    Vector position_, normal_;

    Intersection(const Vector& position, const Vector& normal, double distance) {
        position_ = position;
        normal_ = normal;
        distance_ = distance;
    }

    const Vector& GetPosition() const {
        return position_;
    }
    const Vector& GetNormal() const {
        return normal_;
    }
    double GetDistance() const {
        return distance_;
    }
};
