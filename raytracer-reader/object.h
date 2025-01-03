#pragma once

#include "triangle.h"
#include "material.h"
#include "sphere.h"
#include "vector.h"

struct Object {
    const Material* material = nullptr;
    Triangle polygon;
    std::array<Vector, 3> normals;
    bool has_norm = 0;

    const Vector* GetNormal(size_t index) const {
        return &normals[index];
    }
};

struct SphereObject {
    const Material* material = nullptr;
    Sphere sphere;
};
