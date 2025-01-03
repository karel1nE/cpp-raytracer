#pragma once

#include "vector.h"
#include "sphere.h"
#include "intersection.h"
#include "triangle.h"
#include "ray.h"

#include <optional>

std::optional<Intersection> GetIntersection(const Ray& ray, const Sphere& sphere) {
    Vector u = ray.GetDirection();
    Vector o_minus_c = ray.GetOrigin() - sphere.GetCenter();
    double r = sphere.GetRadius();

    double a = DotProduct(u, u);
    double b = 2 * DotProduct(u, o_minus_c);
    double c = DotProduct(o_minus_c, o_minus_c) - r * r;

    double delta = b * b - 4 * a * c;

    if (delta < 0) {
        return {};
    }

    double d1 = (-b - std::sqrt(delta)) / (2. * a);
    double d2 = (-b + std::sqrt(delta)) / (2. * a);

    double dist = (d1 > 0) ? d1 : d2;

    if (dist < 0) {
        return {};
    }

    Vector position = ray.GetOrigin() + ray.GetDirection() * dist;
    Vector normal = position - sphere.GetCenter();
    normal.Normalize();
    if (DotProduct(normal, ray.GetDirection()) > 0) {
        normal = normal * -1.0;
    }
    dist = Length(position - ray.GetOrigin());
    return Intersection(position, normal, dist);
}

std::optional<Intersection> GetIntersection(const Ray& ray, const Triangle& triangle) {
    constexpr double kEpsilon = std::numeric_limits<float>::epsilon();
    Vector edge1 = triangle[1] - triangle[0];
    Vector edge2 = triangle[2] - triangle[0];
    Vector ray_cross_e2 = CrossProduct(ray.GetDirection(), edge2);
    double det = DotProduct(edge1, ray_cross_e2);

    if (det > -kEpsilon && det < kEpsilon) {
        return {};
    }

    double inv_det = 1. / det;
    Vector s = ray.GetOrigin() - triangle[0];
    double u = inv_det * DotProduct(s, ray_cross_e2);

    if (u < 0 || u > 1) {
        return {};
    }

    Vector s_cross_e1 = CrossProduct(s, edge1);
    double v = inv_det * DotProduct(ray.GetDirection(), s_cross_e1);

    if (v < 0 || u + v > 1) {
        return {};
    }

    double t = inv_det * DotProduct(edge2, s_cross_e1);
    if (t > kEpsilon) {
        Vector position = ray.GetOrigin() + ray.GetDirection() * t;
        Vector normal = CrossProduct(position - triangle[0], position - triangle[1]);
        normal.Normalize();
        if (DotProduct(normal, ray.GetDirection()) > 0) {
            normal = normal * -1.0;
        }
        double dist = Length(ray.GetOrigin() - position);
        return Intersection(position, normal, dist);
    } else {
        return {};
    }
}

Vector Reflect(const Vector& ray, const Vector& normal) {
    double dot = DotProduct(ray, normal);
    if (dot < 0) {
        dot *= -1;
    }
    Vector ans = ray + normal * 2. * dot;
    ans.Normalize();
    return ans;
}

std::optional<Vector> Refract(const Vector& ray, const Vector& normal, double eta) {
    double r = eta;
    double c = -DotProduct(normal, ray);
    if (1 - r * r * (1 - c * c) < 0) {
        return {};
    }
    Vector ans = ray * r + normal * (r * c - sqrt(1 - r * r * (1 - c * c)));
    ans.Normalize();
    return ans;
}

Vector GetBarycentricCoords(const Triangle& triangle, const Vector& point) {
    double area_abc = triangle.Area();
    Triangle pab(point, triangle[0], triangle[1]);
    Triangle pac(point, triangle[0], triangle[2]);
    Triangle pbc(point, triangle[1], triangle[2]);
    double u = pbc.Area() / area_abc;
    double v = pac.Area() / area_abc;
    double w = pab.Area() / area_abc;
    return {u, v, w};
}
