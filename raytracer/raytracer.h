#pragma once

#include "options/camera_options.h"
#include "options/render_options.h"
#include "image.h"
#include "scene.h"
#include "ray.h"
#include "geometry.h"

#include <filesystem>

constexpr double kEps = 1e-6;

std::optional<std::tuple<Intersection, Material, bool>> Porsche911(const Ray& ray,
                                                                   const Scene& scene) {
    Material mat;
    Intersection inter{Vector(0, 0, 0), Vector(0, 0, 0), -1};
    bool is_sphere = false;
    for (const auto& sphere : scene.GetSphereObjects()) {
        auto x = GetIntersection(ray, sphere.sphere);
        if (!x.has_value()) {
            continue;
        }
        if (inter.GetDistance() > x.value().GetDistance() || inter.GetDistance() == -1) {
            inter = x.value();
            mat = *sphere.material;
            is_sphere = true;
        }
    }
    for (const auto& obj : scene.GetObjects()) {
        auto x = GetIntersection(ray, obj.polygon);
        if (!x.has_value()) {
            continue;
        }
        if (inter.GetDistance() > x.value().GetDistance() || inter.GetDistance() == -1) {
            inter = x.value();
            mat = *obj.material;
            if (obj.has_norm) {
                Vector coords = GetBarycentricCoords(obj.polygon, inter.GetPosition());
                inter.normal_ = obj.normals[0] * coords[0] + obj.normals[1] * coords[1] +
                                obj.normals[2] * coords[2];
            }
        }
    }
    if (inter.GetDistance() == -1) {
        return {};
    }

    return std::tuple<Intersection, Material, bool>{inter, mat, is_sphere};
}

Vector VolkswagenPassat(const Vector& v1, const Vector& v2) {
    return {v1[0] * v2[0], v1[1] * v2[1], v1[2] * v2[2]};
}

Vector AudiA8(const Ray& ray, const Scene& scene, int cur_depth, int depth, bool in_sphere) {
    Vector ans = {0, 0, 0};

    if (cur_depth >= depth) {
        return ans;
    }

    auto p = Porsche911(ray, scene);

    if (!p.has_value()) {
        return ans;
    }

    auto& inter = std::get<0>(p.value());
    auto& mat = std::get<1>(p.value());
    bool is_inter_sphere = std::get<2>(p.value());

    ans = ans + mat.ambient_color + mat.intensity;

    auto refl = Reflect(ray.GetDirection(), inter.GetNormal());
    refl.Normalize();
    Ray refl_ray = {inter.GetPosition() + inter.GetNormal() * kEps, refl};

    if (!in_sphere) {
        // ans = ans + AudiA8(refl_ray, scene, cur_depth + 1, depth, in_sphere) * mat.albedo[1];
        ans = ans + AudiA8(refl_ray, scene, cur_depth + 1, depth, 0) * mat.albedo[1];
    }

    auto refr = Refract(ray.GetDirection(), inter.GetNormal(),
                        !in_sphere ? 1.0 / mat.refraction_index : mat.refraction_index);
    if (refr.has_value()) {
        refr.value().Normalize();
        Ray refr_ray = {inter.GetPosition() - inter.GetNormal() * kEps, refr.value()};
        ans = ans + AudiA8(refr_ray, scene, cur_depth + 1, depth, is_inter_sphere == !in_sphere) *
                        (!in_sphere ? mat.albedo[2] : 1.0);
        // ans = ans + AudiA8(refr_ray, scene, cur_depth + 1, depth, is_inter_sphere == !in_sphere)
        // * mat.albedo[2];
    }

    Vector lights_sum = {0, 0, 0};

    for (const auto& light : scene.GetLights()) {
        Vector v_l = light.position - inter.GetPosition();
        double dist_to_light = Length(v_l);
        v_l.Normalize();

        // auto p = Porsche911({inter.GetPosition() + v_l * kEps, v_l}, scene);
        auto p = Porsche911({inter.GetPosition() + inter.GetNormal() * kEps, v_l}, scene);
        if (p.has_value() && std::get<0>(p.value()).GetDistance() < dist_to_light) {
            continue;
        }

        Vector l_d = VolkswagenPassat(light.intensity, mat.diffuse_color) *
                     std::max(0.0, DotProduct(inter.GetNormal(), v_l));

        Vector v_r = inter.GetNormal() * DotProduct(inter.GetNormal(), v_l) * 2 - v_l;
        v_r.Normalize();
        Vector v_e = ray.GetOrigin() - inter.GetPosition();
        v_e.Normalize();

        // Vector v_r = Reflect(v_l * -1, inter.GetNormal()) * -1;
        // v_r.Normalize();
        // Vector v_e = ray.GetDirection();
        // v_e.Normalize();

        Vector l_s = VolkswagenPassat(light.intensity, mat.specular_color) *
                     std::pow(std::max(0.0, DotProduct(v_r, v_e)), mat.specular_exponent);

        lights_sum = lights_sum + l_d + l_s;
    }

    ans = ans + lights_sum * mat.albedo[0];

    return ans;
}

Image Render(const std::filesystem::path& path, const CameraOptions& camera_options,
             const RenderOptions& render_options) {
    Scene scene = ReadScene(path);
    double width = camera_options.screen_width, height = camera_options.screen_height;
    Image image(width, height);
    std::vector<std::vector<Vector>> raw_pix(width, std::vector<Vector>(height));

    Vector rel_z = camera_options.look_to - camera_options.look_from;
    double dy = Length(rel_z) * tan(camera_options.fov / 2);
    double dx = (dy * width) / height;
    Vector up, right;

    if (std::abs(rel_z[0]) < kEps && std::abs(rel_z[2]) < kEps) {
        right = Vector(1, 0, 0) * rel_z[1];
        up = Vector(0, 0, -1) * rel_z[1];
    } else {
        right = CrossProduct(rel_z, {0, 1, 0});
        up = CrossProduct(rel_z, right);
    }

    right.Normalize();
    up.Normalize();

    Vector rel_x = right * (2 * dx);
    Vector rel_y = up * (2 * dy);

    Vector start_vec = rel_z - (rel_x * 0.5) - (rel_y * 0.5);

    if (render_options.mode == RenderMode::kDepth) {

        double max_d = -1;

        for (int w = 0; w < width; ++w) {
            for (int h = 0; h < height; ++h) {
                Vector origin = camera_options.look_from;
                Vector direction =
                    start_vec + (rel_x * ((w + 0.5) / width)) + (rel_y * ((h + 0.5) / height));
                direction.Normalize();
                Ray ray(origin, direction);

                double d = -1;
                bool was = 0;

                for (const auto& sphere : scene.GetSphereObjects()) {
                    auto x = GetIntersection(ray, sphere.sphere);
                    if (x.has_value()) {
                        if (!was) {
                            was = 1;
                            d = x.value().GetDistance();
                        }
                        d = std::min(d, x.value().GetDistance());
                    }
                }
                for (const auto& obj : scene.GetObjects()) {
                    auto x = GetIntersection(ray, obj.polygon);
                    if (x.has_value()) {
                        if (!was) {
                            was = 1;
                            d = x.value().GetDistance();
                        }
                        d = std::min(d, x.value().GetDistance());
                    }
                }

                raw_pix[w][h] = {d, d, d};
                max_d = std::max(d, max_d);
            }
        }

        for (int w = 0; w < width; ++w) {
            for (int h = 0; h < height; ++h) {
                if (raw_pix[w][h][0] == -1) {
                    raw_pix[w][h] = {1, 1, 1};
                } else {
                    raw_pix[w][h] = raw_pix[w][h] * (1 / max_d);
                }
                RGB rgb = {static_cast<int>(raw_pix[w][h][0] * 255),
                           static_cast<int>(raw_pix[w][h][1] * 255),
                           static_cast<int>(raw_pix[w][h][2] * 255)};
                image.SetPixel(rgb, h, w);
            }
        }
    }

    if (render_options.mode == RenderMode::kNormal) {

        for (int w = 0; w < width; ++w) {
            for (int h = 0; h < height; ++h) {
                Vector origin = camera_options.look_from;
                Vector direction =
                    start_vec + (rel_x * ((w + 0.5) / width)) + (rel_y * ((h + 0.5) / height));
                direction.Normalize();
                Ray ray(origin, direction);

                raw_pix[w][h] = {-2, -2, -2};
                double d = -1;

                for (const auto& sphere : scene.GetSphereObjects()) {
                    auto x = GetIntersection(ray, sphere.sphere);
                    if (!x.has_value()) {
                        continue;
                    }
                    if (d > x.value().GetDistance() || d == -1) {
                        d = x.value().GetDistance();
                        raw_pix[w][h] = x.value().GetNormal();
                    }
                }
                for (const auto& obj : scene.GetObjects()) {
                    auto x = GetIntersection(ray, obj.polygon);
                    if (!x.has_value()) {
                        continue;
                    }
                    if (d > x.value().GetDistance() || d == -1) {
                        d = x.value().GetDistance();
                        if (obj.has_norm) {
                            Vector coords =
                                GetBarycentricCoords(obj.polygon, x.value().GetPosition());
                            raw_pix[w][h] = obj.normals[0] * coords[0] +
                                            obj.normals[1] * coords[1] + obj.normals[2] * coords[2];
                        } else {
                            raw_pix[w][h] = x.value().GetNormal();
                        }
                    }
                }
            }
        }

        for (int w = 0; w < width; ++w) {
            for (int h = 0; h < height; ++h) {
                if (raw_pix[w][h][0] == -2 || std::isnan(raw_pix[w][h][0])) {
                    raw_pix[w][h] = {0, 0, 0};
                } else {
                    raw_pix[w][h] = raw_pix[w][h] * 0.5 + Vector(0.5, 0.5, 0.5);
                }
                RGB rgb = {static_cast<int>(raw_pix[w][h][0] * 255),
                           static_cast<int>(raw_pix[w][h][1] * 255),
                           static_cast<int>(raw_pix[w][h][2] * 255)};
                image.SetPixel(rgb, h, w);
            }
        }
    }

    if (render_options.mode == RenderMode::kFull) {

        double c = -1;

        for (int w = 0; w < width; ++w) {
            for (int h = 0; h < height; ++h) {
                Vector origin = camera_options.look_from;
                Vector direction =
                    start_vec + (rel_x * ((w + 0.5) / width)) + (rel_y * ((h + 0.5) / height));
                direction.Normalize();
                Ray ray(origin, direction);

                bool in_sphere = false;

                for (const auto& sphere : scene.GetSphereObjects()) {
                    Vector center = sphere.sphere.GetCenter();
                    double radius = sphere.sphere.GetRadius();
                    if (Length(camera_options.look_from - center) < radius) {
                        in_sphere = true;
                    }
                }

                raw_pix[w][h] = AudiA8(ray, scene, 0, render_options.depth, in_sphere);
                c = std::max(raw_pix[w][h][0], c);
                c = std::max(raw_pix[w][h][1], c);
                c = std::max(raw_pix[w][h][2], c);
            }
        }

        for (int w = 0; w < width; ++w) {
            for (int h = 0; h < height; ++h) {
                raw_pix[w][h][0] =
                    raw_pix[w][h][0] * (1 + raw_pix[w][h][0] / c / c) / (1 + raw_pix[w][h][0]);
                raw_pix[w][h][1] =
                    raw_pix[w][h][1] * (1 + raw_pix[w][h][1] / c / c) / (1 + raw_pix[w][h][1]);
                raw_pix[w][h][2] =
                    raw_pix[w][h][2] * (1 + raw_pix[w][h][2] / c / c) / (1 + raw_pix[w][h][2]);

                raw_pix[w][h][0] = std::pow(raw_pix[w][h][0], 1 / 2.2);
                raw_pix[w][h][1] = std::pow(raw_pix[w][h][1], 1 / 2.2);
                raw_pix[w][h][2] = std::pow(raw_pix[w][h][2], 1 / 2.2);

                if (std::isnan(raw_pix[w][h][0])) {
                    raw_pix[w][h][0] = 0;
                }
                if (std::isnan(raw_pix[w][h][1])) {
                    raw_pix[w][h][1] = 0;
                }
                if (std::isnan(raw_pix[w][h][2])) {
                    raw_pix[w][h][2] = 0;
                }

                RGB rgb = {static_cast<int>(raw_pix[w][h][0] * 255),
                           static_cast<int>(raw_pix[w][h][1] * 255),
                           static_cast<int>(raw_pix[w][h][2] * 255)};
                image.SetPixel(rgb, h, w);
            }
        }
    }
    return image;
}
