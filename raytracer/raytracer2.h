#pragma once

#include <image.h>
#include <options/camera_options.h>
#include <options/render_options.h>

#include <filesystem>
#include "scene.h"
#include "ray.h"
#include "geometry.h"

const double kEpsil = 0.000000001;

Vector RayCast(const Vector& orig, const Vector& dir, int max_depth, const Scene& scene,
               const int depth = 0, bool is_in = false) {
    if (depth > max_depth) {
        return {0, 0, 0};
    }
    Vector res(0, 0, 0);
    bool is_assigned = false;
    bool is_sphere;
    double dis = -1;
    Intersection in_work({0, 0, 0}, {0, 0, 0}, 0);
    Vector norm_iw(0, 0, 0);
    const Material* material = nullptr;
    Ray r(orig, dir);
    for (const SphereObject& s : scene.sphereObjects) {
        auto t = GetIntersection(r, s.sphere);
        if (t.has_value()) {
            if (!is_assigned) {
                in_work = t.value();
                norm_iw = t->normal_;
                material = s.material;
                dis = t.value().distance_;
                is_assigned = true;
                is_sphere = true;
                continue;
            }
            if (dis > t.value().distance_) {
                in_work = t.value();
                norm_iw = t->normal_;
                material = s.material;
                dis = t.value().distance_;
                is_sphere = true;
            }
        }
    }
    for (const Object& s : scene.objects) {
        auto t = GetIntersection(r, s.polygon);
        if (t.has_value()) {
            if (!is_assigned) {
                in_work = t.value();
                material = s.material;
                dis = t.value().distance_;
                if (s.isSetNorm) {
                    auto u = GetBarycentricCoords(s.polygon, t->position_);
                    norm_iw = s.normales[0] * u[0] + s.normales[1] * u[1] + s.normales[2] * u[2];
                } else {
                    norm_iw = t->normal_;
                }
                is_assigned = true;
                is_sphere = false;
                continue;
            }
            if (dis > t.value().distance_) {
                in_work = t.value();
                material = s.material;
                dis = t.value().distance_;
                is_sphere = false;
                if (s.isSetNorm) {
                    auto u = GetBarycentricCoords(s.polygon, t->position_);
                    norm_iw = s.normales[0] * u[0] + s.normales[1] * u[1] + s.normales[2] * u[2];
                } else {
                    norm_iw = t->normal_;
                }
            }
        }
    }

    if (!is_assigned) {
        return {0, 0, 0};
    }

    Vector reflect_dir = Reflect(dir, norm_iw);
    reflect_dir.Normalize();
    auto refract_dir = Refract(
        dir, norm_iw, !is_in ? (1.0 / material->refraction_index) : material->refraction_index);

    Vector reflect_color = !is_in ? RayCast(in_work.position_ + norm_iw * kEpsil, reflect_dir,
                                            max_depth, scene, depth + 1)
                                  : Vector(0, 0, 0);
    Vector refract_color(0, 0, 0);
    if (refract_dir.has_value()) {
        auto p = refract_dir.value();
        p.Normalize();
        refract_color = RayCast(in_work.position_ - norm_iw * kEpsil, p, max_depth, scene,
                                depth + 1, is_sphere == !is_in);
    }

    Vector diffuse_light_intensity(0, 0, 0), specular_light_intensity(0, 0, 0);
    for (const Light& light : scene.lights) {
        Vector light_dir = light.position - in_work.position_;
        double dst_to_light = Length(light_dir);
        double dl, sl;
        light_dir.Normalize();
        Ray r_light(in_work.position_ + norm_iw * kEpsil, light_dir);
        for (const SphereObject& s : scene.sphereObjects) {
            auto t = GetIntersection(r_light, s.sphere);
            if (t.has_value()) {
                if (dst_to_light > t.value().distance_) {
                    goto BigNo;
                }
            }
        }
        for (const Object& s : scene.objects) {
            auto t = GetIntersection(r_light, s.polygon);
            if (t.has_value()) {
                if (dst_to_light > t.value().distance_) {
                    goto BigNo;
                }
            }
        }
        dl = std::max(0., DotProduct(light_dir, norm_iw));
        sl = std::pow(std::max(0., DotProduct(Reflect(light_dir * -1, norm_iw) * -1, dir)),
                      material->specular_exponent);
        diffuse_light_intensity[0] += material->diffuse_color[0] * light.intensity[0] * dl;
        diffuse_light_intensity[1] += material->diffuse_color[1] * light.intensity[1] * dl;
        diffuse_light_intensity[2] += material->diffuse_color[2] * light.intensity[2] * dl;
        specular_light_intensity[0] += material->specular_color[0] * light.intensity[0] * sl;
        specular_light_intensity[1] += material->specular_color[1] * light.intensity[1] * sl;
        specular_light_intensity[2] += material->specular_color[2] * light.intensity[2] * sl;
    BigNo:;
    }
    return material->ambient_color + material->intensity +
           diffuse_light_intensity * material->albedo[0] +
           specular_light_intensity * material->albedo[0] + reflect_color * material->albedo[1] +
           refract_color * (!is_in ? material->albedo[2] : 1.0);
}

class Matrix {
public:
    std::array<std::array<double, 3>, 3> x;

    Matrix() {
    }

    Vector MultMatrix(const Vector& src) const {
        double a, b, c;
        a = src[0] * x[0][0] + src[1] * x[1][0] + src[2] * x[2][0];
        b = src[0] * x[0][1] + src[1] * x[1][1] + src[2] * x[2][1];
        c = src[0] * x[0][2] + src[1] * x[1][2] + src[2] * x[2][2];
        Vector ret(a, b, c);
        ret.Normalize();
        return ret;
    }
};

Image Render(const std::filesystem::path& path, const CameraOptions& camera_options,
             const RenderOptions& render_options) {
    Scene scene = ReadScene(path);
    int width = camera_options.screen_width, height = camera_options.screen_height;
    Image q(width, height);
    std::vector<Vector> framebuffer(width * height);
    Matrix camera_to_world;
    {
        Vector forward = camera_options.look_to - camera_options.look_from;
        forward = forward * -1;
        Vector right, up;
        if (forward[0] == 0 && forward[2] == 0) {
            up = Vector(0, 0, -1) * forward[1];
            right = Vector(1, 0, 0) * forward[1];
            forward.Normalize();
            up.Normalize();
            right.Normalize();
        } else {
            forward.Normalize();
            right = CrossProduct({0, 1, 0}, forward);
            right.Normalize();
            up = CrossProduct(forward, right);
            up.Normalize();
        }

        camera_to_world.x[0][0] = right[0], camera_to_world.x[0][1] = right[1],
        camera_to_world.x[0][2] = right[2];
        camera_to_world.x[1][0] = up[0], camera_to_world.x[1][1] = up[1],
        camera_to_world.x[1][2] = up[2];
        camera_to_world.x[2][0] = forward[0], camera_to_world.x[2][1] = forward[1],
        camera_to_world.x[2][2] = forward[2];
    }
    if (render_options.mode == RenderMode::kDepth) {
        double max_dis = -1;
        for (int pix = 0; pix < width * height; pix++) {
            double dir_x = (pix % width + 0.5) - width / 2.;
            double dir_y = -(pix / width + 0.5) + height / 2.;
            double dir_z = -height / (2. * tan(camera_options.fov / 2.));
            double dis = -1;
            bool is_assigned = false;
            Vector w(dir_x, dir_y, dir_z);
            w.Normalize();
            Ray r(camera_options.look_from, camera_to_world.MultMatrix(w));
            for (auto s : scene.sphereObjects) {
                auto t = GetIntersection(r, s.sphere);
                if (t.has_value()) {
                    if (!is_assigned) {
                        dis = t.value().distance_;
                        is_assigned = true;
                    }
                    dis = std::min(dis, t.value().distance_);
                }
            }
            for (auto s : scene.objects) {
                auto t = GetIntersection(r, s.polygon);
                if (t.has_value()) {
                    if (!is_assigned) {
                        dis = t.value().distance_;
                        is_assigned = true;
                    }
                    dis = std::min(dis, t.value().distance_);
                }
            }
            if (is_assigned) {
                max_dis = std::max(max_dis, dis);
            }
            framebuffer[pix] = Vector(dis, dis, dis);
        }
        for (Vector& v : framebuffer) {
            if (v[0] < 0) {
                v[0] = 1;
                v[1] = 1;
                v[2] = 1;
                continue;
            }
            v[0] /= max_dis;
            v[1] /= max_dis;
            v[2] /= max_dis;
        }
        for (int pix = 0; pix < width * height; pix++) {
            RGB p(255 * framebuffer[pix][0], 255 * framebuffer[pix][1], 255 * framebuffer[pix][2]);
            q.SetPixel(p, pix / width, pix % width);
        }
        return q;
    }
    if (render_options.mode == RenderMode::kNormal) {
        for (int pix = 0; pix < width * height; pix++) {
            double dir_x = (pix % width + 0.5) - width / 2.;
            double dir_y = -(pix / width + 0.5) + height / 2.;
            double dir_z = -height / (2. * tan(camera_options.fov / 2.));
            double dis = -1;
            Vector res;
            bool is_assigned = false;
            Vector w(dir_x, dir_y, dir_z);
            w.Normalize();
            Ray r(camera_options.look_from, camera_to_world.MultMatrix(w));
            for (auto s : scene.sphereObjects) {
                auto t = GetIntersection(r, s.sphere);
                if (t.has_value()) {
                    if (!is_assigned) {
                        dis = t.value().distance_;
                        res = t->normal_;
                        is_assigned = true;
                        continue;
                    }
                    if (dis > t.value().distance_) {
                        dis = t.value().distance_;
                        res = t->normal_;
                    }
                }
            }
            for (auto s : scene.objects) {
                auto t = GetIntersection(r, s.polygon);
                if (t.has_value()) {
                    if (!is_assigned) {
                        dis = t.value().distance_;
                        if (s.isSetNorm) {
                            auto u = GetBarycentricCoords(s.polygon, t->position_);
                            res =
                                s.normales[0] * u[0] + s.normales[1] * u[1] + s.normales[2] * u[2];
                        } else {
                            res = t->normal_;
                        }
                        is_assigned = true;
                        continue;
                    }
                    if (dis > t.value().distance_) {
                        dis = t.value().distance_;
                        if (s.isSetNorm) {
                            auto u = GetBarycentricCoords(s.polygon, t->position_);
                            res =
                                s.normales[0] * u[0] + s.normales[1] * u[1] + s.normales[2] * u[2];
                        } else {
                            res = t->normal_;
                        }
                    }
                }
            }
            if (!is_assigned) {
                framebuffer[pix] = Vector(0, 0, 0);
                continue;
            }
            framebuffer[pix] = Vector((res[0] / 2.0) + (1.0 / 2.0), (res[1] / 2.0) + (1.0 / 2.0),
                                      (res[2] / 2.0) + (1.0 / 2.0));
        }
        for (int pix = 0; pix < width * height; pix++) {
            RGB p(255 * framebuffer[pix][0], 255 * framebuffer[pix][1], 255 * framebuffer[pix][2]);
            q.SetPixel(p, pix / width, pix % width);
        }
        return q;
    }
    if (render_options.mode == RenderMode::kFull) {
        bool in_sphere = false;
        for (const SphereObject& s : scene.sphereObjects) {
            if (Length(s.sphere.center_ - camera_options.look_from) < s.sphere.radius_) {
                in_sphere = true;
                break;
            }
        }
        double max_dis = std::numeric_limits<double>::lowest();
        for (int pix = 0; pix < width * height; pix++) {
            double dir_x = (pix % width + 0.5) - width / 2.;
            double dir_y = -(pix / width + 0.5) + height / 2.;
            double dir_z = -height / (2. * tan(camera_options.fov / 2.));
            Vector w(dir_x, dir_y, dir_z);
            w.Normalize();
            Vector res = RayCast(camera_options.look_from, camera_to_world.MultMatrix(w),
                                 render_options.depth, scene, 0, in_sphere);

            max_dis = std::max(max_dis, std::max(res[0], std::max(res[1], res[2])));
            framebuffer[pix] = Vector(res[0], res[1], res[2]);
        }
        max_dis = 1.0 / (max_dis * max_dis);
        for (Vector& v : framebuffer) {
            v[0] = std::pow((v[0] * (1.0 + v[0] * max_dis) / (1.0 + v[0])), 1.0 / 2.2);
            v[1] = std::pow((v[1] * (1.0 + v[1] * max_dis) / (1.0 + v[1])), 1.0 / 2.2);
            v[2] = std::pow((v[2] * (1.0 + v[2] * max_dis) / (1.0 + v[2])), 1.0 / 2.2);
            if (std::isnan(v[0])) {
                v[0] = 0;
            }
            if (std::isnan(v[1])) {
                v[1] = 0;
            }
            if (std::isnan(v[2])) {
                v[2] = 0;
            }
        }
        for (int pix = 0; pix < width * height; pix++) {
            RGB p(255 * framebuffer[pix][0], 255 * framebuffer[pix][1], 255 * framebuffer[pix][2]);
            q.SetPixel(p, pix / width, pix % width);
        }

        return q;
    }

    return q;
}