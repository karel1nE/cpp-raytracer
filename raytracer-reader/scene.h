#pragma once

#include "material.h"
#include "vector.h"
#include "object.h"
#include "light.h"

#include <vector>
#include <unordered_map>
#include <string>
#include <filesystem>
#include <fstream>
#include <iostream>

std::vector<double> ReadDoubles(std::istream& is, size_t count) {
    std::vector<double> ans;
    for (size_t i = 0; i < count; ++i) {
        std::string token;
        is >> token;
        ans.push_back(std::stod(token));
    }
    return ans;
}

class Scene {
public:
    const std::vector<Object>& GetObjects() const {
        return objects;
    }
    const std::vector<SphereObject>& GetSphereObjects() const {
        return spheres;
    }
    const std::vector<Light>& GetLights() const {
        return lights;
    }
    const std::unordered_map<std::string, Material>& GetMaterials() const {
        return materials;
    }

    std::vector<Object> objects;
    std::vector<SphereObject> spheres;
    std::vector<Light> lights;
    std::unordered_map<std::string, Material> materials;
};

std::unordered_map<std::string, Material> ReadMaterials(const std::filesystem::path& path) {
    std::ifstream is{path};
    std::unordered_map<std::string, Material> ans;
    Material cur_material;
    while (!is.eof()) {
        std::string line;
        std::getline(is, line);

        std::stringstream sstream(line);

        std::string token;
        sstream >> token;

        if (token == "newmtl") {
            if (!cur_material.name.empty()) {
                ans[cur_material.name] = cur_material;
            }
            cur_material = Material();
            sstream >> cur_material.name;
        } else if (token == "Ka") {
            std::vector<double> v = ReadDoubles(sstream, 3);
            cur_material.ambient_color = {v[0], v[1], v[2]};
        } else if (token == "Kd") {
            std::vector<double> v = ReadDoubles(sstream, 3);
            cur_material.diffuse_color = {v[0], v[1], v[2]};
        } else if (token == "Ks") {
            std::vector<double> v = ReadDoubles(sstream, 3);
            cur_material.specular_color = {v[0], v[1], v[2]};
        } else if (token == "Ke") {
            std::vector<double> v = ReadDoubles(sstream, 3);
            cur_material.intensity = {v[0], v[1], v[2]};
        } else if (token == "Ns") {
            std::vector<double> v = ReadDoubles(sstream, 1);
            cur_material.specular_exponent = v[0];
        } else if (token == "Ni") {
            std::vector<double> v = ReadDoubles(sstream, 1);
            cur_material.refraction_index = v[0];
        } else if (token == "al") {
            std::vector<double> v = ReadDoubles(sstream, 3);
            cur_material.albedo = {v[0], v[1], v[2]};
        }
    }
    ans[cur_material.name] = cur_material;
    return ans;
}

size_t TranslateIndex(int ind, size_t total_size) {
    int int_token = ind;
    if (int_token >= 0) {
        --int_token;
    } else {
        int_token = static_cast<int>(total_size) + int_token;
    }
    return static_cast<size_t>(int_token);
}

Scene ReadScene(const std::filesystem::path& path) {
    std::ifstream is{path};
    Scene scene;
    Material* material = nullptr;
    std::vector<Vector> vertexes;
    std::vector<Vector> vertex_normals;

    while (!is.eof()) {
        std::string line;
        std::getline(is, line);
        if (line[0] == '#') {
            continue;
        }
        std::stringstream sstream(line);
        std::string token;
        sstream >> token;
        if (token == "S") {
            auto v = ReadDoubles(sstream, 4);
            SphereObject sphere = {material, Sphere(Vector(v[0], v[1], v[2]), v[3])};
            scene.spheres.push_back(sphere);
        } else if (token == "v") {
            auto v = ReadDoubles(sstream, 3);
            vertexes.push_back({v[0], v[1], v[2]});
        } else if (token == "vn") {
            auto v = ReadDoubles(sstream, 3);
            vertex_normals.push_back({v[0], v[1], v[2]});
        } else if (token == "usemtl") {
            std::string material_name;
            sstream >> material_name;
            material = &scene.materials[material_name];
        } else if (token == "mtllib") {
            std::filesystem::path mtlpath;
            std::string mtlfile;
            sstream >> mtlfile;
            mtlpath = path.parent_path() / mtlfile;
            auto new_materials = ReadMaterials(mtlpath);
            for (auto [k, v] : new_materials) {
                scene.materials[k] = v;
            }
        } else if (token == "P") {
            auto v = ReadDoubles(sstream, 6);
            Light l;
            l.position = {v[0], v[1], v[2]};
            l.intensity = {v[3], v[4], v[5]};
            scene.lights.push_back(l);
        } else if (token == "f") {
            std::vector<std::string> f_tokens;
            while (!sstream.eof()) {
                std::string next_v;
                sstream >> next_v;
                f_tokens.push_back(next_v);
            }
            std::vector<size_t> ind_vertexes, ind_norms;

            for (auto& f_token : f_tokens) {
                if (f_token.empty()) {
                    continue;
                }
                int first_slash = f_token.find('/');
                int second_slash = f_token.rfind('/');
                int first_ind = std::stoi(f_token.substr(0, first_slash));
                int second_ind = std::stoi(f_token.substr(second_slash + 1));
                ind_vertexes.push_back(TranslateIndex(first_ind, vertexes.size()));
                ind_norms.push_back(TranslateIndex(second_ind, vertex_normals.size()));
            }
            size_t i, j, k;
            for (size_t ind = 1; ind + 1 < ind_vertexes.size(); ++ind) {
                i = ind_vertexes[0], j = ind_vertexes[ind], k = ind_vertexes[ind + 1];
                Triangle polygon = {vertexes[i], vertexes[j], vertexes[k]};
                i = ind_norms[0], j = ind_norms[ind], k = ind_norms[ind + 1];
                std::array<Vector, 3> normals;
                bool has_norm = 0;

                if (k < vertex_normals.size()) {
                    normals = {vertex_normals[i], vertex_normals[j], vertex_normals[k]};
                    has_norm = 1;
                }

                Object obj = {material, polygon, normals};
                obj.has_norm = has_norm;
                scene.objects.push_back(obj);
            }
        }
    }

    return scene;
}
