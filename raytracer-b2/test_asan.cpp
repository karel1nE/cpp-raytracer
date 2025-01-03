#include "options/camera_options.h"
#include "options/render_options.h"
#include "tests/commons.h"
#include "raytracer.h"
#include "util.h"

#include <cmath>
#include <string_view>
#include <optional>
#include <filesystem>

#include <catch2/catch_test_macros.hpp>

void CheckImage(std::string_view obj_filename, std::string_view result_filename,
                const CameraOptions& camera_options, const RenderOptions& render_options,
                const std::optional<std::filesystem::path>& output_path = std::nullopt) {
    static const auto kTestsDir = GetRelativeDir(__FILE__, "tests");
    auto image = Render(kTestsDir / obj_filename, camera_options, render_options);
    if (output_path) {
        image.Write(*output_path);
    }
    Compare(image, Image{kTestsDir / result_filename});
}

TEST_CASE("Bunny") {
    CameraOptions camera_opts{.screen_width = 1024,
                              .screen_height = 768,
                              .look_from = {-.8, 1., 1.4},
                              .look_to = {-.2, .9, .0}};
    CheckImage("bunny/bunny.obj", "bunny/result.png", camera_opts, {1});
}
