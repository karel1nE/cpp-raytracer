#include "test_asan.cpp"

TEST_CASE("Dragon") {
    CameraOptions camera_opts{.screen_width = 1024,
                              .screen_height = 768,
                              .fov = std::numbers::pi / 4,
                              .look_from = {-1.1, .6, -.5},
                              .look_to = {.20702, -.26424, .214467}};
    CheckImage("dragon/dragon.obj", "dragon/result.png", camera_opts, {8});
}

TEST_CASE("Figure", "[.]") {
    CameraOptions camera_opts{.screen_width = 1920,
                              .screen_height = 1080,
                              .look_from = {0., 1., 1.3},
                              .look_to = {0., 1., -1.}};
    CheckImage("sculpture/Kayle_sculpt1.OBJ", "sculpture/result.png", camera_opts, {8});
}
