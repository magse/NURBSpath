#include <nurbspath/config.hpp>

#include "test_support.hpp"

#include <string>
#include <string_view>

int main() {
    using namespace test_support;
    using namespace std::string_view_literals;

    static_assert(NURBSPATH_VERSION_MAJOR == 0);
    static_assert(NURBSPATH_VERSION_MINOR == 1);
    static_assert(NURBSPATH_VERSION_PATCH == 2);
    static_assert(NURBSPATH_VERSION_NUMBER == 102);
    static_assert(NURBSPATH_CONFIG_GENERATED == 1);

    check(NURBSPATH_VERSION_STRING == "0.1.2"sv,
          "semantic version string matches CMake project version");
    check(NURBSPATH_GIT_COMMIT[0] != '\0', "Git commit identity is not empty");
    check(NURBSPATH_GIT_COMMIT_SHORT[0] != '\0',
          "short Git commit identity is not empty");
    check(NURBSPATH_GIT_DESCRIBE[0] != '\0',
          "Git description is not empty");
    check(NURBSPATH_GIT_DIRTY == 0 || NURBSPATH_GIT_DIRTY == 1,
          "Git dirty state is Boolean");
    check(NURBSPATH_GIT_COMMIT_AVAILABLE == 0 ||
              NURBSPATH_GIT_COMMIT_AVAILABLE == 1,
          "Git commit availability is Boolean");

    const std::string expected_git_version =
        std::string(NURBSPATH_VERSION_STRING) + "+" + NURBSPATH_GIT_DESCRIBE;
    check(NURBSPATH_GIT_VERSION == expected_git_version,
          "combined Git version contains semantic and repository versions");
    if (NURBSPATH_GIT_DIRTY == 1) {
        check(std::string_view(NURBSPATH_GIT_DESCRIBE).ends_with("-dirty"),
              "dirty Git descriptions have a dirty suffix");
    }
    if (NURBSPATH_GIT_COMMIT_AVAILABLE == 0) {
        check(NURBSPATH_GIT_COMMIT == "unborn"sv &&
                  NURBSPATH_GIT_COMMIT_SHORT == "unborn"sv,
              "a repository without HEAD reports an unborn commit");
    }

    return finish("16_test_config");
}
