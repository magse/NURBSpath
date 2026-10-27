# Building and installing nurbspath

`nurbspath` 0.4.3 is a dependency-free, header-only C++20 library. CMake builds
the examples and tests, generates repository-aware version metadata, installs
the headers, and exports a package target for consumers.

## Requirements

- CMake 3.20 or newer
- A C++20 compiler such as AppleClang, Clang, GCC, or MSVC
- Git for refreshed commit and worktree metadata; builds still work without it
- Doxygen only when `NURBSPATH_BUILD_DOCUMENTATION=ON`

No third-party runtime or geometry library is required.

## Configure, build, and test

Use an out-of-source build directory from the repository root:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

For development and contributor validation, configure with
`-DCMAKE_BUILD_TYPE=Debug`. The test targets enable `-Wall -Wextra -Wpedantic`
on Clang and GCC, or `/W4 /permissive-` on MSVC.

Build directories contain location-specific CMake caches and should not be
copied between source-tree locations. If a copied cache reports a source-path
mismatch, configure a fresh build directory.

### Run the examples

All examples are built by default. Run the 3D and native 2D geometry examples
and the tagged-data example with:

```sh
./build/nurbspath_example
./build/nurbspath_2d_example
./build/nurbspath_tagged_example
```

The examples print floating-point console output in scientific notation with
at most four digits after the decimal point. Generated SVG examples use the
same precision for numeric drawing attributes.

`nurbspath_tagged_example` prints a rounded preview of geometry with optional
`linecolor`, `linefont`, and quoted `text` rows, together with an
untagged `application` row and positioned `warning`, `information`, and `error`
rows. It retains full round-trip precision in its buffered tagged records and
reads back the eight geometry and presentation rows while skipping these
advisory records. It demonstrates normalized RGBA color, solid and
dashed line settings, and escaped labels and diagnostics. See [DATA.md](DATA.md) for the format and
[README.md](README.md#tagged-3d-persistence-and-optional-presentation-records)
for API usage. Application identity and diagnostics are output-only and optional
for consumers to interpret or display. No extra build option or dependency is
needed for presentation or advisory records.

The SVG example writes `nurbspath_orthographic.svg`,
`nurbspath_perspective.svg`, and `nurbspath_2d.svg` into its current working
directory. These rounded previews contain orthographic and perspective 3D
scenes in the first two files and a native, flat 2D scene in the third. Run it
from a temporary output directory to keep generated diagnostics outside the
repository:

```sh
mkdir -p /tmp/nurbspath-svg
cd /tmp/nurbspath-svg
/path/to/nurbspath/build/nurbspath_svg_example
```

For a multi-configuration generator such as Visual Studio or Xcode, select the
configuration when building and testing:

```sh
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
```

## CMake options

| Option | Default | Purpose |
|---|---:|---|
| `NURBSPATH_BUILD_EXAMPLES` | `ON` | Build the 2D, 3D, SVG, and tagged-data examples |
| `NURBSPATH_BUILD_TESTS` | `ON` | Build the numbered tests and `nurbspath_tests` target |
| `NURBSPATH_BUILD_DOCUMENTATION` | `OFF` | Add the `nurbspath_docs` and `docs` targets |

For a library-only configuration:

```sh
cmake -S . -B build \
  -DNURBSPATH_BUILD_EXAMPLES=OFF \
  -DNURBSPATH_BUILD_TESTS=OFF
cmake --build build
```

## Version metadata

Each CMake configuration reads the repository state and generates
`build/generated/include/nurbspath/config.hpp`. The build-tree include order
selects that header before the checked-in fallback, and installation uses the
generated copy. Consumers can inspect `NURBSPATH_VERSION_*` and
`NURBSPATH_GIT_*` after including `<nurbspath/config.hpp>`.

Before a repository has its first commit, the commit macros report `unborn`
instead of failing configuration. A non-clean worktree appends `-dirty` to
`NURBSPATH_GIT_DESCRIBE` and sets `NURBSPATH_GIT_DIRTY` to one. Re-run CMake
after a commit, tag, or worktree-state change to refresh the generated values.

Direct use of the source `include` directory selects the checked-in
`config.hpp`. That fallback reports release metadata and sets
`NURBSPATH_CONFIG_GENERATED` to zero. The CMake-generated and installed copy
sets the macro to one.

## Generate API documentation

Documentation is opt-in, so normal consumers do not need Doxygen:

```sh
cmake -S . -B build-docs \
  -DNURBSPATH_BUILD_DOCUMENTATION=ON \
  -DNURBSPATH_BUILD_TESTS=OFF \
  -DNURBSPATH_BUILD_EXAMPLES=OFF
cmake --build build-docs --target docs
```

Open `build-docs/docs/html/index.html` after generation. Documentation warnings
are treated as errors, so incomplete public API comments fail the target.

### Illustrated PDF manual

Read the [illustrated PDF manual](https://github.com/magse/NURBSpath/blob/main/manual/nurbspath-manual.pdf) for entity
construction, algorithm examples, and the public API catalogue. To rebuild it
from the repository root with an already installed C++20 compiler and LaTeX:

```sh
sh manual/build.sh
```

The manual builds independently of CMake and downloads nothing. See the
[manual build instructions](https://github.com/magse/NURBSpath/blob/main/manual/README.md) for the required local TeX
packages, source layout, and optional introduction and closing files.

## Install

Configure a library-only release build, then choose the installation prefix at
install time:

```sh
cmake -S . -B build-install \
  -DCMAKE_BUILD_TYPE=Release \
  -DNURBSPATH_BUILD_EXAMPLES=OFF \
  -DNURBSPATH_BUILD_TESTS=OFF
cmake --build build-install --parallel
cmake --install build-install --prefix /your/prefix
```

The installation contains:

- public headers under `/your/prefix/include/nurbspath`, including
  `tagged_attributes.hpp` for line settings and text labels,
  `tagged_annotations.hpp` for output-only application identity and positioned
  diagnostics, and `tagged_serialization.hpp` for mixed-record input;
- CMake package files under the platform's CMake library directory, commonly
  `/your/prefix/lib/cmake/nurbspath`.

No object archive or shared library is installed. The exported
`nurbspath::nurbspath` target is an `INTERFACE` target that supplies the include
directory and C++20 requirement.

## Consume an installed package

After installation, use the exported target from another CMake project:

```cmake
cmake_minimum_required(VERSION 3.20)
project(my_geometry_app LANGUAGES CXX)

find_package(nurbspath 0.4.3 CONFIG REQUIRED)

add_executable(my_geometry_app main.cpp)
target_link_libraries(my_geometry_app PRIVATE nurbspath::nurbspath)
```

If the prefix is outside CMake's normal search path, pass it when configuring
the consumer:

```sh
cmake -S . -B build -DCMAKE_PREFIX_PATH=/your/prefix
```

For an in-tree dependency, use `add_subdirectory` instead:

```cmake
add_subdirectory(path/to/nurbspath)
target_link_libraries(my_geometry_app PRIVATE nurbspath::nurbspath)
```

Linking the interface target requests C++20 and adds the correct include
directory automatically.

## Use without CMake

Because the library is header-only, adding the repository's `include`
directory to a C++20 compile is sufficient:

```sh
c++ -std=c++20 -I/path/to/nurbspath/include main.cpp -o my_geometry_app
```

This direct-include workflow uses the checked-in fallback version metadata.
