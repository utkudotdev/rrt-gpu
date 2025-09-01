project(
    'rrt-gpu',
    'cpp',
    version: '0.1.0',
    default_options: ['cpp_std=c++23', 'warning_level=3'],
)

# Dependencies
eigen3_dep = dependency('eigen3', required: true)
sfml_graphics_dep = dependency('sfml-graphics', required: true)
sfml_window_dep = dependency('sfml-window', required: true)
sfml_system_dep = dependency('sfml-system', required: true)
benchmark_dep = dependency('benchmark', required: true)
doctest_inc = include_directories('deps/doctest/doctest')

src_inc = include_directories('src')

# Main executable
main_sources = files(
    'src/algo/dfs.cpp',
    'src/algo/raytrace.cpp',
    'src/ds/occupancy_grid.cpp',
    'src/main.cpp',
)
main_deps = [eigen3_dep, sfml_graphics_dep, sfml_window_dep, sfml_system_dep]

executable(
    'main',
    main_sources,
    include_directories: [src_inc],
    dependencies: main_deps,
    install: true,
)

# Test executable
test_sources = files(
    'src/algo/raytrace.cpp',
    'src/ds/occupancy_grid.cpp',
    'test/test.cpp',
    'test/test_grid.cpp',
    'test/test_kdtree.cpp',
    'test/test_raytrace.cpp',
)
test_deps = [eigen3_dep]

test_exec = executable(
    'test_exec',
    test_sources,
    dependencies: test_deps,
    include_directories: [src_inc, doctest_inc],
    install: false,
)

test('unit tests', test_exec)

# Benchmark executable
bench_sources = files('bench/bench_kdtree.cpp')
bench_deps = [eigen3_dep, benchmark_dep]

executable(
    'bench',
    bench_sources,
    include_directories: [src_inc],
    dependencies: bench_deps,
    install: true,
)