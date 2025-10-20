mc_rtc GO1 robot module
==

This project is a template for a new robot module project wihtin [mc_rtc]

It comes with:
- a CMake project that can build a controller in [mc_rtc], the project can be put within [mc_rtc] source-tree for easier updates
- clang-format files
- automated GitHub Actions builds on three major platforms

This repository actually creates two robot modules:
- `Go1` is a C++ robot module in the [src](src) folder
- `Go1YAML` is a YAML robot module in the [yaml](yaml) folder

They both re-create the JVRC1 robot module shipped with [mc_rtc]

Quick start
--

1. Renaming the robot module from `Go1` to `Go1`. In a shell (Git Bash on Windows, replace sed with gsed on macOS):

```bash
sed -i -e's/Go1/Go1/g' `find . -type f`
```

2. If you choose to build a C++ robot module

```bash
git rm -rf yaml
sed -i -e's/add_subdirectory(yaml)//' CMakeLists.txt
git mv src/Go1.in.cpp src/Go1.in.cpp
git mv src/Go1.h src/Go1.h
```

3. If you choose to build a YAML robot module

```bash
git rm -rf src
sed -i -e's/add_subdirectory(src)//' CMakeLists.txt
```

4. You can customize the project name in vcpkg.json as well, note that this must follow [vcpkg manifest rules](https://github.com/microsoft/vcpkg/blob/master/docs/users/manifests.md)

5. Build and install the project

6. Run using your [mc_rtc] interface of choice, and setting `MainRobot` to `Go1`

[mc_rtc]: https://jrl-umi3218.github.io/mc_rtc/
