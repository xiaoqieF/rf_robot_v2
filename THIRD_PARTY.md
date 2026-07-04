# Third-party sources

This workspace uses git submodules for the Cartographer source and Abseil,
while GoogleTest can be provided by the system packages installed on the
machine:

- `src/cartographer` from `https://github.com/cartographer-project/cartographer.git`
- `third-party/abseil-cpp` from `https://github.com/abseil/abseil-cpp.git`

## Initialize

After cloning the repository, run:

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

The first command makes sure local submodule URLs match `.gitmodules`.
The second command fetches and checks out the submodules tracked here.

The workspace expects these system packages to be installed separately:

- `libgtest-dev`
- `libgmock-dev`
