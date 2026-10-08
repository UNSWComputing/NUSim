#!/usr/bin/env python3
"""Generate a role's main.cpp from a list of module short-names.

Usage: generate_role.py <out.cpp> <source_root> <Module1> <Module2> ...

Each ModuleX becomes `#include "module/ModuleX/src/ModuleX.hpp"` and
`plant.install<k1sim::module::ModuleX>()`, in the order given. ChronoController is
installed first (NUClear does not auto-install it; without it no Every<> fires).
List ConsoleLog first in the .role file so it captures other modules' startup logs.
"""
import os
import sys

out_path = sys.argv[1]
source_root = sys.argv[2]
modules = sys.argv[3:]

lines = [
    "#include <csignal>",
    "#include <nuclear>",
    "",
    '#include "shared/CliOptions.hpp"',
    '#include "shared/gl/XThreads.hpp"',
]

for m in modules:
    header = f"module/{m}/src/{m}.hpp"
    if not os.path.isfile(os.path.join(source_root, header)):
        raise SystemExit(f"generate_role.py: cannot find header {header} for module {m}")
    lines.append(f'#include "{header}"')

lines += [
    "",
    "namespace {",
    "void handle_signal(int /*signum*/) {",
    "    if (NUClear::PowerPlant::powerplant != nullptr) {",
    "        NUClear::PowerPlant::powerplant->shutdown();",
    "    }",
    "}",
    "}  // namespace",
    "",
    "int main(int argc, char** argv) {",
    "    k1sim::init_x_threads();  // must precede any GL/X11 use (Viewer + Camera render threads)",
    "    k1sim::cli() = k1sim::parse_cli(argc, argv);",
    "",
    "    NUClear::Configuration config;",
    "    config.default_pool_concurrency = 4;",
    "    NUClear::PowerPlant plant(config);",
    "",
    "    plant.install<NUClear::extension::ChronoController>();",
    "    plant.install<NUClear::extension::IOController>();",
]
for m in modules:
    lines.append(f"    plant.install<k1sim::module::{m}>();")
lines += [
    "",
    "    std::signal(SIGINT, handle_signal);",
    "    std::signal(SIGTERM, handle_signal);",
    "",
    "    plant.start();",
    "    return 0;",
    "}",
    "",
]

with open(out_path, "w", encoding="utf-8") as f:
    f.write("\n".join(lines))
