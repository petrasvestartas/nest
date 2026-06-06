# nest

Self-contained C++ 2D irregular-nesting engines for [OpenNest](https://github.com/petrasvestartas/OpenNest),
consumed as a git submodule by [compas_nest](https://github.com/petrasvestartas/compas_nest).

| Directory | Engine | C ABI | Dependencies |
|---|---|---|---|
| `nest_physics_cpp/` | physics / overlap-relaxation | `np_nest` (`nest_physics_capi.h`) | none (header-only solver) |
| `opennest_cpp/` | NFP + genetic algorithm | `nfp_nest` (`src/capi/nfp_nest_capi.h`) | bundled Clipper2 + minimal Boost (`third_party/boost_min`) |

Both are fully self-contained — no CGAL / Boost / Eigen download is required.
