#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include "clipper2/clipper.h"

namespace nest {

/// Minkowski convolution (NFP computation) using edge-pair algorithm.
/// Ported from the Boost.Polygon-based minkowski.dll implementation.
/// Uses Clipper2 for the final boolean union instead of Boost.Polygon.
class MinkowskiConvolution {
public:
    struct Result {
        std::vector<std::vector<double>> outerPaths;
        std::vector<std::vector<double>> holes;
    };

    /// Fixed scale matching getOuterNfp's scale for coordinate consistency.
    static constexpr double kScale = 10000000.0;

    static Result compute(
        const std::vector<double>& Apts,
        const std::vector<std::vector<double>>& Aholes,
        const std::vector<double>& Bpts)
    {
        using namespace Clipper2Lib;

        double inputscale = kScale;
        Path64 aOuterPath = scaleToInt(Apts, inputscale);

        double xshift = (Bpts.size() >= 2) ? Bpts[0] : 0.0;
        double yshift = (Bpts.size() >= 2) ? Bpts[1] : 0.0;

        // Scale B negated to int64
        Path64 bNeg;
        bNeg.reserve(Bpts.size() / 2);
        for (size_t i = 0; i + 1 < Bpts.size(); i += 2) {
            bNeg.push_back(Point64(
                static_cast<int64_t>(inputscale * (-Bpts[i])),
                static_cast<int64_t>(inputscale * (-Bpts[i + 1]))));
        }

        // Build polygon set A: directly construct polygon-with-holes
        // (matching Boost's: a += outer; a -= hole1; a -= hole2; ...)
        // Must preserve outer/hole topology — Clipper2 Difference would
        // resolve to simple polygons, losing the hole boundaries that the
        // edge-pair convolution needs.
        // Normalize winding: outer positive, holes negative.
        std::vector<PolyWithHoles> aPolys;
        {
            Path64 outer = aOuterPath;
            if (!IsPositive(outer)) std::reverse(outer.begin(), outer.end());

            std::vector<Path64> holes;
            for (const auto& hole : Aholes) {
                Path64 h = scaleToInt(hole, inputscale);
                if (IsPositive(h)) std::reverse(h.begin(), h.end());
                holes.push_back(std::move(h));
            }

            aPolys.push_back({std::move(outer), std::move(holes)});
        }

        // Build polygon set B (just negated B, no holes)
        // Normalize winding: outer positive.
        std::vector<PolyWithHoles> bPolys;
        {
            Path64 bOuter = bNeg;
            if (!IsPositive(bOuter)) std::reverse(bOuter.begin(), bOuter.end());
            bPolys.push_back({std::move(bOuter), {}});
        }

        // Run edge-pair convolution
        Paths64 allPaths;
        convolve_two_polygon_sets(allPaths, aPolys, bPolys);

        // Final union with NonZero fill
        PolyTree64 resultTree;
        {
            Clipper64 c;
            c.AddSubject(allPaths);
            c.Execute(ClipType::Union, FillRule::NonZero, resultTree);
        }

        Result result;
        extractResult(resultTree, inputscale, xshift, yshift, result);
        return result;
    }

private:
    struct PolyWithHoles {
        Clipper2Lib::Path64 outer;
        std::vector<Clipper2Lib::Path64> holes;
    };

    static Clipper2Lib::Path64 scaleToInt(const std::vector<double>& pts, double scale) {
        Clipper2Lib::Path64 path;
        path.reserve(pts.size() / 2);
        for (size_t i = 0; i + 1 < pts.size(); i += 2) {
            path.push_back(Clipper2Lib::Point64(
                static_cast<int64_t>(scale * pts[i]),
                static_cast<int64_t>(scale * pts[i + 1])));
        }
        return path;
    }

    /// Create parallelogram from two directed edges (edge-pair convolution kernel).
    /// Matches Boost's convolve_two_segments: quad = [a1+b2, a1+b1, a2+b1, a2+b2].
    /// Normalize to positive (CCW) winding so NonZero union merges correctly.
    /// Without this, anti-parallel edge pairs produce CW quads that cancel
    /// overlapping CCW quads, creating gaps in the Minkowski sum.
    static Clipper2Lib::Path64 convolve_two_segments(
        const Clipper2Lib::Point64& a1, const Clipper2Lib::Point64& a2,
        const Clipper2Lib::Point64& b1, const Clipper2Lib::Point64& b2)
    {
        Clipper2Lib::Path64 quad = {
            Clipper2Lib::Point64(a1.x + b2.x, a1.y + b2.y),
            Clipper2Lib::Point64(a1.x + b1.x, a1.y + b1.y),
            Clipper2Lib::Point64(a2.x + b1.x, a2.y + b1.y),
            Clipper2Lib::Point64(a2.x + b2.x, a2.y + b2.y)
        };
        if (!Clipper2Lib::IsPositive(quad))
            std::reverse(quad.begin(), quad.end());
        return quad;
    }

    /// Convolve all edge pairs from two point sequences.
    /// For each edge in seqA × each edge in seqB: create quad.
    /// Clipper2 Path64 stores OPEN paths (N vertices, N edges with wrap-around),
    /// so we use modular indexing to include the closing edge (last→first).
    static void convolve_two_point_sequences(
        Clipper2Lib::Paths64& result,
        const Clipper2Lib::Path64& seqA,
        const Clipper2Lib::Path64& seqB)
    {
        const size_t nA = seqA.size();
        const size_t nB = seqB.size();
        if (nA < 2 || nB < 2) return;

        for (size_t ia = 0; ia < nA; ++ia) {
            for (size_t ib = 0; ib < nB; ++ib) {
                result.push_back(convolve_two_segments(
                    seqB[ib], seqB[(ib + 1) % nB],
                    seqA[ia], seqA[(ia + 1) % nA]));
            }
        }
    }

    /// Convolve a point sequence with all boundaries of a polygon set.
    static void convolve_point_sequence_with_polygons(
        Clipper2Lib::Paths64& result,
        const Clipper2Lib::Path64& seq,
        const std::vector<PolyWithHoles>& polygons)
    {
        for (const auto& poly : polygons) {
            convolve_two_point_sequences(result, seq, poly.outer);
            for (const auto& hole : poly.holes) {
                convolve_two_point_sequences(result, seq, hole);
            }
        }
    }

    /// Top-level edge-pair convolution of two polygon sets.
    /// Direct port of Boost's convolve_two_polygon_sets.
    static void convolve_two_polygon_sets(
        Clipper2Lib::Paths64& result,
        const std::vector<PolyWithHoles>& aPolys,
        const std::vector<PolyWithHoles>& bPolys)
    {
        result.clear();

        for (size_t ai = 0; ai < aPolys.size(); ++ai) {
            // Convolve A[ai]'s outer boundary with all B polygons
            convolve_point_sequence_with_polygons(result, aPolys[ai].outer, bPolys);

            // Convolve each hole of A[ai] with all B polygons
            for (const auto& hole : aPolys[ai].holes) {
                convolve_point_sequence_with_polygons(result, hole, bPolys);
            }

            // Add translated copies
            for (size_t bi = 0; bi < bPolys.size(); ++bi) {
                // Translate A[ai] (outer + holes) by first vertex of B[bi]
                if (!bPolys[bi].outer.empty()) {
                    auto bFirst = bPolys[bi].outer[0];
                    result.push_back(translatePath(aPolys[ai].outer, bFirst));
                    for (const auto& hole : aPolys[ai].holes) {
                        result.push_back(translatePath(hole, bFirst));
                    }
                }

                // Translate B[bi] (outer + holes) by first vertex of A[ai]
                if (!aPolys[ai].outer.empty()) {
                    auto aFirst = aPolys[ai].outer[0];
                    result.push_back(translatePath(bPolys[bi].outer, aFirst));
                    for (const auto& hole : bPolys[bi].holes) {
                        result.push_back(translatePath(hole, aFirst));
                    }
                }
            }
        }
    }

    /// Translate a path by a point offset.
    static Clipper2Lib::Path64 translatePath(
        const Clipper2Lib::Path64& path,
        const Clipper2Lib::Point64& offset)
    {
        Clipper2Lib::Path64 result;
        result.reserve(path.size());
        for (const auto& pt : path) {
            result.push_back(Clipper2Lib::Point64(pt.x + offset.x, pt.y + offset.y));
        }
        return result;
    }

    static void extractResult(
        const Clipper2Lib::PolyTree64& tree,
        double scale, double xshift, double yshift,
        Result& result)
    {
        for (const auto& outerNode : tree) {
            const auto& poly = outerNode->Polygon();
            std::vector<double> pts;
            pts.reserve(poly.size() * 2);
            for (const auto& pt : poly) {
                pts.push_back(static_cast<double>(pt.x) / scale + xshift);
                pts.push_back(static_cast<double>(pt.y) / scale + yshift);
            }
            result.outerPaths.push_back(std::move(pts));

            for (size_t j = 0; j < outerNode->Count(); j++) {
                const auto& holePoly = (*outerNode)[j]->Polygon();
                std::vector<double> holePts;
                holePts.reserve(holePoly.size() * 2);
                for (const auto& pt : holePoly) {
                    holePts.push_back(static_cast<double>(pt.x) / scale + xshift);
                    holePts.push_back(static_cast<double>(pt.y) / scale + yshift);
                }
                result.holes.push_back(std::move(holePts));
            }
        }
    }
};

} // namespace nest
