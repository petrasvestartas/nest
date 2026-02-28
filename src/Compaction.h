#pragma once

#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>
#include <limits>

#include "NFP.h"
#include "HelperTypes.h"
#include "NestConfig.h"
#include "GeometryUtil.h"
#include "NfpWorker.h"
#include "SolverConfig.h"
#include "clipper2/clipper.h"

namespace nest {

/// Post-placement slide compaction.
/// After BLF places all pieces, slide each piece toward the origin (left/down/diagonal)
/// using binary search until collision. Reuses NFP/geometry infrastructure.
class Compaction {
public:
    /// Direction for sliding
    struct SlideDirection {
        double dx, dy;
        std::string name;
    };

    /// Compact a set of placed polygons on a sheet.
    /// Modifies placements in-place (updates x, y coordinates).
    /// @param placed The placed NFP polygons (with rotation applied internally)
    /// @param placements The placement items with current x,y positions
    /// @param sheet The sheet boundary
    /// @param config Solver configuration
    /// @return Total distance moved (sum of all slides)
    static double compact(
        std::vector<std::shared_ptr<NFP>>& placed,
        std::vector<PlacementItem>& placements,
        const NFP& sheet,
        const SolverConfig& config);

    /// Compact in a specific direction.
    /// @param direction The slide direction vector (normalized)
    /// @param placed Currently placed polygons
    /// @param placements Current placement positions
    /// @param sheet Sheet boundary
    /// @param processOrder Order in which to process pieces (indices into placements)
    /// @return Total distance moved
    static double compactInDirection(
        const SlideDirection& direction,
        std::vector<std::shared_ptr<NFP>>& placed,
        std::vector<PlacementItem>& placements,
        const NFP& sheet,
        double stepMin);

    /// Slide a single piece in a direction until it collides with another piece or sheet boundary.
    /// Uses binary search (exponential probe + binary refinement).
    /// @param pieceIdx Index of the piece to slide
    /// @param direction Direction to slide
    /// @param placed All placed polygons
    /// @param placements All placement positions
    /// @param sheet Sheet boundary
    /// @return Distance slid
    static double slideUntilCollision(
        int pieceIdx,
        const SlideDirection& direction,
        const std::vector<std::shared_ptr<NFP>>& placed,
        const std::vector<PlacementItem>& placements,
        const NFP& sheet,
        double stepMin);

    /// Standard directions for compaction
    static std::vector<SlideDirection> getStandardDirections();

    /// Check if a piece at a given position overlaps any other placed piece
    static bool hasOverlap(
        int pieceIdx,
        double testX, double testY,
        const std::vector<std::shared_ptr<NFP>>& placed,
        const std::vector<PlacementItem>& placements,
        const NFP& sheet);

    /// Check if a piece at a given position is inside the sheet boundary
    static bool isInsideSheet(
        int pieceIdx,
        double testX, double testY,
        const std::vector<std::shared_ptr<NFP>>& placed,
        const NFP& sheet);

    /// Get the shifted polygon for a piece at a specific position
    static NFP getShiftedPolygon(
        const NFP& piece, double px, double py, float rotation);

    /// Compute overlap between two shifted polygons using Clipper2
    static double computeOverlapArea(const NFP& polyA, const NFP& polyB);
};

} // namespace nest
