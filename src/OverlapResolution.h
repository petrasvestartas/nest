#pragma once

#include <vector>
#include <memory>
#include <unordered_map>
#include <random>
#include <chrono>
#include <functional>

#include "NFP.h"
#include "HelperTypes.h"
#include "SolverConfig.h"
#include "GeometryUtil.h"
#include "Compaction.h"
#include "clipper2/clipper.h"

namespace nest {

/// Sparrow-inspired two-phase overlap resolution solver.
/// Phase 1 (Exploration): Shrink strip, allow overlaps, resolve via sampling + coordinate descent.
/// Phase 2 (Compression): From best feasible, progressively shrink with finer steps.
class OverlapResolution {
public:
    /// Rectangular sheet region in the virtual multi-sheet layout
    struct SheetRegion {
        double x, y, width, height;
        int index;
    };

    /// Placement state for a single item
    struct ItemState {
        int partIndex;
        double x, y;
        float rotation;
        std::shared_ptr<NFP> polygon;   // Original (unrotated) polygon
        int sheetIdx = 0;               // Assigned sheet (for multi-sheet)
    };

    /// Solution: a complete set of item placements
    struct Solution {
        std::vector<ItemState> items;
        double stripWidth;
        double totalLoss;       // Total overlap (0 = feasible)
        double weightedLoss;    // GLS-weighted overlap
        bool feasible;
    };

    /// Coordinate descent axis
    enum class CDAxis { Horizontal, Vertical, ForwardDiag, BackwardDiag, Rotation };

    /// GLS collision weight for an item pair
    struct CollisionWeight {
        double weight = 1.0;
        double loss = 0.0;
    };

    /// Spatial hash grid for O(1) overlap pair detection
    class SpatialGrid {
    public:
        SpatialGrid() = default;
        void build(const std::vector<ItemState>& items, double cellSize);
        std::vector<int> getNeighbors(int itemIdx) const;
        void clear();

    private:
        struct Cell { std::vector<int> items; };
        std::unordered_map<int64_t, Cell> cells;
        double cellSize_ = 100.0;
        double offsetX_ = 0, offsetY_ = 0;

        int64_t cellKey(double x, double y) const;
    };

    /// Run the overlap resolution solver (single sheet, strip packing).
    static Solution solve(
        const std::vector<std::shared_ptr<NFP>>& parts,
        const NFP& sheet,
        const SolverConfig& config,
        std::function<void(int, double)> progressCallback = nullptr);

    /// Run multi-sheet overlap resolution solver (bin packing).
    /// Distributes parts across K sheets, minimizes sheet count.
    /// @param parts Parts to nest (unrotated)
    /// @param sheetTemplate Template sheet (defines W, H for each copy)
    /// @param maxSheets Maximum number of sheets to use
    /// @param config Solver configuration
    /// @param progressCallback Optional callback(sheetsUsed, iteration, totalLoss)
    /// @return Best feasible solution found, with items placed in absolute coordinates
    static Solution solveMultiSheet(
        const std::vector<std::shared_ptr<NFP>>& parts,
        const NFP& sheetTemplate,
        int maxSheets,
        const SolverConfig& config,
        std::function<void(int, int, double)> progressCallback = nullptr);

    /// Create a layout of K sheets side by side
    static std::vector<SheetRegion> createSheetLayout(
        const NFP& sheetTemplate, int numSheets, double gap = 100.0);

    /// Determine which sheet region an item is closest to
    static int nearestSheet(const ItemState& item,
                            const std::vector<SheetRegion>& regions);

private:
    /// Initialize items with BLF or random placement
    static Solution initializeSolution(
        const std::vector<std::shared_ptr<NFP>>& parts,
        const NFP& sheet,
        std::mt19937& rng);

    /// Compute total overlap for a solution
    static void evaluateSolution(Solution& sol, const NFP& sheet,
                                  const std::unordered_map<int64_t, CollisionWeight>& weights);

    /// Compute overlap between two placed items using Clipper2
    static double computePairOverlap(const ItemState& a, const ItemState& b);

    /// Check if item is inside sheet boundary
    static double computeSheetViolation(const ItemState& item, const NFP& sheet);

    /// Get the transformed polygon for an item
    static NFP getTransformedPoly(const ItemState& item);

    /// Separation: iteratively move colliding items to reduce overlap
    static bool separate(
        Solution& sol,
        const NFP& sheet,
        std::unordered_map<int64_t, CollisionWeight>& weights,
        SpatialGrid& grid,
        const SolverConfig& config,
        std::mt19937& rng,
        int strikeLimit,
        int iterNoImproveLimit);

    /// Move a single colliding item to a better position
    static bool moveItem(
        Solution& sol,
        int itemIdx,
        const NFP& sheet,
        const std::unordered_map<int64_t, CollisionWeight>& weights,
        const SpatialGrid& grid,
        const SolverConfig& config,
        std::mt19937& rng);

    /// Sample candidate positions for an item
    static std::vector<ItemState> samplePositions(
        const Solution& sol,
        int itemIdx,
        const NFP& sheet,
        int containerSamples,
        int focusedSamples,
        std::mt19937& rng);

    /// Coordinate descent refinement
    static ItemState coordinateDescent(
        const Solution& sol,
        int itemIdx,
        const ItemState& start,
        const NFP& sheet,
        const std::unordered_map<int64_t, CollisionWeight>& weights,
        const SpatialGrid& grid,
        double stepInit,
        double stepLimit,
        double rotStepInit = 5.0,
        double rotStepLimit = 0.5);

    /// Evaluate loss for a single item at a position
    static double evaluateItemLoss(
        const Solution& sol,
        int itemIdx,
        const ItemState& candidate,
        const NFP& sheet,
        const std::unordered_map<int64_t, CollisionWeight>& weights,
        const SpatialGrid& grid);

    /// Update GLS weights based on current collisions
    static void updateGLSWeights(
        const Solution& sol,
        const NFP& sheet,
        std::unordered_map<int64_t, CollisionWeight>& weights,
        const SolverConfig& config);

    /// Disrupt: swap two large items
    static void disruptSolution(
        Solution& sol,
        const SolverConfig& config,
        std::mt19937& rng);

    /// Shrink strip width
    static void shrinkStrip(
        Solution& sol,
        double shrinkRatio,
        std::mt19937& rng);

    /// Make pair key from two item indices
    static int64_t pairKey(int a, int b);

    // === Multi-sheet variants ===

    /// Compute sheet violation against multiple disjoint regions
    static double computeMultiRegionViolation(
        const ItemState& item, const std::vector<SheetRegion>& regions);

    /// Evaluate full solution against multiple regions
    static void evaluateMultiSheetSolution(
        Solution& sol, const std::vector<SheetRegion>& regions,
        const std::unordered_map<int64_t, CollisionWeight>& weights);

    /// Evaluate single item loss against multiple regions
    static double evaluateMultiSheetItemLoss(
        const Solution& sol, int itemIdx, const ItemState& candidate,
        const std::vector<SheetRegion>& regions,
        const std::unordered_map<int64_t, CollisionWeight>& weights,
        const SpatialGrid& grid);

    /// Initialize items distributed across multiple sheet regions
    static Solution initializeMultiSheetSolution(
        const std::vector<std::shared_ptr<NFP>>& parts,
        const std::vector<SheetRegion>& regions,
        std::mt19937& rng);

    /// Sample candidate positions across all valid sheet regions
    static std::vector<ItemState> sampleMultiSheetPositions(
        const Solution& sol, int itemIdx,
        const std::vector<SheetRegion>& regions,
        int containerSamples, int focusedSamples,
        std::mt19937& rng);

    /// Coordinate descent with multi-region awareness
    static ItemState coordinateDescentMS(
        const Solution& sol, int itemIdx, const ItemState& start,
        const std::vector<SheetRegion>& regions,
        const std::unordered_map<int64_t, CollisionWeight>& weights,
        const SpatialGrid& grid,
        double stepInit, double stepLimit,
        double rotStepInit = 5.0, double rotStepLimit = 0.5);

    /// Move item with multi-region awareness
    static bool moveItemMS(
        Solution& sol, int itemIdx,
        const std::vector<SheetRegion>& regions,
        const std::unordered_map<int64_t, CollisionWeight>& weights,
        const SpatialGrid& grid,
        const SolverConfig& config,
        std::mt19937& rng);

    /// Separation loop with multi-region awareness
    static bool separateMS(
        Solution& sol,
        const std::vector<SheetRegion>& regions,
        std::unordered_map<int64_t, CollisionWeight>& weights,
        SpatialGrid& grid,
        const SolverConfig& config,
        std::mt19937& rng,
        int strikeLimit,
        int iterNoImproveLimit);

    /// Update GLS weights (multi-sheet version)
    static void updateGLSWeightsMS(
        const Solution& sol,
        const std::vector<SheetRegion>& regions,
        std::unordered_map<int64_t, CollisionWeight>& weights,
        const SolverConfig& config);

    /// Disrupt multi-sheet solution: swap items between sheets
    static void disruptMultiSheet(
        Solution& sol,
        const std::vector<SheetRegion>& regions,
        const SolverConfig& config,
        std::mt19937& rng);
};

} // namespace nest
