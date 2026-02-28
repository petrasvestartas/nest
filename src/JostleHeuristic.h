#pragma once

#include <vector>
#include <memory>

#include "NFP.h"
#include "HelperTypes.h"
#include "SolverConfig.h"
#include "Compaction.h"

namespace nest {

/// Bidirectional breathing compaction (Jostle heuristic).
/// Inspired by Dowsland 1998 / Abeysooriya 2018.
/// Alternating left→down→right→up passes let pieces escape local minima.
class JostleHeuristic {
public:
    /// Run the jostle heuristic on placed pieces.
    /// @param placed The placed polygons (already positioned by BLF)
    /// @param placements Current placement positions
    /// @param sheet Sheet boundary
    /// @param config Solver configuration
    /// @return Total distance moved across all iterations
    static double jostle(
        std::vector<std::shared_ptr<NFP>>& placed,
        std::vector<PlacementItem>& placements,
        const NFP& sheet,
        const SolverConfig& config);

    /// Single jostle cycle: compact in 4 directions.
    /// @return Total distance moved in this cycle
    static double jostleCycle(
        std::vector<std::shared_ptr<NFP>>& placed,
        std::vector<PlacementItem>& placements,
        const NFP& sheet,
        double stepMin);

    /// Compute the total bounding box area used by all placements.
    /// Used as the fitness metric for jostle improvement tracking.
    static double computeUsedArea(
        const std::vector<std::shared_ptr<NFP>>& placed,
        const std::vector<PlacementItem>& placements);
};

} // namespace nest
