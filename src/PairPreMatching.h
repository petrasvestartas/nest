#pragma once

#include <vector>
#include <memory>
#include <utility>

#include "NFP.h"
#include "HelperTypes.h"
#include "NfpWorker.h"
#include "SolverConfig.h"

namespace nest {

/// Pre-compute NFP area for all piece pairs at multiple rotations.
/// Small NFP area = tight interlock. Group best pairs adjacent in placement order.
class PairPreMatching {
public:
    /// Score for a pair of pieces at specific rotations.
    struct PairScore {
        int indexA, indexB;
        float rotA, rotB;
        double nfpArea;         // NFP area (smaller = tighter fit)
        double score;           // Combined score (lower = better pairing)
    };

    /// Compute pair scores for all piece pairs at multiple rotations.
    /// @param parts The parts to analyze
    /// @param rotations Number of rotation angles to try per part
    /// @return Vector of pair scores, sorted by score (best first)
    static std::vector<PairScore> computePairScores(
        const std::vector<std::shared_ptr<NFP>>& parts,
        int rotations = 4);

    /// Reorder parts for better packing based on pair matching.
    /// Groups tightly-interlocking pairs adjacent in the ordering.
    /// @param parts The parts to reorder (modified in place)
    /// @param scores Pre-computed pair scores
    /// @return Reordered indices
    static std::vector<int> reorderByPairMatching(
        const std::vector<std::shared_ptr<NFP>>& parts,
        const std::vector<PairScore>& scores);

    /// Compute the NFP area between two parts at given rotations.
    static double computeNfpArea(
        NFP& partA, NFP& partB,
        float rotA, float rotB);

    /// Get recommended rotations for a pair based on pair analysis.
    /// @param partA First part
    /// @param partB Second part
    /// @param scores Pre-computed scores for this pair
    /// @return Pair of (rotA, rotB) for best interlock
    static std::pair<float, float> getBestRotations(
        int indexA, int indexB,
        const std::vector<PairScore>& scores);
};

} // namespace nest
