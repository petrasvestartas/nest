#pragma once

#include <vector>
#include <memory>
#include <functional>

#include "NFP.h"
#include "HelperTypes.h"
#include "SolverConfig.h"
#include "NfpWorker.h"
#include "Compaction.h"
#include "ContinuousRotation.h"

namespace nest {

/// Full enumeration solver for small N.
/// Enumerates all N! orderings x R^N rotation combinations,
/// runs fast BLF+compaction for each, keeps best.
class ExhaustiveSolver {
public:
    /// Result of a single evaluation
    struct EvalResult {
        std::vector<int> ordering;          // Part ordering
        std::vector<float> rotations;       // Rotation per part
        double fitness;                     // Fitness (lower = better)
        int sheetsUsed;                     // Number of sheets used
        std::vector<PlacementItem> placements;
    };

    /// Run exhaustive search.
    /// @param parts Parts to nest
    /// @param sheets Available sheets
    /// @param config Solver configuration
    /// @param progressCallback Optional callback(evaluated, total)
    /// @return Best evaluation result
    static EvalResult solve(
        const std::vector<std::shared_ptr<NFP>>& parts,
        const std::vector<std::shared_ptr<NFP>>& sheets,
        const SolverConfig& config,
        std::function<void(int, int)> progressCallback = nullptr);

    /// Estimate total evaluations for given N and rotations.
    static long long estimateEvaluations(int n, int rotations);

    /// Check if exhaustive search is feasible within time budget.
    static bool isFeasible(int n, int rotations, double timeBudgetSeconds);

private:
    /// Evaluate a single ordering+rotation combination using BLF placement.
    static EvalResult evaluateSingle(
        const std::vector<std::shared_ptr<NFP>>& parts,
        const std::vector<std::shared_ptr<NFP>>& sheets,
        const std::vector<int>& ordering,
        const std::vector<float>& rotations,
        const SolverConfig& config,
        NfpWorker& worker);

    /// Generate all permutations of [0, n)
    static void generatePermutations(
        int n,
        std::vector<std::vector<int>>& result);

    /// Generate all rotation combinations
    static void generateRotationCombinations(
        int n, int numRotations,
        std::vector<std::vector<float>>& result);
};

} // namespace nest
