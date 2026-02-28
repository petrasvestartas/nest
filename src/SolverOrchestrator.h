#pragma once

#include <vector>
#include <memory>
#include <string>
#include <chrono>
#include <functional>

#include "NFP.h"
#include "HelperTypes.h"
#include "SolverConfig.h"
#include "NestingContext.h"

namespace nest {

/// Unified solver result
struct SolverResult {
    SolverType solverUsed;
    double fitness;                     // Lower = better (sheet area used)
    double materialUtilization;         // 0-1 ratio
    int sheetsUsed;
    int partsPlaced;
    int totalParts;
    double elapsedMs;
    std::string solverName;

    // Per-part results
    struct PartResult {
        int partIndex;
        int source;
        double x, y;
        float rotation;
        int sheetIndex;
        bool placed;
    };
    std::vector<PartResult> parts;
};

/// Routes to the correct solver based on SolverConfig.
/// Also provides solveAuto() which auto-selects based on N.
class SolverOrchestrator {
public:
    /// Solve using the specified solver type.
    /// @param parts Parts to nest
    /// @param sheets Available sheets
    /// @param config Solver configuration
    /// @param progressCallback Optional callback
    /// @return Solver result
    static SolverResult solve(
        const std::vector<std::shared_ptr<NFP>>& parts,
        const std::vector<std::shared_ptr<NFP>>& sheets,
        const SolverConfig& config,
        std::function<void(const std::string&)> progressCallback = nullptr);

    /// Auto-select solver based on problem size.
    /// N <= 8 -> Exhaustive + compaction
    /// N = 9-20 -> GA + jostle
    /// N > 20 -> OverlapResolution
    static SolverResult solveAuto(
        const std::vector<std::shared_ptr<NFP>>& parts,
        const std::vector<std::shared_ptr<NFP>>& sheets,
        const SolverConfig& config,
        std::function<void(const std::string&)> progressCallback = nullptr);

    /// Run classic GA solver (wraps NestingContext)
    static SolverResult solveClassic(
        const std::vector<std::shared_ptr<NFP>>& parts,
        const std::vector<std::shared_ptr<NFP>>& sheets,
        const SolverConfig& config,
        std::function<void(const std::string&)> progressCallback = nullptr);

    /// Run BLF + compaction
    static SolverResult solveCompactedBLF(
        const std::vector<std::shared_ptr<NFP>>& parts,
        const std::vector<std::shared_ptr<NFP>>& sheets,
        const SolverConfig& config,
        std::function<void(const std::string&)> progressCallback = nullptr);

    /// Run jostle heuristic (BLF + compaction + jostle breathing)
    static SolverResult solveJostle(
        const std::vector<std::shared_ptr<NFP>>& parts,
        const std::vector<std::shared_ptr<NFP>>& sheets,
        const SolverConfig& config,
        std::function<void(const std::string&)> progressCallback = nullptr);

    /// Run exhaustive enumeration (small N only)
    static SolverResult solveExhaustive(
        const std::vector<std::shared_ptr<NFP>>& parts,
        const std::vector<std::shared_ptr<NFP>>& sheets,
        const SolverConfig& config,
        std::function<void(const std::string&)> progressCallback = nullptr);

    /// Run overlap resolution solver
    static SolverResult solveOverlapResolution(
        const std::vector<std::shared_ptr<NFP>>& parts,
        const std::vector<std::shared_ptr<NFP>>& sheets,
        const SolverConfig& config,
        std::function<void(const std::string&)> progressCallback = nullptr);

    /// Run SmartBLF: PairMatching + tryAllRotations BLF + Compaction + Jostle
    static SolverResult solveSmartBLF(
        const std::vector<std::shared_ptr<NFP>>& parts,
        const std::vector<std::shared_ptr<NFP>>& sheets,
        const SolverConfig& config,
        std::function<void(const std::string&)> progressCallback = nullptr);

    /// Get human-readable name for a solver type
    static std::string solverName(SolverType type);
};

} // namespace nest
