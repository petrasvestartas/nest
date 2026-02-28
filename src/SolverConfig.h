#pragma once

#include "NestConfig.h"

namespace nest {

enum class SolverType {
    Classic,            // Original NFP+GA (BLF placement)
    CompactedBLF,       // BLF + post-placement slide compaction
    Jostle,             // Bidirectional breathing compaction
    Exhaustive,         // Full enumeration for small N
    OverlapResolution,  // Sparrow-inspired two-phase solver
    SmartBLF,           // PairMatching + tryAllRotations BLF + Compaction + Jostle
    Auto                // Auto-select based on N
};

struct SolverConfig {
    SolverType solver = SolverType::Auto;

    // Base nesting config (shared across all solvers)
    NestConfig baseConfig;

    // Compaction parameters
    int compactionRounds = 5;           // Number of compaction passes
    double compactionStepMin = 0.1;     // Minimum slide step (binary search limit)

    // Jostle parameters
    int jostleIterations = 10;          // Number of jostle oscillation cycles

    // Continuous rotation parameters
    int rotationSteps = 36;             // Number of rotation angles to try (360/steps degrees each)
    bool usePrincipalAxis = true;       // Align to principal axis first

    // Exhaustive solver parameters
    int exhaustiveMaxN = 8;             // Max items for exhaustive search
    int exhaustiveMaxRotations = 4;     // Rotations per item in exhaustive

    // Overlap resolution parameters (Sparrow-inspired)
    double explorationTimeFraction = 0.8;   // Fraction of time for exploration phase
    double shrinkStep = 0.001;              // Strip shrink ratio per iteration
    double shrinkMin = 0.00001;             // Minimum shrink ratio (compression phase)
    double shrinkMax = 0.0005;              // Maximum shrink ratio (compression phase)
    int orMaxWorkers = 3;                   // Parallel workers
    int orContainerSamples = 50;            // Random samples across container
    int orFocusedSamples = 25;              // Focused samples near current position
    int orCoordDescentTopN = 3;             // Top-N samples for coordinate descent
    int orStrikeLimit = 3;                  // Strikes before abandoning width
    int orIterNoImproveLimit = 200;         // Iterations without improvement before strike
    double glsWeightDecay = 0.95;           // GLS weight decay for resolved collisions
    double glsWeightMinInc = 1.2;           // Minimum weight increase for collisions
    double glsWeightMaxInc = 2.0;           // Maximum weight increase for collisions
    double largeItemPercentile = 0.75;      // Percentile for "large" item classification

    // Pair pre-matching
    bool usePairMatching = false;           // Enable pair pre-matching for ordering

    // Time budget (seconds), 0 = unlimited
    double timeBudgetSeconds = 60.0;
};

} // namespace nest
