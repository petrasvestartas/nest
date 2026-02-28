#pragma once

namespace nest {

enum class PlacementTypeEnum {
    box = 0,
    gravity = 1,
    squeeze = 2
};

/// Nesting configuration
struct NestConfig {
    PlacementTypeEnum placementType = PlacementTypeEnum::box;
    double curveTolerance = 0.72;
    double scale = 25;
    double clipperScale = 10000000;
    bool exploreConcave = false;
    int mutationRate = 10;
    int populationSize = 120;
    int rotations = 4;
    double spacing = 10;
    double sheetSpacing = 0;
    bool mergeLines = false;
    bool simplify = false;

    // Port features (don't exist in the original DeepNest project)
    bool clipByHull = false;
    bool clipByRects = true;
    int seed = -1;
    float rotation_limit = 360.0f;

    // Placement improvements
    int edgeSamples = 2;         // 0=off, N=samples per edge of feasible region
    int compactionPasses = 2;    // 0=off, N=compaction iterations after placement
    double gravityWeight = 0.1;  // 0=off, weight for centroid-distance scoring
    bool tryAllRotations = false;   // try all valid rotations per part, pick best
};

} // namespace nest
