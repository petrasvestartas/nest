#include "JostleHeuristic.h"
#include "GeometryUtil.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>

namespace nest {

double JostleHeuristic::computeUsedArea(
    const std::vector<std::shared_ptr<NFP>>& placed,
    const std::vector<PlacementItem>& placements)
{
    if (placed.empty()) return 0;

    double minX = std::numeric_limits<double>::max();
    double maxX = -std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxY = -std::numeric_limits<double>::max();

    for (size_t i = 0; i < placed.size(); i++) {
        for (int j = 0; j < placed[i]->length(); j++) {
            double px = placed[i]->Points[j].x + placements[i].x;
            double py = placed[i]->Points[j].y + placements[i].y;
            minX = std::min(minX, px);
            maxX = std::max(maxX, px);
            minY = std::min(minY, py);
            maxY = std::max(maxY, py);
        }
    }

    return (maxX - minX) * (maxY - minY);
}

double JostleHeuristic::jostleCycle(
    std::vector<std::shared_ptr<NFP>>& placed,
    std::vector<PlacementItem>& placements,
    const NFP& sheet,
    double stepMin)
{
    double totalMoved = 0;

    // Four-direction compaction: left, down, right, up
    // The order and direction reversal is key to the jostle effect
    std::vector<Compaction::SlideDirection> directions = {
        { -1.0,  0.0, "left" },
        {  0.0, -1.0, "down" },
        {  1.0,  0.0, "right" },
        {  0.0,  1.0, "up" },
    };

    for (auto& dir : directions) {
        totalMoved += Compaction::compactInDirection(
            dir, placed, placements, sheet, stepMin);
    }

    return totalMoved;
}

double JostleHeuristic::jostle(
    std::vector<std::shared_ptr<NFP>>& placed,
    std::vector<PlacementItem>& placements,
    const NFP& sheet,
    const SolverConfig& config)
{
    if (placed.empty() || placements.empty()) return 0;

    double totalMoved = 0;
    double bestArea = computeUsedArea(placed, placements);
    double stepMin = config.compactionStepMin;

    // Save best placement state
    std::vector<PlacementItem> bestPlacements = placements;

    for (int iter = 0; iter < config.jostleIterations; iter++) {
        double cycleMoved = jostleCycle(placed, placements, sheet, stepMin);
        totalMoved += cycleMoved;

        double currentArea = computeUsedArea(placed, placements);

        if (currentArea < bestArea - 0.01) {
            bestArea = currentArea;
            bestPlacements = placements;
        }

        // Early exit if no significant movement
        if (cycleMoved < stepMin * placements.size()) {
            break;
        }
    }

    // Restore best placement found
    placements = bestPlacements;

    return totalMoved;
}

} // namespace nest
