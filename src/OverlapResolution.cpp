#include "OverlapResolution.h"
#include "NfpWorker.h"
#include <algorithm>
#include <numeric>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <cstring>
#include <set>

namespace nest {

// ============================================================================
// Spatial Grid
// ============================================================================

int64_t OverlapResolution::SpatialGrid::cellKey(double x, double y) const {
    int32_t cx = static_cast<int32_t>(std::floor((x - offsetX_) / cellSize_));
    int32_t cy = static_cast<int32_t>(std::floor((y - offsetY_) / cellSize_));
    return (static_cast<int64_t>(cx) << 32) | (static_cast<uint32_t>(cy));
}

void OverlapResolution::SpatialGrid::build(
    const std::vector<ItemState>& items, double cellSize)
{
    cells.clear();
    cellSize_ = cellSize;

    if (items.empty()) return;

    // Find offset
    offsetX_ = items[0].x;
    offsetY_ = items[0].y;
    for (auto& item : items) {
        offsetX_ = std::min(offsetX_, item.x);
        offsetY_ = std::min(offsetY_, item.y);
    }

    for (size_t i = 0; i < items.size(); i++) {
        auto& item = items[i];
        NFP poly = getTransformedPoly(item);
        PolygonBounds bounds = GeometryUtil::getPolygonBounds(poly);

        // Insert into all cells the bounding box overlaps
        int cxMin = static_cast<int>(std::floor((bounds.x - offsetX_) / cellSize_));
        int cxMax = static_cast<int>(std::floor((bounds.x + bounds.width - offsetX_) / cellSize_));
        int cyMin = static_cast<int>(std::floor((bounds.y - offsetY_) / cellSize_));
        int cyMax = static_cast<int>(std::floor((bounds.y + bounds.height - offsetY_) / cellSize_));

        for (int cx = cxMin; cx <= cxMax; cx++) {
            for (int cy = cyMin; cy <= cyMax; cy++) {
                int64_t key = (static_cast<int64_t>(cx) << 32) | static_cast<uint32_t>(cy);
                cells[key].items.push_back(static_cast<int>(i));
            }
        }
    }
}

std::vector<int> OverlapResolution::SpatialGrid::getNeighbors(int itemIdx) const {
    std::vector<int> result;
    std::set<int> seen;

    for (auto& [key, cell] : cells) {
        bool hasItem = false;
        for (int idx : cell.items) {
            if (idx == itemIdx) { hasItem = true; break; }
        }
        if (hasItem) {
            for (int idx : cell.items) {
                if (idx != itemIdx && !seen.count(idx)) {
                    result.push_back(idx);
                    seen.insert(idx);
                }
            }
        }
    }
    return result;
}

void OverlapResolution::SpatialGrid::clear() {
    cells.clear();
}

// ============================================================================
// Helper functions
// ============================================================================

int64_t OverlapResolution::pairKey(int a, int b) {
    if (a > b) std::swap(a, b);
    return (static_cast<int64_t>(a) << 32) | static_cast<uint32_t>(b);
}

NFP OverlapResolution::getTransformedPoly(const ItemState& item) {
    double rad = item.rotation * M_PI / 180.0;
    double cosA = std::cos(rad);
    double sinA = std::sin(rad);

    NFP result;
    result.Points.reserve(item.polygon->length());
    for (int i = 0; i < item.polygon->length(); i++) {
        double px = item.polygon->Points[i].x;
        double py = item.polygon->Points[i].y;
        double rx = px * cosA - py * sinA + item.x;
        double ry = px * sinA + py * cosA + item.y;
        result.AddPoint(Point(rx, ry));
    }
    return result;
}

double OverlapResolution::computePairOverlap(const ItemState& a, const ItemState& b) {
    NFP polyA = getTransformedPoly(a);
    NFP polyB = getTransformedPoly(b);

    // Bounding box pre-check
    PolygonBounds boundsA = GeometryUtil::getPolygonBounds(polyA);
    PolygonBounds boundsB = GeometryUtil::getPolygonBounds(polyB);

    if (boundsA.x > boundsB.x + boundsB.width + 0.1 ||
        boundsB.x > boundsA.x + boundsA.width + 0.1 ||
        boundsA.y > boundsB.y + boundsB.height + 0.1 ||
        boundsB.y > boundsA.y + boundsA.height + 0.1) {
        return 0;
    }

    return Compaction::computeOverlapArea(polyA, polyB);
}

double OverlapResolution::computeSheetViolation(const ItemState& item, const NFP& sheet) {
    NFP poly = getTransformedPoly(item);
    PolygonBounds sheetBounds = GeometryUtil::getPolygonBounds(sheet);
    PolygonBounds polyBounds = GeometryUtil::getPolygonBounds(poly);

    double violation = 0;

    // Compute area outside sheet
    if (polyBounds.x < sheetBounds.x)
        violation += (sheetBounds.x - polyBounds.x) * polyBounds.height;
    if (polyBounds.x + polyBounds.width > sheetBounds.x + sheetBounds.width)
        violation += (polyBounds.x + polyBounds.width - sheetBounds.x - sheetBounds.width) * polyBounds.height;
    if (polyBounds.y < sheetBounds.y)
        violation += (sheetBounds.y - polyBounds.y) * polyBounds.width;
    if (polyBounds.y + polyBounds.height > sheetBounds.y + sheetBounds.height)
        violation += (polyBounds.y + polyBounds.height - sheetBounds.y - sheetBounds.height) * polyBounds.width;

    return violation;
}

// ============================================================================
// Solution evaluation
// ============================================================================

void OverlapResolution::evaluateSolution(
    Solution& sol, const NFP& sheet,
    const std::unordered_map<int64_t, CollisionWeight>& weights)
{
    int n = static_cast<int>(sol.items.size());
    sol.totalLoss = 0;
    sol.weightedLoss = 0;

    for (int i = 0; i < n; i++) {
        // Sheet violation
        double sheetViol = computeSheetViolation(sol.items[i], sheet);
        sol.totalLoss += sheetViol * 2.0;
        sol.weightedLoss += sheetViol * 2.0;

        // Pairwise overlaps
        for (int j = i + 1; j < n; j++) {
            double overlap = computePairOverlap(sol.items[i], sol.items[j]);
            if (overlap > 0.01) {
                sol.totalLoss += overlap;
                int64_t pk = pairKey(i, j);
                auto it = weights.find(pk);
                double w = (it != weights.end()) ? it->second.weight : 1.0;
                sol.weightedLoss += overlap * w;
            }
        }
    }

    sol.feasible = (sol.totalLoss < 0.1);
}

double OverlapResolution::evaluateItemLoss(
    const Solution& sol,
    int itemIdx,
    const ItemState& candidate,
    const NFP& sheet,
    const std::unordered_map<int64_t, CollisionWeight>& weights,
    const SpatialGrid& grid)
{
    double loss = 0;

    // Sheet violation
    loss += computeSheetViolation(candidate, sheet) * 2.0;

    // Check against neighbors (from spatial grid)
    auto neighbors = grid.getNeighbors(itemIdx);
    for (int j : neighbors) {
        if (j == itemIdx) continue;
        double overlap = computePairOverlap(candidate, sol.items[j]);
        if (overlap > 0.01) {
            int64_t pk = pairKey(itemIdx, j);
            auto it = weights.find(pk);
            double w = (it != weights.end()) ? it->second.weight : 1.0;
            loss += overlap * w;
        }
    }

    return loss;
}

// ============================================================================
// Initialization
// ============================================================================

OverlapResolution::Solution OverlapResolution::initializeSolution(
    const std::vector<std::shared_ptr<NFP>>& parts,
    const NFP& sheet,
    std::mt19937& rng)
{
    Solution sol;
    PolygonBounds sheetBounds = GeometryUtil::getPolygonBounds(sheet);

    // Place items in a grid pattern within the sheet
    int n = static_cast<int>(parts.size());
    int cols = static_cast<int>(std::ceil(std::sqrt(n)));

    double cellW = sheetBounds.width / cols;
    double cellH = sheetBounds.height / std::max(1, (n + cols - 1) / cols);

    for (int i = 0; i < n; i++) {
        ItemState item;
        item.partIndex = i;
        item.polygon = parts[i];
        item.rotation = 0;

        int col = i % cols;
        int row = i / cols;

        // Center in cell
        PolygonBounds partBounds = GeometryUtil::getPolygonBounds(*parts[i]);
        item.x = sheetBounds.x + col * cellW + cellW / 2.0 - partBounds.width / 2.0;
        item.y = sheetBounds.y + row * cellH + cellH / 2.0 - partBounds.height / 2.0;

        sol.items.push_back(item);
    }

    sol.stripWidth = sheetBounds.width;
    sol.totalLoss = 0;
    sol.weightedLoss = 0;
    sol.feasible = false;

    return sol;
}

// ============================================================================
// Sampling and coordinate descent
// ============================================================================

std::vector<OverlapResolution::ItemState> OverlapResolution::samplePositions(
    const Solution& sol,
    int itemIdx,
    const NFP& sheet,
    int containerSamples,
    int focusedSamples,
    std::mt19937& rng)
{
    std::vector<ItemState> candidates;
    PolygonBounds sheetBounds = GeometryUtil::getPolygonBounds(sheet);
    PolygonBounds partBounds = GeometryUtil::getPolygonBounds(*sol.items[itemIdx].polygon);

    double minX = sheetBounds.x;
    double maxX = sheetBounds.x + sheetBounds.width - partBounds.width;
    double minY = sheetBounds.y;
    double maxY = sheetBounds.y + sheetBounds.height - partBounds.height;

    if (maxX < minX) maxX = minX;
    if (maxY < minY) maxY = minY;

    std::uniform_real_distribution<double> distX(minX, maxX);
    std::uniform_real_distribution<double> distY(minY, maxY);

    // Container-wide samples
    for (int s = 0; s < containerSamples; s++) {
        ItemState candidate = sol.items[itemIdx];
        candidate.x = distX(rng);
        candidate.y = distY(rng);
        candidates.push_back(candidate);
    }

    // Focused samples near current position
    double focusRadius = std::min(sheetBounds.width, sheetBounds.height) / 4.0;
    std::normal_distribution<double> normalDist(0, focusRadius);

    for (int s = 0; s < focusedSamples; s++) {
        ItemState candidate = sol.items[itemIdx];
        candidate.x = std::clamp(candidate.x + normalDist(rng), minX, maxX);
        candidate.y = std::clamp(candidate.y + normalDist(rng), minY, maxY);
        candidates.push_back(candidate);
    }

    return candidates;
}

OverlapResolution::ItemState OverlapResolution::coordinateDescent(
    const Solution& sol,
    int itemIdx,
    const ItemState& start,
    const NFP& sheet,
    const std::unordered_map<int64_t, CollisionWeight>& weights,
    const SpatialGrid& grid,
    double stepInit,
    double stepLimit,
    double rotStepInit,
    double rotStepLimit)
{
    ItemState current = start;
    double currentLoss = evaluateItemLoss(sol, itemIdx, current, sheet, weights, grid);

    double stepX = stepInit;
    double stepY = stepInit;
    double stepRot = rotStepInit;

    const double successMul = 1.1;
    const double failMul = 0.5;

    // Cycle through axes
    std::vector<CDAxis> axes = {
        CDAxis::Horizontal, CDAxis::Vertical,
        CDAxis::ForwardDiag, CDAxis::BackwardDiag,
        CDAxis::Rotation
    };
    int axisIdx = 0;

    int maxIter = 200;
    for (int iter = 0; iter < maxIter; iter++) {
        // Check if all steps are below limit
        if (stepX < stepLimit && stepY < stepLimit && stepRot < rotStepLimit) {
            break;
        }

        CDAxis axis = axes[axisIdx % axes.size()];

        // Generate two candidates (positive and negative step)
        ItemState cand1 = current, cand2 = current;

        switch (axis) {
            case CDAxis::Horizontal:
                cand1.x += stepX; cand2.x -= stepX;
                break;
            case CDAxis::Vertical:
                cand1.y += stepY; cand2.y -= stepY;
                break;
            case CDAxis::ForwardDiag:
                cand1.x += stepX * 0.7071; cand1.y += stepY * 0.7071;
                cand2.x -= stepX * 0.7071; cand2.y -= stepY * 0.7071;
                break;
            case CDAxis::BackwardDiag:
                cand1.x -= stepX * 0.7071; cand1.y += stepY * 0.7071;
                cand2.x += stepX * 0.7071; cand2.y -= stepY * 0.7071;
                break;
            case CDAxis::Rotation:
                cand1.rotation += static_cast<float>(stepRot);
                cand2.rotation -= static_cast<float>(stepRot);
                break;
        }

        double loss1 = evaluateItemLoss(sol, itemIdx, cand1, sheet, weights, grid);
        double loss2 = evaluateItemLoss(sol, itemIdx, cand2, sheet, weights, grid);

        double bestCandLoss = std::min(loss1, loss2);
        ItemState& bestCand = (loss1 <= loss2) ? cand1 : cand2;

        if (bestCandLoss < currentLoss - 0.001) {
            // Success: accept and grow step
            current = bestCand;
            currentLoss = bestCandLoss;

            switch (axis) {
                case CDAxis::Horizontal: stepX *= successMul; break;
                case CDAxis::Vertical: stepY *= successMul; break;
                case CDAxis::ForwardDiag:
                case CDAxis::BackwardDiag:
                    stepX *= std::sqrt(successMul);
                    stepY *= std::sqrt(successMul);
                    break;
                case CDAxis::Rotation: stepRot *= successMul; break;
            }
        } else {
            // Failure: shrink step, try next axis
            switch (axis) {
                case CDAxis::Horizontal: stepX *= failMul; break;
                case CDAxis::Vertical: stepY *= failMul; break;
                case CDAxis::ForwardDiag:
                case CDAxis::BackwardDiag:
                    stepX *= std::sqrt(failMul);
                    stepY *= std::sqrt(failMul);
                    break;
                case CDAxis::Rotation: stepRot *= failMul; break;
            }
            axisIdx++;
        }
    }

    return current;
}

// ============================================================================
// GLS weight management
// ============================================================================

void OverlapResolution::updateGLSWeights(
    const Solution& sol,
    const NFP& sheet,
    std::unordered_map<int64_t, CollisionWeight>& weights,
    const SolverConfig& config)
{
    int n = static_cast<int>(sol.items.size());
    double maxLoss = 0;

    // First pass: find max loss and update loss values
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            double overlap = computePairOverlap(sol.items[i], sol.items[j]);
            int64_t pk = pairKey(i, j);
            weights[pk].loss = overlap;
            maxLoss = std::max(maxLoss, overlap);
        }
    }

    // Second pass: update weights
    for (auto& [pk, cw] : weights) {
        if (cw.loss < 0.01) {
            // No collision: decay weight
            cw.weight *= config.glsWeightDecay;
            if (cw.weight < 1.0) cw.weight = 1.0;
        } else if (maxLoss > 0.01) {
            // Collision: increase weight based on severity
            double relSeverity = cw.loss / maxLoss;
            double multiplier = config.glsWeightMinInc +
                               (config.glsWeightMaxInc - config.glsWeightMinInc) * relSeverity;
            cw.weight *= multiplier;
        }
    }
}

// ============================================================================
// Disruption
// ============================================================================

void OverlapResolution::disruptSolution(
    Solution& sol,
    const SolverConfig& config,
    std::mt19937& rng)
{
    if (sol.items.size() < 2) return;

    // Find "large" items (by convex hull area)
    std::vector<std::pair<double, int>> areas;
    for (size_t i = 0; i < sol.items.size(); i++) {
        double area = std::fabs(GeometryUtil::polygonArea(*sol.items[i].polygon));
        areas.push_back({area, static_cast<int>(i)});
    }
    std::sort(areas.rbegin(), areas.rend());

    int cutoff = static_cast<int>(areas.size() * (1.0 - config.largeItemPercentile));
    cutoff = std::max(cutoff, 2);

    // Pick two random large items and swap positions
    std::uniform_int_distribution<int> dist(0, std::min(cutoff, static_cast<int>(areas.size())) - 1);
    int idx1 = areas[dist(rng)].second;
    int idx2 = areas[dist(rng)].second;
    while (idx2 == idx1 && sol.items.size() > 1) {
        idx2 = areas[dist(rng)].second;
    }

    std::swap(sol.items[idx1].x, sol.items[idx2].x);
    std::swap(sol.items[idx1].y, sol.items[idx2].y);
}

void OverlapResolution::shrinkStrip(
    Solution& sol,
    double shrinkRatio,
    std::mt19937& rng)
{
    double delta = sol.stripWidth * shrinkRatio;
    sol.stripWidth -= delta;

    // Shift items that are too far right
    for (auto& item : sol.items) {
        NFP poly = getTransformedPoly(item);
        PolygonBounds bounds = GeometryUtil::getPolygonBounds(poly);
        if (bounds.x + bounds.width > sol.stripWidth) {
            item.x -= (bounds.x + bounds.width - sol.stripWidth);
        }
    }
}

// ============================================================================
// Separation procedure
// ============================================================================

bool OverlapResolution::moveItem(
    Solution& sol,
    int itemIdx,
    const NFP& sheet,
    const std::unordered_map<int64_t, CollisionWeight>& weights,
    const SpatialGrid& grid,
    const SolverConfig& config,
    std::mt19937& rng)
{
    double currentLoss = evaluateItemLoss(sol, itemIdx, sol.items[itemIdx],
                                           sheet, weights, grid);
    if (currentLoss < 0.01) return false; // Not colliding

    // Sample candidate positions
    auto candidates = samplePositions(sol, itemIdx, sheet,
                                       config.orContainerSamples,
                                       config.orFocusedSamples, rng);

    // Evaluate all candidates
    std::vector<std::pair<double, int>> scores;
    for (size_t i = 0; i < candidates.size(); i++) {
        double loss = evaluateItemLoss(sol, itemIdx, candidates[i],
                                        sheet, weights, grid);
        scores.push_back({loss, static_cast<int>(i)});
    }

    // Keep top-N for coordinate descent
    std::sort(scores.begin(), scores.end());
    int topN = std::min(config.orCoordDescentTopN, static_cast<int>(scores.size()));

    PolygonBounds partBounds = GeometryUtil::getPolygonBounds(*sol.items[itemIdx].polygon);
    double minDim = std::min(partBounds.width, partBounds.height);
    if (minDim < 1.0) minDim = 1.0;

    // Coarse coordinate descent on top-N
    ItemState bestCandidate = sol.items[itemIdx];
    double bestLoss = currentLoss;

    for (int i = 0; i < topN; i++) {
        auto refined = coordinateDescent(
            sol, itemIdx, candidates[scores[i].second], sheet, weights, grid,
            0.25 * minDim, 0.02 * minDim);

        double loss = evaluateItemLoss(sol, itemIdx, refined, sheet, weights, grid);
        if (loss < bestLoss) {
            bestLoss = loss;
            bestCandidate = refined;
        }
    }

    // Fine coordinate descent on the winner
    if (bestLoss < currentLoss) {
        auto fineRefined = coordinateDescent(
            sol, itemIdx, bestCandidate, sheet, weights, grid,
            0.01 * minDim, 0.001 * minDim, 0.5, 0.05);

        double fineLoss = evaluateItemLoss(sol, itemIdx, fineRefined, sheet, weights, grid);
        if (fineLoss < bestLoss) {
            bestCandidate = fineRefined;
            bestLoss = fineLoss;
        }
    }

    if (bestLoss < currentLoss - 0.001) {
        sol.items[itemIdx] = bestCandidate;
        return true;
    }

    return false;
}

bool OverlapResolution::separate(
    Solution& sol,
    const NFP& sheet,
    std::unordered_map<int64_t, CollisionWeight>& weights,
    SpatialGrid& grid,
    const SolverConfig& config,
    std::mt19937& rng,
    int strikeLimit,
    int iterNoImproveLimit)
{
    int strikes = 0;
    int iterNoImprove = 0;
    double bestLoss = std::numeric_limits<double>::max();

    // Compute median item size for grid cell size
    double medianDim = 100;
    if (!sol.items.empty()) {
        std::vector<double> dims;
        for (auto& item : sol.items) {
            PolygonBounds b = GeometryUtil::getPolygonBounds(*item.polygon);
            dims.push_back(std::max(b.width, b.height));
        }
        std::sort(dims.begin(), dims.end());
        medianDim = dims.back(); // Use max dimension so largest items share cells with neighbors
    }

    auto separateStart = std::chrono::high_resolution_clock::now();

    for (int iter = 0; iter < 10000; iter++) {
        // Time guard: abort after 10 seconds to prevent infinite iteration
        auto now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(now - separateStart).count();
        if (elapsed > 10.0) return sol.feasible;

        // Rebuild spatial grid
        grid.build(sol.items, medianDim);

        // Evaluate current solution
        evaluateSolution(sol, sheet, weights);

        if (sol.feasible) return true;

        if (sol.totalLoss < bestLoss - 0.01) {
            bestLoss = sol.totalLoss;
            iterNoImprove = 0;
        } else {
            iterNoImprove++;
        }

        // Strike on stagnation
        if (iterNoImprove >= iterNoImproveLimit) {
            strikes++;
            iterNoImprove = 0;
            updateGLSWeights(sol, sheet, weights, config);

            if (strikes >= strikeLimit) {
                return false;
            }
        }

        // Collect colliding items
        std::vector<int> collidingItems;
        for (size_t i = 0; i < sol.items.size(); i++) {
            double itemLoss = evaluateItemLoss(sol, static_cast<int>(i), sol.items[i],
                                                sheet, weights, grid);
            if (itemLoss > 0.01) {
                collidingItems.push_back(static_cast<int>(i));
            }
        }

        if (collidingItems.empty()) {
            sol.feasible = true;
            return true;
        }

        // Shuffle and process colliding items
        std::shuffle(collidingItems.begin(), collidingItems.end(), rng);

        for (int idx : collidingItems) {
            moveItem(sol, idx, sheet, weights, grid, config, rng);
        }
    }

    return sol.feasible;
}

// ============================================================================
// Main solver
// ============================================================================

OverlapResolution::Solution OverlapResolution::solve(
    const std::vector<std::shared_ptr<NFP>>& parts,
    const NFP& sheet,
    const SolverConfig& config,
    std::function<void(int, double)> progressCallback)
{
    std::mt19937 rng(42);

    // Initialize solution
    Solution current = initializeSolution(parts, sheet, rng);
    std::unordered_map<int64_t, CollisionWeight> weights;
    SpatialGrid grid;

    Solution bestFeasible;
    bestFeasible.feasible = false;
    bestFeasible.totalLoss = std::numeric_limits<double>::max();
    bestFeasible.stripWidth = current.stripWidth;

    auto startTime = std::chrono::high_resolution_clock::now();
    double totalTime = config.timeBudgetSeconds;
    double explorationTime = totalTime * config.explorationTimeFraction;

    // Pool of infeasible solutions for disruption sampling
    std::vector<Solution> solutionPool;

    int iteration = 0;

    // ========================
    // Phase 1: Exploration
    // ========================
    while (true) {
        auto now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(now - startTime).count();
        if (elapsed > explorationTime) break;

        iteration++;

        // Try to achieve feasibility at current strip width
        bool feasible = separate(current, sheet, weights, grid, config, rng,
                                  config.orStrikeLimit, config.orIterNoImproveLimit);

        if (feasible) {
            if (!bestFeasible.feasible || current.stripWidth < bestFeasible.stripWidth) {
                bestFeasible = current;

                if (progressCallback) {
                    progressCallback(iteration, bestFeasible.stripWidth);
                }
            }

            // Shrink strip
            shrinkStrip(current, config.shrinkStep, rng);
            solutionPool.clear();
        } else {
            // Store infeasible solution
            if (solutionPool.size() < 50) {
                solutionPool.push_back(current);
            }

            // Sample from solution pool or disrupt
            if (!solutionPool.empty()) {
                std::normal_distribution<double> normalDist(0, 0.25 * solutionPool.size());
                int poolIdx = std::clamp(
                    static_cast<int>(std::fabs(normalDist(rng))),
                    0, static_cast<int>(solutionPool.size()) - 1);
                current = solutionPool[poolIdx];
            }

            disruptSolution(current, config, rng);
        }
    }

    // ========================
    // Phase 2: Compression
    // ========================
    if (bestFeasible.feasible) {
        current = bestFeasible;
        double shrinkRatio = config.shrinkMax;

        while (true) {
            auto now = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double>(now - startTime).count();
            if (elapsed > totalTime) break;

            // Time-based shrink decay
            double progress = (elapsed - explorationTime) / (totalTime - explorationTime);
            shrinkRatio = config.shrinkMax +
                         progress * (config.shrinkMin - config.shrinkMax);

            Solution attempt = current;
            shrinkStrip(attempt, shrinkRatio, rng);

            bool feasible = separate(attempt, sheet, weights, grid, config, rng,
                                      5, 100);

            if (feasible) {
                current = attempt;
                bestFeasible = current;

                if (progressCallback) {
                    progressCallback(iteration, bestFeasible.stripWidth);
                }
            }

            iteration++;
        }
    }

    // If no feasible solution found, return the initial with overlaps
    if (!bestFeasible.feasible) {
        evaluateSolution(current, sheet, weights);
        return current;
    }

    return bestFeasible;
}

// ============================================================================
// Multi-Sheet: Sheet layout
// ============================================================================

std::vector<OverlapResolution::SheetRegion> OverlapResolution::createSheetLayout(
    const NFP& sheetTemplate, int numSheets, double gap)
{
    PolygonBounds bounds = GeometryUtil::getPolygonBounds(sheetTemplate);
    std::vector<SheetRegion> regions;
    for (int i = 0; i < numSheets; i++) {
        SheetRegion r;
        r.x = i * (bounds.width + gap);
        r.y = 0;
        r.width = bounds.width;
        r.height = bounds.height;
        r.index = i;
        regions.push_back(r);
    }
    return regions;
}

int OverlapResolution::nearestSheet(
    const ItemState& item, const std::vector<SheetRegion>& regions)
{
    double bestDist = std::numeric_limits<double>::max();
    int best = 0;
    for (auto& r : regions) {
        double cx = r.x + r.width / 2.0;
        double cy = r.y + r.height / 2.0;
        double dx = item.x - cx;
        double dy = item.y - cy;
        double d = dx * dx + dy * dy;
        if (d < bestDist) {
            bestDist = d;
            best = r.index;
        }
    }
    return best;
}

// ============================================================================
// Multi-Sheet: Violation computation
// ============================================================================

double OverlapResolution::computeMultiRegionViolation(
    const ItemState& item, const std::vector<SheetRegion>& regions)
{
    NFP poly = getTransformedPoly(item);
    PolygonBounds pb = GeometryUtil::getPolygonBounds(poly);

    // Find the assigned/nearest region
    int sheetIdx = item.sheetIdx;
    if (sheetIdx < 0 || sheetIdx >= static_cast<int>(regions.size()))
        sheetIdx = nearestSheet(item, regions);

    auto& r = regions[sheetIdx];

    double violation = 0;
    if (pb.x < r.x)
        violation += (r.x - pb.x) * pb.height;
    if (pb.x + pb.width > r.x + r.width)
        violation += (pb.x + pb.width - r.x - r.width) * pb.height;
    if (pb.y < r.y)
        violation += (r.y - pb.y) * pb.width;
    if (pb.y + pb.height > r.y + r.height)
        violation += (pb.y + pb.height - r.y - r.height) * pb.width;

    return violation;
}

// ============================================================================
// Multi-Sheet: Solution evaluation
// ============================================================================

void OverlapResolution::evaluateMultiSheetSolution(
    Solution& sol,
    const std::vector<SheetRegion>& regions,
    const std::unordered_map<int64_t, CollisionWeight>& weights)
{
    int n = static_cast<int>(sol.items.size());
    sol.totalLoss = 0;
    sol.weightedLoss = 0;

    for (int i = 0; i < n; i++) {
        double sheetViol = computeMultiRegionViolation(sol.items[i], regions);
        sol.totalLoss += sheetViol * 2.0;
        sol.weightedLoss += sheetViol * 2.0;

        for (int j = i + 1; j < n; j++) {
            // Items on different sheets can't overlap (they're far apart)
            // but the bbox pre-check in computePairOverlap handles this
            double overlap = computePairOverlap(sol.items[i], sol.items[j]);
            if (overlap > 0.01) {
                sol.totalLoss += overlap;
                int64_t pk = pairKey(i, j);
                auto it = weights.find(pk);
                double w = (it != weights.end()) ? it->second.weight : 1.0;
                sol.weightedLoss += overlap * w;
            }
        }
    }

    sol.feasible = (sol.totalLoss < 0.1);
}

double OverlapResolution::evaluateMultiSheetItemLoss(
    const Solution& sol, int itemIdx, const ItemState& candidate,
    const std::vector<SheetRegion>& regions,
    const std::unordered_map<int64_t, CollisionWeight>& weights,
    const SpatialGrid& grid)
{
    double loss = 0;

    loss += computeMultiRegionViolation(candidate, regions) * 2.0;

    auto neighbors = grid.getNeighbors(itemIdx);
    for (int j : neighbors) {
        if (j == itemIdx) continue;
        double overlap = computePairOverlap(candidate, sol.items[j]);
        if (overlap > 0.01) {
            int64_t pk = pairKey(itemIdx, j);
            auto it = weights.find(pk);
            double w = (it != weights.end()) ? it->second.weight : 1.0;
            loss += overlap * w;
        }
    }

    return loss;
}

// ============================================================================
// Multi-Sheet: Initialization
// ============================================================================

OverlapResolution::Solution OverlapResolution::initializeMultiSheetSolution(
    const std::vector<std::shared_ptr<NFP>>& parts,
    const std::vector<SheetRegion>& regions,
    std::mt19937& rng)
{
    Solution sol;
    int n = static_cast<int>(parts.size());
    int K = static_cast<int>(regions.size());

    // Sort parts by area (largest first) for balanced distribution
    std::vector<std::pair<double, int>> areaOrder;
    for (int i = 0; i < n; i++) {
        double area = std::fabs(GeometryUtil::polygonArea(*parts[i]));
        areaOrder.push_back({area, i});
    }
    std::sort(areaOrder.rbegin(), areaOrder.rend());

    sol.items.resize(n);

    // Per-sheet shelf packing state with rotation awareness
    struct ShelfState {
        double cursorX = 0;
        double cursorY = 0;
        double rowHeight = 0;
        int itemCount = 0;
    };
    std::vector<ShelfState> shelves(K);
    std::vector<double> sheetAreaUsed(K, 0);

    // Rotation options
    float rotOptions[] = {0, 90, 180, 270};

    for (auto& [area, partIdx] : areaOrder) {
        // Find best sheet: least loaded
        int bestSheet = 0;
        for (int s = 1; s < K; s++) {
            if (sheetAreaUsed[s] < sheetAreaUsed[bestSheet])
                bestSheet = s;
        }

        auto& sh = shelves[bestSheet];
        auto& r = regions[bestSheet];

        // Try each rotation, pick the one with best fit in current shelf row
        float bestRot = 0;
        double bestFitWidth = 1e9;

        for (float rot : rotOptions) {
            double rad = rot * M_PI / 180.0;
            double cosA = std::cos(rad), sinA = std::sin(rad);
            double rxMin = 1e9, rxMax = -1e9, ryMin = 1e9, ryMax = -1e9;
            for (int pi = 0; pi < parts[partIdx]->length(); pi++) {
                double px = parts[partIdx]->Points[pi].x;
                double py = parts[partIdx]->Points[pi].y;
                double rx = px * cosA - py * sinA;
                double ry = px * sinA + py * cosA;
                rxMin = std::min(rxMin, rx); rxMax = std::max(rxMax, rx);
                ryMin = std::min(ryMin, ry); ryMax = std::max(ryMax, ry);
            }
            double rotW = rxMax - rxMin, rotH = ryMax - ryMin;

            // Does it fit in the current row?
            if (sh.cursorX + rotW <= r.width && sh.cursorY + rotH <= r.height) {
                if (rotW < bestFitWidth) {
                    bestFitWidth = rotW;
                    bestRot = rot;
                }
            }
        }

        // If nothing fits in current row, start new row
        if (bestFitWidth > 1e8) {
            sh.cursorY += sh.rowHeight + 5;
            sh.cursorX = 0;
            sh.rowHeight = 0;
            bestRot = 0;
            bestFitWidth = 1e9;

            for (float rot : rotOptions) {
                double rad = rot * M_PI / 180.0;
                double cosA = std::cos(rad), sinA = std::sin(rad);
                double rxMin = 1e9, rxMax = -1e9, ryMin = 1e9, ryMax = -1e9;
                for (int pi = 0; pi < parts[partIdx]->length(); pi++) {
                    double px = parts[partIdx]->Points[pi].x;
                    double py = parts[partIdx]->Points[pi].y;
                    double rx = px * cosA - py * sinA;
                    double ry = px * sinA + py * cosA;
                    rxMin = std::min(rxMin, rx); rxMax = std::max(rxMax, rx);
                    ryMin = std::min(ryMin, ry); ryMax = std::max(ryMax, ry);
                }
                double rotW = rxMax - rxMin;

                if (rotW < bestFitWidth) {
                    bestFitWidth = rotW;
                    bestRot = rot;
                }
            }
        }

        // Compute actual bounds for chosen rotation
        double rad = bestRot * M_PI / 180.0;
        double cosA = std::cos(rad), sinA = std::sin(rad);
        double rxMin = 1e9, rxMax = -1e9, ryMin = 1e9, ryMax = -1e9;
        for (int pi = 0; pi < parts[partIdx]->length(); pi++) {
            double px = parts[partIdx]->Points[pi].x;
            double py = parts[partIdx]->Points[pi].y;
            double rx = px * cosA - py * sinA;
            double ry = px * sinA + py * cosA;
            rxMin = std::min(rxMin, rx); rxMax = std::max(rxMax, rx);
            ryMin = std::min(ryMin, ry); ryMax = std::max(ryMax, ry);
        }
        double rotW = rxMax - rxMin, rotH = ryMax - ryMin;

        ItemState& item = sol.items[partIdx];
        item.partIndex = partIdx;
        item.polygon = parts[partIdx];
        item.rotation = bestRot;
        item.sheetIdx = bestSheet;
        item.x = r.x + sh.cursorX;
        item.y = r.y + sh.cursorY;

        sh.cursorX += rotW + 5;
        sh.rowHeight = std::max(sh.rowHeight, rotH);
        sh.itemCount++;
        sheetAreaUsed[bestSheet] += rotW * rotH;
    }

    sol.stripWidth = 0;
    sol.totalLoss = 0;
    sol.weightedLoss = 0;
    sol.feasible = false;

    return sol;
}

// ============================================================================
// Multi-Sheet: Position sampling
// ============================================================================

std::vector<OverlapResolution::ItemState> OverlapResolution::sampleMultiSheetPositions(
    const Solution& sol, int itemIdx,
    const std::vector<SheetRegion>& regions,
    int containerSamples, int focusedSamples,
    std::mt19937& rng)
{
    std::vector<ItemState> candidates;
    auto& origItem = sol.items[itemIdx];

    int K = static_cast<int>(regions.size());

    // Rotation angles to try (critical for concave shapes like arches)
    std::vector<float> rotations = {0, 90, 180, 270};

    // Container-wide samples: distribute across all sheets × rotations
    int samplesPerSheetRot = std::max(1, containerSamples / (K * static_cast<int>(rotations.size())));
    for (int s = 0; s < K; s++) {
        auto& r = regions[s];
        for (float rot : rotations) {
            // Compute bounds for this rotation
            NFP tempPoly;
            double rad = rot * M_PI / 180.0;
            double cosA = std::cos(rad), sinA = std::sin(rad);
            double rxMin = 1e9, rxMax = -1e9, ryMin = 1e9, ryMax = -1e9;
            for (int pi = 0; pi < origItem.polygon->length(); pi++) {
                double px = origItem.polygon->Points[pi].x;
                double py = origItem.polygon->Points[pi].y;
                double rx = px * cosA - py * sinA;
                double ry = px * sinA + py * cosA;
                rxMin = std::min(rxMin, rx); rxMax = std::max(rxMax, rx);
                ryMin = std::min(ryMin, ry); ryMax = std::max(ryMax, ry);
            }
            double rotW = rxMax - rxMin, rotH = ryMax - ryMin;

            double minX = r.x;
            double maxX = r.x + r.width - rotW;
            double minY = r.y;
            double maxY = r.y + r.height - rotH;
            if (maxX < minX) maxX = minX;
            if (maxY < minY) maxY = minY;

            std::uniform_real_distribution<double> distX(minX, maxX);
            std::uniform_real_distribution<double> distY(minY, maxY);

            for (int i = 0; i < samplesPerSheetRot; i++) {
                ItemState cand = origItem;
                cand.x = distX(rng);
                cand.y = distY(rng);
                cand.rotation = rot;
                cand.sheetIdx = s;
                candidates.push_back(cand);
            }
        }
    }

    // Focused samples: near current position, same sheet, multiple rotations
    int curSheet = origItem.sheetIdx;
    if (curSheet >= 0 && curSheet < K) {
        auto& r = regions[curSheet];
        double focusRadius = std::min(r.width, r.height) / 4.0;
        std::normal_distribution<double> normalDist(0, focusRadius);

        int samplesPerRot = std::max(1, focusedSamples / static_cast<int>(rotations.size()));
        for (float rot : rotations) {
            // Bounds for this rotation
            double rad = rot * M_PI / 180.0;
            double cosA = std::cos(rad), sinA = std::sin(rad);
            double rxMin = 1e9, rxMax = -1e9, ryMin = 1e9, ryMax = -1e9;
            for (int pi = 0; pi < origItem.polygon->length(); pi++) {
                double px = origItem.polygon->Points[pi].x;
                double py = origItem.polygon->Points[pi].y;
                double rx = px * cosA - py * sinA;
                double ry = px * sinA + py * cosA;
                rxMin = std::min(rxMin, rx); rxMax = std::max(rxMax, rx);
                ryMin = std::min(ryMin, ry); ryMax = std::max(ryMax, ry);
            }
            double rotW = rxMax - rxMin, rotH = ryMax - ryMin;

            double minX = r.x;
            double maxX = r.x + r.width - rotW;
            double minY = r.y;
            double maxY = r.y + r.height - rotH;
            if (maxX < minX) maxX = minX;
            if (maxY < minY) maxY = minY;

            for (int i = 0; i < samplesPerRot; i++) {
                ItemState cand = origItem;
                cand.x = std::clamp(origItem.x + normalDist(rng), minX, maxX);
                cand.y = std::clamp(origItem.y + normalDist(rng), minY, maxY);
                cand.rotation = rot;
                cand.sheetIdx = curSheet;
                candidates.push_back(cand);
            }
        }
    }

    return candidates;
}

// ============================================================================
// Multi-Sheet: Coordinate descent
// ============================================================================

OverlapResolution::ItemState OverlapResolution::coordinateDescentMS(
    const Solution& sol, int itemIdx, const ItemState& start,
    const std::vector<SheetRegion>& regions,
    const std::unordered_map<int64_t, CollisionWeight>& weights,
    const SpatialGrid& grid,
    double stepInit, double stepLimit,
    double rotStepInit, double rotStepLimit)
{
    ItemState current = start;
    double currentLoss = evaluateMultiSheetItemLoss(sol, itemIdx, current, regions, weights, grid);

    double stepX = stepInit, stepY = stepInit, stepRot = rotStepInit;
    const double successMul = 1.1, failMul = 0.5;

    std::vector<CDAxis> axes = {
        CDAxis::Horizontal, CDAxis::Vertical,
        CDAxis::ForwardDiag, CDAxis::BackwardDiag,
        CDAxis::Rotation
    };
    int axisIdx = 0;

    for (int iter = 0; iter < 200; iter++) {
        if (stepX < stepLimit && stepY < stepLimit && stepRot < rotStepLimit)
            break;

        CDAxis axis = axes[axisIdx % axes.size()];
        ItemState cand1 = current, cand2 = current;

        switch (axis) {
            case CDAxis::Horizontal:  cand1.x += stepX; cand2.x -= stepX; break;
            case CDAxis::Vertical:    cand1.y += stepY; cand2.y -= stepY; break;
            case CDAxis::ForwardDiag:
                cand1.x += stepX * 0.7071; cand1.y += stepY * 0.7071;
                cand2.x -= stepX * 0.7071; cand2.y -= stepY * 0.7071;
                break;
            case CDAxis::BackwardDiag:
                cand1.x -= stepX * 0.7071; cand1.y += stepY * 0.7071;
                cand2.x += stepX * 0.7071; cand2.y -= stepY * 0.7071;
                break;
            case CDAxis::Rotation:
                cand1.rotation += static_cast<float>(stepRot);
                cand2.rotation -= static_cast<float>(stepRot);
                break;
        }

        double loss1 = evaluateMultiSheetItemLoss(sol, itemIdx, cand1, regions, weights, grid);
        double loss2 = evaluateMultiSheetItemLoss(sol, itemIdx, cand2, regions, weights, grid);
        double bestCandLoss = std::min(loss1, loss2);
        ItemState& bestCand = (loss1 <= loss2) ? cand1 : cand2;

        if (bestCandLoss < currentLoss - 0.001) {
            current = bestCand;
            currentLoss = bestCandLoss;
            switch (axis) {
                case CDAxis::Horizontal: stepX *= successMul; break;
                case CDAxis::Vertical: stepY *= successMul; break;
                case CDAxis::ForwardDiag:
                case CDAxis::BackwardDiag:
                    stepX *= std::sqrt(successMul); stepY *= std::sqrt(successMul); break;
                case CDAxis::Rotation: stepRot *= successMul; break;
            }
        } else {
            switch (axis) {
                case CDAxis::Horizontal: stepX *= failMul; break;
                case CDAxis::Vertical: stepY *= failMul; break;
                case CDAxis::ForwardDiag:
                case CDAxis::BackwardDiag:
                    stepX *= std::sqrt(failMul); stepY *= std::sqrt(failMul); break;
                case CDAxis::Rotation: stepRot *= failMul; break;
            }
            axisIdx++;
        }
    }

    return current;
}

// ============================================================================
// Multi-Sheet: Move item
// ============================================================================

bool OverlapResolution::moveItemMS(
    Solution& sol, int itemIdx,
    const std::vector<SheetRegion>& regions,
    const std::unordered_map<int64_t, CollisionWeight>& weights,
    const SpatialGrid& grid,
    const SolverConfig& config,
    std::mt19937& rng)
{
    double currentLoss = evaluateMultiSheetItemLoss(
        sol, itemIdx, sol.items[itemIdx], regions, weights, grid);
    if (currentLoss < 0.01) return false;

    auto candidates = sampleMultiSheetPositions(
        sol, itemIdx, regions,
        config.orContainerSamples, config.orFocusedSamples, rng);

    std::vector<std::pair<double, int>> scores;
    for (size_t i = 0; i < candidates.size(); i++) {
        double loss = evaluateMultiSheetItemLoss(
            sol, itemIdx, candidates[i], regions, weights, grid);
        scores.push_back({loss, static_cast<int>(i)});
    }

    std::sort(scores.begin(), scores.end());
    int topN = std::min(config.orCoordDescentTopN, static_cast<int>(scores.size()));

    PolygonBounds partBounds = GeometryUtil::getPolygonBounds(*sol.items[itemIdx].polygon);
    double minDim = std::min(partBounds.width, partBounds.height);
    if (minDim < 1.0) minDim = 1.0;

    ItemState bestCandidate = sol.items[itemIdx];
    double bestLoss = currentLoss;

    for (int i = 0; i < topN; i++) {
        auto refined = coordinateDescentMS(
            sol, itemIdx, candidates[scores[i].second], regions, weights, grid,
            0.25 * minDim, 0.02 * minDim);
        double loss = evaluateMultiSheetItemLoss(sol, itemIdx, refined, regions, weights, grid);
        if (loss < bestLoss) {
            bestLoss = loss;
            bestCandidate = refined;
        }
    }

    if (bestLoss < currentLoss) {
        auto fineRefined = coordinateDescentMS(
            sol, itemIdx, bestCandidate, regions, weights, grid,
            0.01 * minDim, 0.001 * minDim, 0.5, 0.05);
        double fineLoss = evaluateMultiSheetItemLoss(sol, itemIdx, fineRefined, regions, weights, grid);
        if (fineLoss < bestLoss) {
            bestCandidate = fineRefined;
            bestLoss = fineLoss;
        }
    }

    if (bestLoss < currentLoss - 0.001) {
        sol.items[itemIdx] = bestCandidate;
        return true;
    }

    return false;
}

// ============================================================================
// Multi-Sheet: GLS weights
// ============================================================================

void OverlapResolution::updateGLSWeightsMS(
    const Solution& sol,
    const std::vector<SheetRegion>& regions,
    std::unordered_map<int64_t, CollisionWeight>& weights,
    const SolverConfig& config)
{
    int n = static_cast<int>(sol.items.size());
    double maxLoss = 0;

    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            double overlap = computePairOverlap(sol.items[i], sol.items[j]);
            int64_t pk = pairKey(i, j);
            weights[pk].loss = overlap;
            maxLoss = std::max(maxLoss, overlap);
        }
    }

    for (auto& [pk, cw] : weights) {
        if (cw.loss < 0.01) {
            cw.weight *= config.glsWeightDecay;
            if (cw.weight < 1.0) cw.weight = 1.0;
        } else if (maxLoss > 0.01) {
            double relSeverity = cw.loss / maxLoss;
            double multiplier = config.glsWeightMinInc +
                               (config.glsWeightMaxInc - config.glsWeightMinInc) * relSeverity;
            cw.weight *= multiplier;
        }
    }
}

// ============================================================================
// Multi-Sheet: Disruption
// ============================================================================

void OverlapResolution::disruptMultiSheet(
    Solution& sol,
    const std::vector<SheetRegion>& regions,
    const SolverConfig& config,
    std::mt19937& rng)
{
    if (sol.items.size() < 2) return;

    int n = static_cast<int>(sol.items.size());
    int K = static_cast<int>(regions.size());

    // 50% chance: swap two items between different sheets
    // 50% chance: swap two items within same sheet (position swap)
    std::uniform_real_distribution<double> coinFlip(0, 1);
    std::uniform_int_distribution<int> itemDist(0, n - 1);

    if (coinFlip(rng) < 0.5 && K > 1) {
        // Cross-sheet swap: pick two items on different sheets
        int idx1 = itemDist(rng);
        int tries = 0;
        int idx2 = itemDist(rng);
        while (sol.items[idx2].sheetIdx == sol.items[idx1].sheetIdx && tries < 20) {
            idx2 = itemDist(rng);
            tries++;
        }
        if (sol.items[idx2].sheetIdx != sol.items[idx1].sheetIdx) {
            // Swap sheet assignments and reposition within new sheet
            int s1 = sol.items[idx1].sheetIdx;
            int s2 = sol.items[idx2].sheetIdx;

            // Move item 1 to sheet of item 2, centered
            auto& r2 = regions[s2];
            PolygonBounds b1 = GeometryUtil::getPolygonBounds(*sol.items[idx1].polygon);
            sol.items[idx1].x = r2.x + (r2.width - b1.width) / 2.0;
            sol.items[idx1].y = r2.y + (r2.height - b1.height) / 2.0;
            sol.items[idx1].sheetIdx = s2;

            // Move item 2 to sheet of item 1, centered
            auto& r1 = regions[s1];
            PolygonBounds b2 = GeometryUtil::getPolygonBounds(*sol.items[idx2].polygon);
            sol.items[idx2].x = r1.x + (r1.width - b2.width) / 2.0;
            sol.items[idx2].y = r1.y + (r1.height - b2.height) / 2.0;
            sol.items[idx2].sheetIdx = s1;
        }
    } else {
        // Within-sheet position swap (same as original disrupt)
        disruptSolution(sol, config, rng);
    }
}

// ============================================================================
// Multi-Sheet: Separation loop
// ============================================================================

bool OverlapResolution::separateMS(
    Solution& sol,
    const std::vector<SheetRegion>& regions,
    std::unordered_map<int64_t, CollisionWeight>& weights,
    SpatialGrid& grid,
    const SolverConfig& config,
    std::mt19937& rng,
    int strikeLimit,
    int iterNoImproveLimit)
{
    int strikes = 0;
    int iterNoImprove = 0;
    double bestLoss = std::numeric_limits<double>::max();

    double medianDim = 100;
    if (!sol.items.empty()) {
        std::vector<double> dims;
        for (auto& item : sol.items) {
            PolygonBounds b = GeometryUtil::getPolygonBounds(*item.polygon);
            dims.push_back(std::max(b.width, b.height));
        }
        std::sort(dims.begin(), dims.end());
        // Use median dimension for grid cell size (not max — max makes cells too large)
        medianDim = dims[dims.size() / 2];
    }

    auto separateStart = std::chrono::high_resolution_clock::now();
    double maxSeparateTime = std::max(15.0, config.timeBudgetSeconds * 0.3);

    for (int iter = 0; iter < 50000; iter++) {
        auto now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(now - separateStart).count();
        if (elapsed > maxSeparateTime) return sol.feasible;

        grid.build(sol.items, medianDim);
        evaluateMultiSheetSolution(sol, regions, weights);

        if (sol.feasible) return true;

        // Progress logging every 50 iterations
        if (iter % 50 == 0) {
            std::cout << "    sep iter " << iter << " loss=" << std::fixed
                      << std::setprecision(0) << sol.totalLoss
                      << " colliding=";
            int numColliding = 0;
            for (size_t i = 0; i < sol.items.size(); i++) {
                double itemLoss = evaluateMultiSheetItemLoss(
                    sol, static_cast<int>(i), sol.items[i], regions, weights, grid);
                if (itemLoss > 0.01) numColliding++;
            }
            std::cout << numColliding << "/" << sol.items.size()
                      << " t=" << std::setprecision(1) << elapsed << "s" << std::endl;
        }

        if (sol.totalLoss < bestLoss - 0.01) {
            bestLoss = sol.totalLoss;
            iterNoImprove = 0;
        } else {
            iterNoImprove++;
        }

        if (iterNoImprove >= iterNoImproveLimit) {
            strikes++;
            iterNoImprove = 0;
            updateGLSWeightsMS(sol, regions, weights, config);
            std::cout << "    strike " << strikes << "/" << strikeLimit
                      << " (loss=" << std::setprecision(0) << bestLoss << ")" << std::endl;

            if (strikes >= strikeLimit) return false;
        }

        std::vector<int> collidingItems;
        for (size_t i = 0; i < sol.items.size(); i++) {
            double itemLoss = evaluateMultiSheetItemLoss(
                sol, static_cast<int>(i), sol.items[i], regions, weights, grid);
            if (itemLoss > 0.01) {
                collidingItems.push_back(static_cast<int>(i));
            }
        }

        if (collidingItems.empty()) {
            sol.feasible = true;
            return true;
        }

        std::shuffle(collidingItems.begin(), collidingItems.end(), rng);

        for (int idx : collidingItems) {
            moveItemMS(sol, idx, regions, weights, grid, config, rng);
        }
    }

    return sol.feasible;
}

// ============================================================================
// Multi-Sheet: Main solver
// ============================================================================

OverlapResolution::Solution OverlapResolution::solveMultiSheet(
    const std::vector<std::shared_ptr<NFP>>& parts,
    const NFP& sheetTemplate,
    int maxSheets,
    const SolverConfig& config,
    std::function<void(int, int, double)> progressCallback)
{
    std::mt19937 rng(42);

    PolygonBounds sheetBounds = GeometryUtil::getPolygonBounds(sheetTemplate);
    double sheetArea = sheetBounds.width * sheetBounds.height;

    // Compute total parts area to estimate minimum sheets
    double totalPartsArea = 0;
    for (auto& p : parts) {
        totalPartsArea += std::fabs(GeometryUtil::polygonArea(*p));
    }

    // Start with generous estimate (80% target density), then go up if needed
    int minK = std::max(1, static_cast<int>(std::ceil(totalPartsArea / sheetArea)));
    int startK = std::max(minK, static_cast<int>(std::ceil(totalPartsArea / (sheetArea * 0.65))));
    startK = std::min(startK, maxSheets);

    std::cout << "  Multi-sheet OR: partsArea=" << std::fixed << std::setprecision(0)
              << totalPartsArea << " sheetArea=" << sheetArea
              << " minSheets=" << minK << " startK=" << startK << std::endl;

    auto startTime = std::chrono::high_resolution_clock::now();
    double totalTime = config.timeBudgetSeconds;

    Solution bestFeasible;
    bestFeasible.feasible = false;
    bestFeasible.totalLoss = std::numeric_limits<double>::max();
    int bestSheetsUsed = maxSheets + 1;

    // Strategy: start with startK sheets. If feasible, try fewer. If not, try more.
    // Phase 1: Find any feasible solution (start at startK, increase if needed)
    for (int numSheets = startK; numSheets <= maxSheets; numSheets++) {
        auto now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(now - startTime).count();
        if (elapsed > totalTime * 0.5) break;

        std::cout << "  Trying " << numSheets << " sheets..." << std::endl;

        auto regions = createSheetLayout(sheetTemplate, numSheets, 200.0);
        std::unordered_map<int64_t, CollisionWeight> weights;
        SpatialGrid grid;

        Solution current = initializeMultiSheetSolution(parts, regions, rng);

        // Evaluate initial
        evaluateMultiSheetSolution(current, regions, weights);
        std::cout << "    Init loss=" << std::setprecision(0) << current.totalLoss << std::endl;

        if (progressCallback) {
            progressCallback(numSheets, 0, current.totalLoss);
        }

        bool feasible = separateMS(current, regions, weights, grid, config, rng,
                                    config.orStrikeLimit, config.orIterNoImproveLimit);

        if (feasible) {
            bestFeasible = current;
            bestSheetsUsed = numSheets;
            std::cout << "  FEASIBLE at " << numSheets << " sheets!" << std::endl;

            if (progressCallback) {
                progressCallback(numSheets, 1, current.totalLoss);
            }
            break; // Found feasible, now try reducing
        } else {
            std::cout << "  Failed at " << numSheets << " sheets (loss="
                      << std::setprecision(0) << current.totalLoss << ")" << std::endl;
        }
    }

    // Phase 2: Try reducing sheet count from best feasible
    if (bestFeasible.feasible) {
        for (int numSheets = bestSheetsUsed - 1; numSheets >= minK; numSheets--) {
            auto now = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double>(now - startTime).count();
            if (elapsed > totalTime * 0.95) break;

            std::cout << "  Compressing to " << numSheets << " sheets..." << std::endl;

            auto regions = createSheetLayout(sheetTemplate, numSheets, 200.0);
            std::unordered_map<int64_t, CollisionWeight> weights;
            SpatialGrid grid;

            // Reinitialize for fewer sheets
            Solution current = initializeMultiSheetSolution(parts, regions, rng);

            evaluateMultiSheetSolution(current, regions, weights);
            std::cout << "    Init loss=" << std::setprecision(0) << current.totalLoss << std::endl;

            // Multiple attempts with disruption
            bool feasible = false;
            int maxAttempts = 3;
            for (int attempt = 0; attempt < maxAttempts; attempt++) {
                auto attNow = std::chrono::high_resolution_clock::now();
                double attElapsed = std::chrono::duration<double>(attNow - startTime).count();
                if (attElapsed > totalTime * 0.95) break;

                feasible = separateMS(current, regions, weights, grid, config, rng,
                                       config.orStrikeLimit, config.orIterNoImproveLimit);

                if (feasible) break;

                std::cout << "    Attempt " << (attempt + 1) << " failed (loss="
                          << std::setprecision(0) << current.totalLoss
                          << "), disrupting..." << std::endl;
                disruptMultiSheet(current, regions, config, rng);
            }

            if (feasible) {
                bestFeasible = current;
                bestSheetsUsed = numSheets;
                std::cout << "  FEASIBLE at " << numSheets << " sheets!" << std::endl;

                if (progressCallback) {
                    progressCallback(numSheets, 1, current.totalLoss);
                }
            } else {
                std::cout << "  Cannot compress to " << numSheets << " sheets" << std::endl;
                break; // If can't do K-1, definitely can't do K-2
            }
        }
    }

    if (bestFeasible.feasible) {
        // Update sheet indices based on final positions
        auto regions = createSheetLayout(sheetTemplate, bestSheetsUsed, 200.0);
        for (auto& item : bestFeasible.items) {
            item.sheetIdx = nearestSheet(item, regions);
        }
    }

    std::cout << "  Multi-sheet OR result: "
              << (bestFeasible.feasible ? "feasible" : "INFEASIBLE")
              << " sheets=" << bestSheetsUsed << std::endl;

    return bestFeasible;
}

} // namespace nest
