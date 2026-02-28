#include "ExhaustiveSolver.h"
#include "GeometryUtil.h"
#include "NestingEngine.h"
#include <algorithm>
#include <numeric>
#include <chrono>
#include <iostream>

namespace nest {

long long ExhaustiveSolver::estimateEvaluations(int n, int rotations) {
    // N! * rotations^N
    long long factorial = 1;
    for (int i = 2; i <= n; i++) factorial *= i;

    long long rotCombos = 1;
    for (int i = 0; i < n; i++) rotCombos *= rotations;

    return factorial * rotCombos;
}

bool ExhaustiveSolver::isFeasible(int n, int rotations, double timeBudgetSeconds) {
    long long evals = estimateEvaluations(n, rotations);
    // Rough estimate: ~1000 evals/sec for small polygons
    double estimatedSeconds = static_cast<double>(evals) / 1000.0;
    return estimatedSeconds < timeBudgetSeconds;
}

void ExhaustiveSolver::generatePermutations(
    int n, std::vector<std::vector<int>>& result)
{
    std::vector<int> perm(n);
    std::iota(perm.begin(), perm.end(), 0);

    do {
        result.push_back(perm);
    } while (std::next_permutation(perm.begin(), perm.end()));
}

void ExhaustiveSolver::generateRotationCombinations(
    int n, int numRotations,
    std::vector<std::vector<float>>& result)
{
    float step = 360.0f / numRotations;
    int total = 1;
    for (int i = 0; i < n; i++) total *= numRotations;

    result.reserve(total);
    for (int combo = 0; combo < total; combo++) {
        std::vector<float> rots(n);
        int val = combo;
        for (int i = 0; i < n; i++) {
            rots[i] = (val % numRotations) * step;
            val /= numRotations;
        }
        result.push_back(rots);
    }
}

ExhaustiveSolver::EvalResult ExhaustiveSolver::evaluateSingle(
    const std::vector<std::shared_ptr<NFP>>& parts,
    const std::vector<std::shared_ptr<NFP>>& sheets,
    const std::vector<int>& ordering,
    const std::vector<float>& rotations,
    const SolverConfig& config,
    NfpWorker& worker)
{
    EvalResult result;
    result.ordering = ordering;
    result.rotations = rotations;
    result.fitness = std::numeric_limits<double>::max();
    result.sheetsUsed = 0;

    // Build ordered + rotated parts list
    std::vector<std::shared_ptr<NFP>> orderedParts;
    for (int idx : ordering) {
        auto rotated = std::make_shared<NFP>(
            NfpWorker::rotatePolygon(*parts[idx], rotations[idx]));
        rotated->source = parts[idx]->source;
        rotated->Id = parts[idx]->Id;
        orderedParts.push_back(rotated);
    }

    // Use placeParts for BLF placement
    auto placement = worker.placeParts(sheets, orderedParts, config.baseConfig, 0);

    result.fitness = placement.fitness.value_or(std::numeric_limits<double>::max());

    // Count sheets used
    for (auto& group : placement.placements) {
        result.sheetsUsed += static_cast<int>(group.size());
    }

    return result;
}

ExhaustiveSolver::EvalResult ExhaustiveSolver::solve(
    const std::vector<std::shared_ptr<NFP>>& parts,
    const std::vector<std::shared_ptr<NFP>>& sheets,
    const SolverConfig& config,
    std::function<void(int, int)> progressCallback)
{
    int n = static_cast<int>(parts.size());
    int numRots = config.exhaustiveMaxRotations;

    // Generate all permutations
    std::vector<std::vector<int>> permutations;
    generatePermutations(n, permutations);

    // Generate all rotation combinations
    std::vector<std::vector<float>> rotCombos;
    generateRotationCombinations(n, numRots, rotCombos);

    int totalEvals = static_cast<int>(permutations.size() * rotCombos.size());
    int evalCount = 0;

    EvalResult best;
    best.fitness = std::numeric_limits<double>::max();

    // Single NfpWorker instance — NFP cache shared across all evaluations
    NfpWorker worker;

    auto startTime = std::chrono::high_resolution_clock::now();

    for (auto& perm : permutations) {
        for (auto& rots : rotCombos) {
            // Map rotation indices to actual part rotations
            std::vector<float> partRots(n);
            for (int i = 0; i < n; i++) {
                partRots[perm[i]] = rots[i];
            }

            auto result = evaluateSingle(parts, sheets, perm, partRots, config, worker);
            evalCount++;

            if (result.fitness < best.fitness) {
                best = result;
            }

            if (progressCallback) {
                progressCallback(evalCount, totalEvals);
            }

            // Check time budget
            if (config.timeBudgetSeconds > 0) {
                auto now = std::chrono::high_resolution_clock::now();
                double elapsed = std::chrono::duration<double>(now - startTime).count();
                if (elapsed > config.timeBudgetSeconds) {
                    return best;
                }
            }
        }
    }

    return best;
}

} // namespace nest
