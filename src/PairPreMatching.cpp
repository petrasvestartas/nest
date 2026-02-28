#include "PairPreMatching.h"
#include "GeometryUtil.h"
#include <algorithm>
#include <cmath>
#include <set>

namespace nest {

double PairPreMatching::computeNfpArea(
    NFP& partA, NFP& partB,
    float rotA, float rotB)
{
    // Rotate parts
    NFP rotatedA = NfpWorker::rotatePolygon(partA, rotA);
    NFP rotatedB = NfpWorker::rotatePolygon(partB, rotB);

    // Compute outer NFP (uses thread-local worker for caching)
    static thread_local NfpWorker worker;
    auto outerNfp = worker.getOuterNfp(rotatedA, rotatedB, 0);
    if (!outerNfp) return std::numeric_limits<double>::max();

    return std::fabs(GeometryUtil::polygonArea(*outerNfp));
}

std::vector<PairPreMatching::PairScore> PairPreMatching::computePairScores(
    const std::vector<std::shared_ptr<NFP>>& parts,
    int rotations)
{
    std::vector<PairScore> scores;
    int n = static_cast<int>(parts.size());

    float rotStep = 360.0f / rotations;

    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            double bestNfpArea = std::numeric_limits<double>::max();
            float bestRotA = 0, bestRotB = 0;

            for (int ra = 0; ra < rotations; ra++) {
                float rotA = ra * rotStep;
                for (int rb = 0; rb < rotations; rb++) {
                    float rotB = rb * rotStep;

                    double area = computeNfpArea(*parts[i], *parts[j], rotA, rotB);
                    if (area < bestNfpArea) {
                        bestNfpArea = area;
                        bestRotA = rotA;
                        bestRotB = rotB;
                    }
                }
            }

            // Score: normalized by product of individual areas (scale-invariant)
            double areaA = std::fabs(GeometryUtil::polygonArea(*parts[i]));
            double areaB = std::fabs(GeometryUtil::polygonArea(*parts[j]));
            double normalizer = std::sqrt(areaA * areaB);
            double score = (normalizer > 0) ? bestNfpArea / normalizer : bestNfpArea;

            scores.push_back({i, j, bestRotA, bestRotB, bestNfpArea, score});
        }
    }

    std::sort(scores.begin(), scores.end(),
              [](const PairScore& a, const PairScore& b) { return a.score < b.score; });

    return scores;
}

std::vector<int> PairPreMatching::reorderByPairMatching(
    const std::vector<std::shared_ptr<NFP>>& parts,
    const std::vector<PairScore>& scores)
{
    int n = static_cast<int>(parts.size());
    std::vector<int> order;
    std::set<int> used;

    // Greedy pair grouping: take best pairs first, interleave unpaired
    for (const auto& ps : scores) {
        if (used.count(ps.indexA) || used.count(ps.indexB)) continue;

        order.push_back(ps.indexA);
        order.push_back(ps.indexB);
        used.insert(ps.indexA);
        used.insert(ps.indexB);

        if (static_cast<int>(order.size()) >= n) break;
    }

    // Add remaining unpaired items
    for (int i = 0; i < n; i++) {
        if (!used.count(i)) {
            order.push_back(i);
        }
    }

    return order;
}

std::pair<float, float> PairPreMatching::getBestRotations(
    int indexA, int indexB,
    const std::vector<PairScore>& scores)
{
    for (const auto& ps : scores) {
        if ((ps.indexA == indexA && ps.indexB == indexB) ||
            (ps.indexA == indexB && ps.indexB == indexA)) {
            if (ps.indexA == indexA) {
                return {ps.rotA, ps.rotB};
            } else {
                return {ps.rotB, ps.rotA};
            }
        }
    }
    return {0.0f, 0.0f};
}

} // namespace nest
