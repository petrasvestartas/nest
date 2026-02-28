#include "Compaction.h"
#include <iostream>
#include <cstring>

namespace nest {

namespace {
    Clipper2Lib::Path64 nfpToPath64(const NFP& poly, double scale = 1e7) {
        Clipper2Lib::Path64 path;
        path.reserve(poly.length());
        for (int i = 0; i < poly.length(); i++) {
            path.push_back(Clipper2Lib::Point64(
                static_cast<int64_t>(std::round(poly[i].x * scale)),
                static_cast<int64_t>(std::round(poly[i].y * scale))));
        }
        return path;
    }
} // anonymous namespace

std::vector<Compaction::SlideDirection> Compaction::getStandardDirections() {
    return {
        { -1.0,  0.0, "left" },
        {  0.0, -1.0, "down" },
        { -0.7071, -0.7071, "diagonal-LD" },
        {  0.0,  1.0, "up" },
        {  1.0,  0.0, "right" },
        {  0.7071, -0.7071, "diagonal-RD" },
    };
}

NFP Compaction::getShiftedPolygon(const NFP& piece, double px, double py, float rotation) {
    double rad = rotation * M_PI / 180.0;
    double cosA = std::cos(rad);
    double sinA = std::sin(rad);

    NFP shifted;
    shifted.Points.reserve(piece.length());
    for (int i = 0; i < piece.length(); i++) {
        double rx = piece[i].x * cosA - piece[i].y * sinA + px;
        double ry = piece[i].x * sinA + piece[i].y * cosA + py;
        shifted.AddPoint(Point(rx, ry));
    }
    return shifted;
}

double Compaction::computeOverlapArea(const NFP& polyA, const NFP& polyB) {
    const double scale = 1e7;
    Clipper2Lib::Path64 pathA = nfpToPath64(polyA, scale);
    Clipper2Lib::Path64 pathB = nfpToPath64(polyB, scale);

    Clipper2Lib::Clipper64 clipper;
    clipper.AddSubject({pathA});
    clipper.AddClip({pathB});

    Clipper2Lib::Paths64 solution;
    clipper.Execute(Clipper2Lib::ClipType::Intersection,
                    Clipper2Lib::FillRule::NonZero, solution);

    double area = 0;
    for (auto& path : solution) {
        area += std::fabs(Clipper2Lib::Area(path));
    }
    return area / (scale * scale);
}

bool Compaction::isInsideSheet(
    int pieceIdx, double testX, double testY,
    const std::vector<std::shared_ptr<NFP>>& placed,
    const NFP& sheet)
{
    NFP shifted = getShiftedPolygon(*placed[pieceIdx], testX, testY,
                                     0); // rotation already baked into placed points

    // Check all vertices are inside sheet bounds
    PolygonBounds sheetBounds = GeometryUtil::getPolygonBounds(sheet);
    for (int i = 0; i < shifted.length(); i++) {
        auto pip = GeometryUtil::pointInPolygon(shifted[i], sheet);
        if (pip.has_value() && pip.value() == false) {
            return false;
        }
        // Also check bounding box as fast reject
        if (shifted[i].x < sheetBounds.x - 0.01 ||
            shifted[i].x > sheetBounds.x + sheetBounds.width + 0.01 ||
            shifted[i].y < sheetBounds.y - 0.01 ||
            shifted[i].y > sheetBounds.y + sheetBounds.height + 0.01) {
            return false;
        }
    }
    return true;
}

bool Compaction::hasOverlap(
    int pieceIdx, double testX, double testY,
    const std::vector<std::shared_ptr<NFP>>& placed,
    const std::vector<PlacementItem>& placements,
    const NFP& sheet)
{
    // Build shifted polygon for the test piece
    NFP testPoly = getShiftedPolygon(*placed[pieceIdx], testX, testY, 0);

    // Check sheet boundary first
    PolygonBounds sheetBounds = GeometryUtil::getPolygonBounds(sheet);
    PolygonBounds testBounds = GeometryUtil::getPolygonBounds(testPoly);

    if (testBounds.x < sheetBounds.x - 0.1 ||
        testBounds.x + testBounds.width > sheetBounds.x + sheetBounds.width + 0.1 ||
        testBounds.y < sheetBounds.y - 0.1 ||
        testBounds.y + testBounds.height > sheetBounds.y + sheetBounds.height + 0.1) {
        return true;
    }

    // Check against all other placed pieces
    for (size_t j = 0; j < placed.size(); j++) {
        if (static_cast<int>(j) == pieceIdx) continue;

        NFP otherPoly = getShiftedPolygon(*placed[j], placements[j].x, placements[j].y, 0);

        // Bounding box pre-check
        PolygonBounds otherBounds = GeometryUtil::getPolygonBounds(otherPoly);
        if (testBounds.x > otherBounds.x + otherBounds.width + 0.1 ||
            otherBounds.x > testBounds.x + testBounds.width + 0.1 ||
            testBounds.y > otherBounds.y + otherBounds.height + 0.1 ||
            otherBounds.y > testBounds.y + testBounds.height + 0.1) {
            continue;
        }

        // Precise overlap check
        double overlap = computeOverlapArea(testPoly, otherPoly);
        if (overlap > 0.01) {
            return true;
        }
    }
    return false;
}

double Compaction::slideUntilCollision(
    int pieceIdx,
    const SlideDirection& direction,
    const std::vector<std::shared_ptr<NFP>>& placed,
    const std::vector<PlacementItem>& placements,
    const NFP& sheet,
    double stepMin)
{
    double curX = placements[pieceIdx].x;
    double curY = placements[pieceIdx].y;

    // Exponential probing: find upper bound
    double step = 1.0;
    double maxDist = 0;

    // Compute max possible distance (sheet diagonal)
    PolygonBounds sb = GeometryUtil::getPolygonBounds(sheet);
    double maxPossible = std::sqrt(sb.width * sb.width + sb.height * sb.height);

    // Exponential probing phase
    while (step < maxPossible) {
        double testX = curX + direction.dx * step;
        double testY = curY + direction.dy * step;

        if (hasOverlap(pieceIdx, testX, testY, placed, placements, sheet)) {
            break;
        }
        maxDist = step;
        step *= 2.0;
    }

    if (maxDist <= 0 && step <= 1.0) {
        // Can't move at all
        return 0;
    }

    // Binary search between maxDist and step (or maxDist and step)
    double lo = maxDist;
    double hi = std::min(step, maxPossible);

    while (hi - lo > stepMin) {
        double mid = (lo + hi) / 2.0;
        double testX = curX + direction.dx * mid;
        double testY = curY + direction.dy * mid;

        if (hasOverlap(pieceIdx, testX, testY, placed, placements, sheet)) {
            hi = mid;
        } else {
            lo = mid;
        }
    }

    return lo;
}

double Compaction::compactInDirection(
    const SlideDirection& direction,
    std::vector<std::shared_ptr<NFP>>& placed,
    std::vector<PlacementItem>& placements,
    const NFP& sheet,
    double stepMin)
{
    double totalMoved = 0;

    // Sort pieces by position along the direction (process farthest-along first)
    std::vector<int> order(placements.size());
    for (size_t i = 0; i < order.size(); i++) order[i] = static_cast<int>(i);

    std::sort(order.begin(), order.end(), [&](int a, int b) {
        double projA = placements[a].x * (-direction.dx) + placements[a].y * (-direction.dy);
        double projB = placements[b].x * (-direction.dx) + placements[b].y * (-direction.dy);
        return projA < projB; // Farthest from target direction first
    });

    for (int idx : order) {
        double dist = slideUntilCollision(idx, direction, placed, placements, sheet, stepMin);
        if (dist > stepMin) {
            placements[idx].x += direction.dx * dist;
            placements[idx].y += direction.dy * dist;
            totalMoved += dist;
        }
    }

    return totalMoved;
}

double Compaction::compact(
    std::vector<std::shared_ptr<NFP>>& placed,
    std::vector<PlacementItem>& placements,
    const NFP& sheet,
    const SolverConfig& config)
{
    if (placed.empty() || placements.empty()) return 0;

    double totalMoved = 0;
    double stepMin = config.compactionStepMin;

    // Standard compaction: left, down, diagonal
    std::vector<SlideDirection> directions = {
        { -1.0,  0.0, "left" },
        {  0.0, -1.0, "down" },
        { -0.7071, -0.7071, "diagonal-LD" },
    };

    for (int round = 0; round < config.compactionRounds; round++) {
        double roundMoved = 0;
        for (auto& dir : directions) {
            roundMoved += compactInDirection(dir, placed, placements, sheet, stepMin);
        }
        totalMoved += roundMoved;

        // Early exit if no significant movement
        if (roundMoved < stepMin * placements.size()) {
            break;
        }
    }

    return totalMoved;
}

} // namespace nest
