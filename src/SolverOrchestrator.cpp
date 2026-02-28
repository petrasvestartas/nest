#include "SolverOrchestrator.h"
#include "NfpWorker.h"
#include "NestingEngine.h"
#include "Compaction.h"
#include "JostleHeuristic.h"
#include "ContinuousRotation.h"
#include "ExhaustiveSolver.h"
#include "OverlapResolution.h"
#include "PairPreMatching.h"
#include <iostream>
#include <algorithm>
#include <numeric>
#include <set>

namespace nest {

std::string SolverOrchestrator::solverName(SolverType type) {
    switch (type) {
        case SolverType::Classic:           return "Classic (GA+BLF)";
        case SolverType::CompactedBLF:      return "Compacted BLF";
        case SolverType::Jostle:            return "Jostle Heuristic";
        case SolverType::Exhaustive:        return "Exhaustive Search";
        case SolverType::OverlapResolution: return "Overlap Resolution";
        case SolverType::SmartBLF:          return "Smart BLF";
        case SolverType::Auto:              return "Auto";
        default:                            return "Unknown";
    }
}

// ============================================================================
// Classic solver (wraps NestingContext)
// ============================================================================

SolverResult SolverOrchestrator::solveClassic(
    const std::vector<std::shared_ptr<NFP>>& parts,
    const std::vector<std::shared_ptr<NFP>>& sheets,
    const SolverConfig& config,
    std::function<void(const std::string&)> progressCallback)
{
    auto startTime = std::chrono::high_resolution_clock::now();

    NestingContext context;
    NestingEngine::Config = config.baseConfig;

    // Add sheets and parts
    for (auto& s : sheets) context.Sheets.push_back(s);
    for (auto& p : parts) context.Polygons.push_back(p);

    NfpWorker::UseParallel = true;
    context.StartNest();

    // Run enough iterations to converge (original needed ~28 for 16-part dataset)
    int maxIter = std::max(30, static_cast<int>(config.timeBudgetSeconds / 2.0));
    for (int i = 0; i < maxIter; i++) {
        context.NestIterate(maxIter);
        if (progressCallback && (i % 10 == 0 || i == maxIter - 1)) {
            progressCallback("Classic iteration " + std::to_string(i + 1) + "/" +
                           std::to_string(maxIter) + " placed=" +
                           std::to_string(context.PlacedPartsCount) + "/" +
                           std::to_string(parts.size()));
        }
        // Early exit if all parts placed
        if (context.PlacedPartsCount == static_cast<int>(parts.size())) break;
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    double elapsedMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();

    // Build result
    SolverResult result;
    result.solverUsed = SolverType::Classic;
    result.solverName = solverName(SolverType::Classic);
    result.fitness = context.HasCurrent() ? context.Current().fitness.value_or(1e9) : 1e9;
    result.materialUtilization = context.MaterialUtilization;
    result.sheetsUsed = static_cast<int>(sheets.size()) - context.SheetsNotUsed;
    result.partsPlaced = context.PlacedPartsCount;
    result.totalParts = static_cast<int>(parts.size());
    result.elapsedMs = elapsedMs;

    for (size_t i = 0; i < context.Polygons.size(); i++) {
        auto& poly = context.Polygons[i];
        SolverResult::PartResult pr;
        pr.partIndex = static_cast<int>(i);
        pr.source = poly->source.value_or(-1);
        pr.x = poly->x;
        pr.y = poly->y;
        pr.rotation = poly->Rotation;
        pr.placed = poly->fitted();
        pr.sheetIndex = poly->sheet ? poly->sheet->Id : -1;
        result.parts.push_back(pr);
    }

    return result;
}

// ============================================================================
// Compacted BLF
// ============================================================================

SolverResult SolverOrchestrator::solveCompactedBLF(
    const std::vector<std::shared_ptr<NFP>>& parts,
    const std::vector<std::shared_ptr<NFP>>& sheets,
    const SolverConfig& config,
    std::function<void(const std::string&)> progressCallback)
{
    auto startTime = std::chrono::high_resolution_clock::now();

    // First run classic to get initial placement
    if (progressCallback) progressCallback("Running BLF placement...");

    auto classicResult = solveClassic(parts, sheets, config, nullptr);

    // Now apply compaction to placed parts
    if (progressCallback) progressCallback("Applying compaction...");

    // Build placed polygon list and placements from classic result
    std::vector<std::shared_ptr<NFP>> placedPolys;
    std::vector<PlacementItem> placements;

    for (auto& pr : classicResult.parts) {
        if (!pr.placed) continue;

        // Create a rotated copy at the placed position
        auto rotated = std::make_shared<NFP>(
            NfpWorker::rotatePolygon(*parts[pr.partIndex], pr.rotation));

        PlacementItem pi;
        pi.id = pr.partIndex;
        pi.source = pr.source;
        pi.x = pr.x;
        pi.y = pr.y;
        pi.rotation = 0; // Rotation already applied to polygon

        placedPolys.push_back(rotated);
        placements.push_back(pi);
    }

    // Apply compaction on each sheet
    if (!sheets.empty() && !placedPolys.empty()) {
        double moved = Compaction::compact(placedPolys, placements, *sheets[0], config);
        if (progressCallback) {
            progressCallback("Compaction moved " + std::to_string(moved) + " total");
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    double elapsedMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();

    // Build result
    SolverResult result = classicResult;
    result.solverUsed = SolverType::CompactedBLF;
    result.solverName = solverName(SolverType::CompactedBLF);
    result.elapsedMs = elapsedMs;

    // Update positions from compaction
    for (size_t i = 0; i < placements.size() && i < result.parts.size(); i++) {
        for (auto& pr : result.parts) {
            if (pr.placed && pr.partIndex == placements[i].id) {
                pr.x = placements[i].x;
                pr.y = placements[i].y;
                break;
            }
        }
    }

    return result;
}

// ============================================================================
// Jostle heuristic
// ============================================================================

SolverResult SolverOrchestrator::solveJostle(
    const std::vector<std::shared_ptr<NFP>>& parts,
    const std::vector<std::shared_ptr<NFP>>& sheets,
    const SolverConfig& config,
    std::function<void(const std::string&)> progressCallback)
{
    auto startTime = std::chrono::high_resolution_clock::now();

    // First run compacted BLF
    if (progressCallback) progressCallback("Running BLF + compaction...");
    auto compactedResult = solveCompactedBLF(parts, sheets, config, nullptr);

    // Now apply jostle
    if (progressCallback) progressCallback("Applying jostle heuristic...");

    std::vector<std::shared_ptr<NFP>> placedPolys;
    std::vector<PlacementItem> placements;

    for (auto& pr : compactedResult.parts) {
        if (!pr.placed) continue;

        auto rotated = std::make_shared<NFP>(
            NfpWorker::rotatePolygon(*parts[pr.partIndex], pr.rotation));

        PlacementItem pi;
        pi.id = pr.partIndex;
        pi.source = pr.source;
        pi.x = pr.x;
        pi.y = pr.y;
        pi.rotation = 0;

        placedPolys.push_back(rotated);
        placements.push_back(pi);
    }

    if (!sheets.empty() && !placedPolys.empty()) {
        double moved = JostleHeuristic::jostle(placedPolys, placements, *sheets[0], config);
        if (progressCallback) {
            progressCallback("Jostle moved " + std::to_string(moved) + " total");
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    double elapsedMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();

    SolverResult result = compactedResult;
    result.solverUsed = SolverType::Jostle;
    result.solverName = solverName(SolverType::Jostle);
    result.elapsedMs = elapsedMs;

    for (size_t i = 0; i < placements.size(); i++) {
        for (auto& pr : result.parts) {
            if (pr.placed && pr.partIndex == placements[i].id) {
                pr.x = placements[i].x;
                pr.y = placements[i].y;
                break;
            }
        }
    }

    return result;
}

// ============================================================================
// Exhaustive solver
// ============================================================================

SolverResult SolverOrchestrator::solveExhaustive(
    const std::vector<std::shared_ptr<NFP>>& parts,
    const std::vector<std::shared_ptr<NFP>>& sheets,
    const SolverConfig& config,
    std::function<void(const std::string&)> progressCallback)
{
    auto startTime = std::chrono::high_resolution_clock::now();

    int n = static_cast<int>(parts.size());

    if (progressCallback) {
        long long evals = ExhaustiveSolver::estimateEvaluations(n, config.exhaustiveMaxRotations);
        progressCallback("Exhaustive: " + std::to_string(evals) + " evaluations for N=" + std::to_string(n));
    }

    auto evalResult = ExhaustiveSolver::solve(parts, sheets, config,
        [&](int done, int total) {
            if (progressCallback && done % 100 == 0) {
                progressCallback("Exhaustive: " + std::to_string(done) + "/" + std::to_string(total));
            }
        });

    auto endTime = std::chrono::high_resolution_clock::now();
    double elapsedMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();

    SolverResult result;
    result.solverUsed = SolverType::Exhaustive;
    result.solverName = solverName(SolverType::Exhaustive);
    result.fitness = evalResult.fitness;
    result.sheetsUsed = evalResult.sheetsUsed;
    result.totalParts = n;
    result.partsPlaced = n; // Exhaustive places all
    result.elapsedMs = elapsedMs;
    result.materialUtilization = 0; // Would need sheet area calculation

    return result;
}

// ============================================================================
// Overlap resolution — Multi-start sequential BLF
//
// Strategy: Run the full multi-sheet BLF placement with many different part
// orderings. Each ordering produces a different sheet assignment because BLF
// places parts sequentially, filling sheets one by one. A single NfpWorker
// instance is reused so NFP computations are cached across all attempts.
//
// With tryAllRotations=true, each part is placed at its optimal rotation.
// The ordering determines WHICH parts compete for space on each sheet.
// ============================================================================

SolverResult SolverOrchestrator::solveOverlapResolution(
    const std::vector<std::shared_ptr<NFP>>& parts,
    const std::vector<std::shared_ptr<NFP>>& sheets,
    const SolverConfig& config,
    std::function<void(const std::string&)> progressCallback)
{
    auto startTime = std::chrono::high_resolution_clock::now();

    if (sheets.empty() || parts.empty()) {
        SolverResult result;
        result.solverUsed = SolverType::OverlapResolution;
        result.solverName = solverName(SolverType::OverlapResolution);
        result.fitness = 1e9;
        result.totalParts = static_cast<int>(parts.size());
        return result;
    }

    int maxSheets = static_cast<int>(sheets.size());
    int n = static_cast<int>(parts.size());

    // Compute sheet area and total parts area
    PolygonBounds sheetBounds = GeometryUtil::getPolygonBounds(*sheets[0]);
    double sheetArea = sheetBounds.width * sheetBounds.height;
    double totalPartsArea = 0;
    std::vector<double> areas(n);
    for (int i = 0; i < n; i++) {
        areas[i] = std::fabs(GeometryUtil::polygonArea(*parts[i]));
        totalPartsArea += areas[i];
    }

    // Assign unique IDs to parts (placeParts uses Id for PlacementItem mapping)
    for (int i = 0; i < n; i++) parts[i]->Id = i;

    // Build source → original index map for result extraction
    std::unordered_map<int, int> sourceToOrigIdx;
    for (int i = 0; i < n; i++) {
        sourceToOrigIdx[parts[i]->source.value_or(-1)] = i;
    }

    // Single NfpWorker instance — NFP cache shared across ALL attempts
    NfpWorker bg;
    bg.EnableCaches = true;
    bg.UseParallel = false;

    std::mt19937 rng(42);

    // Precompute area-sorted order (largest first = FFD heuristic)
    std::vector<int> indices(n);
    std::iota(indices.begin(), indices.end(), 0);
    std::vector<int> areaDescOrder = indices;
    std::sort(areaDescOrder.begin(), areaDescOrder.end(),
              [&](int a, int b) { return areas[a] > areas[b]; });

    // Precompute bounding boxes for shape similarity grouping
    struct ShapeKey {
        double area, width, height;
        int partIdx;
    };
    std::vector<ShapeKey> shapeKeys(n);
    for (int i = 0; i < n; i++) {
        PolygonBounds pb = GeometryUtil::getPolygonBounds(*parts[i]);
        shapeKeys[i] = {areas[i], pb.width, pb.height, i};
    }

    // Group similar shapes together (same area+bbox = identical copies)
    // This enables interlocking: when two identical concave shapes are placed
    // consecutively, the second at 180° can nest inside the first's concavity
    auto groupedByShape = indices;
    std::sort(groupedByShape.begin(), groupedByShape.end(),
              [&](int a, int b) {
                  // Primary: group by area (within 1% tolerance)
                  double areaDiff = std::fabs(areas[a] - areas[b]) / std::max(areas[a], areas[b]);
                  if (areaDiff > 0.01) return areas[a] > areas[b]; // larger first
                  // Secondary: group by bounding box dimensions
                  double wa = shapeKeys[a].width, wb = shapeKeys[b].width;
                  double ha = shapeKeys[a].height, hb = shapeKeys[b].height;
                  double bboxDiff = std::fabs(wa - wb) + std::fabs(ha - hb);
                  if (bboxDiff > 10.0) return wa > wb;
                  return a < b; // stable order for identical shapes
              });

    // Track best result
    int bestSheetsUsed = maxSheets + 1;
    int bestPartsPlaced = 0;
    std::vector<SolverResult::PartResult> bestPlacements;
    std::vector<int> bestOrder;
    std::vector<float> bestRotations;

    // Allowed rotations: {0, 90, 180, 270} based on config.rotations
    int numRotSteps = static_cast<int>(360.0f / config.baseConfig.rotations);
    std::vector<float> allowedRotations;
    for (int r = 0; r < numRotSteps; r++) {
        allowedRotations.push_back(r * (360.0f / config.baseConfig.rotations));
    }

    if (progressCallback) {
        progressCallback("Multi-start BLF: " + std::to_string(n) + " parts, " +
                         std::to_string(maxSheets) + " sheets, budget " +
                         std::to_string(static_cast<int>(config.timeBudgetSeconds)) + "s");
    }

    // Helper: run one BLF attempt with given ordering + rotations, extract results
    // targetSheets: if > 0, use only that many sheets (forces tighter packing)
    auto runBLFAttempt = [&](const std::vector<int>& order,
                              const std::vector<float>& rotations,
                              const NestConfig& nestCfg,
                              int targetSheets = -1)
        -> std::tuple<int, int, std::vector<SolverResult::PartResult>>
    {
        // Set per-part rotations
        for (int i = 0; i < n; i++) {
            parts[order[i]]->Rotation = rotations[i];
        }

        // Build ordered parts list
        std::vector<std::shared_ptr<NFP>> orderedParts;
        orderedParts.reserve(n);
        for (int idx : order) {
            orderedParts.push_back(parts[idx]);
        }

        // Fresh copy of sheets (placeParts consumes them from the vector)
        int numSheets = (targetSheets > 0) ? targetSheets : maxSheets;
        std::vector<std::shared_ptr<NFP>> sheetsCopy;
        sheetsCopy.reserve(numSheets);
        for (int si = 0; si < numSheets && si < static_cast<int>(sheets.size()); si++) {
            sheetsCopy.push_back(std::make_shared<NFP>(*sheets[si]));
        }

        // Run BLF placement
        auto placement = bg.placeParts(sheetsCopy, orderedParts, nestCfg, 0);

        // Extract results
        int sheetsUsed = 0;
        int totalPlaced = 0;
        std::vector<SolverResult::PartResult> placements;
        PolygonBounds sheetBnds = GeometryUtil::getPolygonBounds(*sheets[0]);

        if (!placement.placements.empty()) {
            auto& allSheetItems = placement.placements[0];
            sheetsUsed = static_cast<int>(allSheetItems.size());
            for (int si = 0; si < static_cast<int>(allSheetItems.size()); si++) {
                for (auto& sp : allSheetItems[si].sheetplacements) {
                    SolverResult::PartResult pr;
                    auto it = sourceToOrigIdx.find(sp.source);
                    pr.partIndex = (it != sourceToOrigIdx.end()) ? it->second : sp.id;
                    pr.source = sp.source;
                    pr.x = sp.x;
                    pr.y = sp.y;
                    pr.rotation = sp.rotation;
                    pr.sheetIndex = si;

                    // Validate placement is inside sheet bounds
                    auto& origPoly = parts[pr.partIndex];
                    NFP rotPoly = NfpWorker::rotatePolygon(*origPoly, pr.rotation);
                    NFP shifted = Compaction::getShiftedPolygon(rotPoly, pr.x, pr.y, 0);
                    PolygonBounds pb = GeometryUtil::getPolygonBounds(shifted);
                    bool inBounds = (pb.x >= sheetBnds.x - 2.0 &&
                        pb.x + pb.width <= sheetBnds.x + sheetBnds.width + 2.0 &&
                        pb.y >= sheetBnds.y - 2.0 &&
                        pb.y + pb.height <= sheetBnds.y + sheetBnds.height + 2.0);

                    pr.placed = inBounds;
                    placements.push_back(pr);
                    if (inBounds) totalPlaced++;
                }
            }
        }

        // Recount sheets based on actually valid placements
        int validSheetsUsed = 0;
        for (auto& pr : placements) {
            if (pr.placed) validSheetsUsed = std::max(validSheetsUsed, pr.sheetIndex + 1);
        }

        return {validSheetsUsed, totalPlaced, placements};
    };

    // Helper: check improvement and update best
    auto checkImprovement = [&](int sheetsUsed, int totalPlaced,
                                 std::vector<SolverResult::PartResult>& placements,
                                 const std::vector<int>& order,
                                 const std::vector<float>& rotations,
                                 int attempt) -> bool
    {
        bool improved = false;
        if (totalPlaced > bestPartsPlaced) {
            improved = true;
        } else if (totalPlaced == bestPartsPlaced && sheetsUsed < bestSheetsUsed) {
            improved = true;
        }

        if (improved) {
            bestSheetsUsed = sheetsUsed;
            bestPartsPlaced = totalPlaced;
            bestPlacements = placements;
            bestOrder = order;
            bestRotations = rotations;
            if (progressCallback) {
                progressCallback("  attempt " + std::to_string(attempt) +
                               ": " + std::to_string(totalPlaced) + "/" + std::to_string(n) +
                               " placed on " + std::to_string(sheetsUsed) + " sheets");
            }
        }
        return improved;
    };

    // =====================================================================
    // Identify shape groups: parts with identical bbox dimensions are the
    // same shape and can interlock at 0°+180°. Group by (width, height)
    // within 5% tolerance.
    // =====================================================================
    struct ShapeGroup {
        std::vector<int> members;
        double avgWidth, avgHeight;
    };
    std::vector<ShapeGroup> shapeGroups;

    for (int i = 0; i < n; i++) {
        double w = shapeKeys[i].width, h = shapeKeys[i].height;
        bool found = false;
        for (auto& g : shapeGroups) {
            if (std::fabs(w - g.avgWidth) / std::max(w, g.avgWidth) < 0.05 &&
                std::fabs(h - g.avgHeight) / std::max(h, g.avgHeight) < 0.05) {
                g.members.push_back(i);
                g.avgWidth = (g.avgWidth * (g.members.size() - 1) + w) / g.members.size();
                g.avgHeight = (g.avgHeight * (g.members.size() - 1) + h) / g.members.size();
                found = true;
                break;
            }
        }
        if (!found) {
            shapeGroups.push_back({{i}, w, h});
        }
    }

    // Sort groups by average area DESCENDING (large parts first = arches before panels)
    std::sort(shapeGroups.begin(), shapeGroups.end(),
              [&](const ShapeGroup& a, const ShapeGroup& b) {
                  double areaA = 0, areaB = 0;
                  for (int idx : a.members) areaA += areas[idx];
                  for (int idx : b.members) areaB += areas[idx];
                  areaA /= a.members.size();
                  areaB /= b.members.size();
                  return areaA > areaB;
              });

    if (progressCallback) {
        std::string groupInfo;
        for (auto& g : shapeGroups) {
            if (!groupInfo.empty()) groupInfo += ", ";
            groupInfo += std::to_string(g.members.size()) + "x(" +
                        std::to_string(static_cast<int>(g.avgWidth)) + "x" +
                        std::to_string(static_cast<int>(g.avgHeight)) + ")";
        }
        progressCallback("  Shape groups: " + groupInfo);
    }

    // =====================================================================
    // Generate targeted orderings for concave interlocking
    // Key insight: identical shapes grouped together let BLF discover
    // interlocking via tryAllRotations (chain: →←→ arrangement)
    // =====================================================================
    std::vector<std::vector<int>> targetOrderings;

    // Ordering 1: Each shape group together, largest groups first
    {
        std::vector<int> order;
        for (auto& g : shapeGroups) {
            for (int idx : g.members) order.push_back(idx);
        }
        targetOrderings.push_back(order);
    }

    // Ordering 2: Same but smallest groups first (different sheet assignment)
    {
        std::vector<int> order;
        for (int gi = static_cast<int>(shapeGroups.size()) - 1; gi >= 0; gi--) {
            for (int idx : shapeGroups[gi].members) order.push_back(idx);
        }
        targetOrderings.push_back(order);
    }

    // Ordering 3: Groups interleaved with panels
    // Arches from each group, then some panels, then next group
    {
        std::vector<int> arches, panels;
        for (auto& g : shapeGroups) {
            if (g.avgWidth > 500 || g.avgHeight > 500) { // arch threshold
                for (int idx : g.members) arches.push_back(idx);
            } else {
                for (int idx : g.members) panels.push_back(idx);
            }
        }
        // Interleave: put panels after each arch-group batch
        std::vector<int> order;
        int panelIdx = 0;
        for (auto& g : shapeGroups) {
            if (g.avgWidth > 500 || g.avgHeight > 500) {
                for (int idx : g.members) order.push_back(idx);
                // Insert 2-3 panels after each arch group
                for (int p = 0; p < 3 && panelIdx < static_cast<int>(panels.size()); p++, panelIdx++) {
                    order.push_back(panels[panelIdx]);
                }
            }
        }
        // Add remaining panels
        while (panelIdx < static_cast<int>(panels.size())) {
            order.push_back(panels[panelIdx++]);
        }
        if (static_cast<int>(order.size()) == n) {
            targetOrderings.push_back(order);
        }
    }

    // Ordering 4: FFD (area descending)
    targetOrderings.push_back(areaDescOrder);

    // Ordering 5: Original shape-grouped ordering
    targetOrderings.push_back(groupedByShape);

    // Cross-type orderings: lead with 2 arches from one group + 1 from another.
    // Key insight: different arch shapes can interlock (e.g., 2 D-arches + 1 A-arch).
    {
        std::vector<int> lgIdx, smIdx; // large (arch) vs small (panel) group indices
        for (int gi = 0; gi < static_cast<int>(shapeGroups.size()); gi++) {
            double avgA = 0;
            for (int idx : shapeGroups[gi].members) avgA += areas[idx];
            avgA /= shapeGroups[gi].members.size();
            (avgA > 300000 ? lgIdx : smIdx).push_back(gi);
        }
        for (int gA : lgIdx) {
            if (shapeGroups[gA].members.size() < 2) continue;
            for (int gB : lgIdx) {
                if (gA == gB) continue;
                std::vector<int> order;
                // Lead with cross-type trio
                order.push_back(shapeGroups[gA].members[0]);
                order.push_back(shapeGroups[gA].members[1]);
                order.push_back(shapeGroups[gB].members[0]);
                // Add remaining arches from all groups
                for (int gi : lgIdx) {
                    int skip = (gi == gA) ? 2 : (gi == gB) ? 1 : 0;
                    for (int k = skip; k < static_cast<int>(shapeGroups[gi].members.size()); k++)
                        order.push_back(shapeGroups[gi].members[k]);
                }
                // Add panels
                for (int gi : smIdx)
                    for (int idx : shapeGroups[gi].members) order.push_back(idx);

                if (static_cast<int>(order.size()) == n)
                    targetOrderings.push_back(order);
            }
        }
    }

    // Orderings 6+: Shuffle within each shape group (different interlocking sequences)
    for (int perm = 0; perm < 3; perm++) {
        std::vector<int> order;
        for (auto g : shapeGroups) { // copy intentional
            std::shuffle(g.members.begin(), g.members.end(), rng);
            for (int idx : g.members) order.push_back(idx);
        }
        targetOrderings.push_back(order);
    }

    // =====================================================================
    // Phase 1: Fast exploration (no tryAllRotations)
    // Quick survey of orderings to find promising ones
    // =====================================================================
    NestConfig fastConfig = config.baseConfig;
    fastConfig.tryAllRotations = false;
    fastConfig.edgeSamples = 1;
    fastConfig.compactionPasses = 0;
    fastConfig.clipByRects = true;
    fastConfig.exploreConcave = true;

    std::uniform_int_distribution<int> rotDist(0, static_cast<int>(allowedRotations.size()) - 1);

    int attempt = 0;
    double phase1Budget = config.timeBudgetSeconds * 0.20;
    int phase1MaxAttempts = 2;  // 1-2 attempts: warm NFP cache + find baseline

    while (true) {
        auto now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(now - startTime).count();
        if (elapsed > phase1Budget || attempt >= phase1MaxAttempts) break;

        std::vector<int> order;
        std::vector<float> rotations(n, 0.0f);

        if (attempt < static_cast<int>(targetOrderings.size())) {
            order = targetOrderings[attempt];
            // Alternating 0°/180° within each shape group for interlocking
            for (int i = 0; i < n; i++) {
                rotations[i] = (i % 2 == 0) ? 0.0f : 180.0f;
            }
        } else {
            // Random permutation + random rotations
            order = indices;
            std::shuffle(order.begin(), order.end(), rng);
            for (int i = 0; i < n; i++) {
                rotations[i] = allowedRotations[rotDist(rng)];
            }
        }

        auto [sheetsUsed, totalPlaced, placements] = runBLFAttempt(order, rotations, fastConfig);
        checkImprovement(sheetsUsed, totalPlaced, placements, order, rotations, attempt + 1);
        attempt++;
    }

    if (progressCallback) {
        progressCallback("Phase 1 done: " + std::to_string(attempt) + " attempts, best " +
                        std::to_string(bestSheetsUsed) + " sheets");
    }

    // =====================================================================
    // Phase 2: Target (bestSheetsUsed - 1) sheets with tryAllRotations
    // Force BLF to use fewer sheets → harder interlocking search.
    // Try each ordering with reduced sheet count.
    // =====================================================================
    int targetSheetCount = std::max(1, bestSheetsUsed - 1);
    // Theoretical minimum: ceil(totalPartsArea / sheetArea)
    int theoreticalMin = static_cast<int>(std::ceil(totalPartsArea / sheetArea));
    targetSheetCount = std::max(targetSheetCount, theoreticalMin);

    NestConfig tryAllConfig = config.baseConfig;
    tryAllConfig.tryAllRotations = true;
    tryAllConfig.edgeSamples = 3;
    tryAllConfig.compactionPasses = 4;
    tryAllConfig.clipByRects = true;
    tryAllConfig.exploreConcave = true;
    // Keep 4 rotations for speed (8 is too slow for multiple attempts)

    if (progressCallback) {
        progressCallback("Phase 2: targeting " + std::to_string(targetSheetCount) +
                        " sheets (theoretical min=" + std::to_string(theoreticalMin) +
                        "), tryAllRotations=" + std::to_string(tryAllConfig.rotations) + " angles");
    }

    int phase2Attempt = 0;
    double phase2Budget = config.timeBudgetSeconds * 0.50;
    // Run multiple attempts with different orderings to explore interlocking opportunities
    int phase2MaxAttempts = 3;

    while (true) {
        auto now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(now - startTime).count();
        if (elapsed > phase2Budget || phase2Attempt >= phase2MaxAttempts) break;

        std::vector<int> order;
        std::vector<float> rotations(n, 0.0f); // tryAllRotations handles rotations

        if (phase2Attempt < static_cast<int>(targetOrderings.size())) {
            order = targetOrderings[phase2Attempt];
        } else if (!bestOrder.empty()) {
            // Perturbation of best: swap 2-4 random pairs
            order = bestOrder;
            int swaps = 2 + (rng() % 3);
            for (int s = 0; s < swaps && n > 1; s++) {
                int i = rng() % (n - 1);
                std::swap(order[i], order[i + 1]);
            }
        } else {
            order = indices;
            std::shuffle(order.begin(), order.end(), rng);
        }

        // Try with reduced sheet count (forces tighter interlocking)
        auto [sheetsUsed, totalPlaced, placements] =
            runBLFAttempt(order, rotations, tryAllConfig, targetSheetCount);

        bool foundImprovement = false;
        if (totalPlaced == n && sheetsUsed <= targetSheetCount) {
            foundImprovement = checkImprovement(sheetsUsed, totalPlaced, placements, order, rotations,
                           attempt + phase2Attempt + 1);

            if (progressCallback) {
                progressCallback("  Found " + std::to_string(sheetsUsed) +
                               "-sheet solution at attempt " + std::to_string(phase2Attempt + 1));
            }
            // Try even fewer sheets
            targetSheetCount = std::max(theoreticalMin, sheetsUsed - 1);
        }

        // Only try with all sheets if reduced didn't work (saves ~15s per attempt)
        if (!foundImprovement) {
            auto [su2, tp2, pl2] = runBLFAttempt(order, rotations, tryAllConfig);
            checkImprovement(su2, tp2, pl2, order, rotations, attempt + phase2Attempt + 1);
        }

        phase2Attempt++;
    }

    if (progressCallback) {
        progressCallback("Phase 2 done: " + std::to_string(phase2Attempt) +
                        " attempts, best " + std::to_string(bestSheetsUsed) + " sheets");
    }

    attempt += phase2Attempt;

    // =====================================================================
    // Phase 3: Per-sheet assignment search
    // Instead of letting BLF decide which parts go on which sheet,
    // explicitly assign subsets of parts to individual sheets and check
    // if each subset fits. This lets us discover configurations like
    // 3 identical arches chain-interlocked on a single sheet.
    // =====================================================================
    if (bestSheetsUsed > theoreticalMin) {
        if (progressCallback) {
            progressCallback("Phase 3: per-sheet assignment search...");
        }

        // Separate arches from panels
        std::vector<int> archIndices, panelIndices;
        for (auto& g : shapeGroups) {
            double avgArea = 0;
            for (int idx : g.members) avgArea += areas[idx];
            avgArea /= g.members.size();
            if (avgArea > 300000) { // arch threshold
                for (int idx : g.members) archIndices.push_back(idx);
            } else {
                for (int idx : g.members) panelIndices.push_back(idx);
            }
        }

        // Helper: run BLF on a subset of parts on numSheets sheets
        auto runSubsetBLF = [&](const std::vector<int>& partIndices,
                                 const NestConfig& cfg,
                                 int numSheets = 1)
            -> std::pair<int, std::vector<SolverResult::PartResult>>
        {
            // Assign IDs
            for (int i = 0; i < static_cast<int>(partIndices.size()); i++) {
                parts[partIndices[i]]->Rotation = 0;
            }

            std::vector<std::shared_ptr<NFP>> subParts;
            for (int idx : partIndices) subParts.push_back(parts[idx]);

            std::vector<std::shared_ptr<NFP>> sheetsCopy;
            for (int si = 0; si < numSheets && si < static_cast<int>(sheets.size()); si++) {
                sheetsCopy.push_back(std::make_shared<NFP>(*sheets[si]));
            }

            auto placement = bg.placeParts(sheetsCopy, subParts, cfg, 0);

            int totalPlaced = 0;
            std::vector<SolverResult::PartResult> results;
            PolygonBounds sheetBounds = GeometryUtil::getPolygonBounds(*sheets[0]);
            if (!placement.placements.empty()) {
                auto& allSheetItems = placement.placements[0];
                for (int si = 0; si < static_cast<int>(allSheetItems.size()); si++) {
                    for (auto& sp : allSheetItems[si].sheetplacements) {
                        SolverResult::PartResult pr;
                        auto it = sourceToOrigIdx.find(sp.source);
                        pr.partIndex = (it != sourceToOrigIdx.end()) ? it->second : sp.id;
                        pr.source = sp.source;
                        pr.x = sp.x;
                        pr.y = sp.y;
                        pr.rotation = sp.rotation;
                        pr.sheetIndex = si;

                        // Validate: check placed polygon is inside sheet bounds
                        auto& origPoly = parts[pr.partIndex];
                        NFP rotated = NfpWorker::rotatePolygon(*origPoly, pr.rotation);
                        PolygonBounds pb = GeometryUtil::getPolygonBounds(
                            Compaction::getShiftedPolygon(rotated, pr.x, pr.y, 0));
                        bool inBounds = (pb.x >= sheetBounds.x - 2.0 &&
                            pb.x + pb.width <= sheetBounds.x + sheetBounds.width + 2.0 &&
                            pb.y >= sheetBounds.y - 2.0 &&
                            pb.y + pb.height <= sheetBounds.y + sheetBounds.height + 2.0);

                        pr.placed = inBounds;
                        results.push_back(pr);
                        if (inBounds) totalPlaced++;
                    }
                }
            }
            return {totalPlaced, results};
        };

        // Generate sheet assignment candidates
        int targetK = std::max(theoreticalMin, bestSheetsUsed - 1);
        int nPanel = static_cast<int>(panelIndices.size());

        // Find groups of identical arches (same shape group with >1 member)
        struct ArchGroup { std::vector<int> indices; double area; };
        std::vector<ArchGroup> archGroups;
        for (auto& g : shapeGroups) {
            double avgArea = 0;
            for (int idx : g.members) avgArea += areas[idx];
            avgArea /= g.members.size();
            if (avgArea > 300000) {
                archGroups.push_back({g.members, avgArea});
            }
        }

        // Step 1b: Cross-type chain analysis
        // Place D1 at origin, then middle-arch tangent to D1 (nestles into concavity),
        // then D2 tangent to middle-arch. Check D2 doesn't overlap D1.
        if (progressCallback) {
            progressCallback("  Chain analysis: " + std::to_string(archGroups.size()) + " arch groups");
        }

        struct CrossChainResult {
            int groupA, groupB;
            float rotA1, rotA2, rotB;
            double dx12, dy12;      // D2 shift
            double dx3, dy3;        // B (middle arch) shift
            double chainWidth, chainHeight;
        };
        std::vector<CrossChainResult> validCrossChains;

        int nArchGroups = static_cast<int>(archGroups.size());

        // Sort group pairs: try tallest pair groups first (deeper concavity)
        struct GroupPair { int gA, gB; };
        std::vector<GroupPair> groupPairs;
        for (int gA = 0; gA < nArchGroups; gA++) {
            if (archGroups[gA].indices.size() < 2) continue;
            for (int gB = 0; gB < nArchGroups; gB++) {
                if (gA == gB) continue;
                groupPairs.push_back({gA, gB});
            }
        }
        // Sort: tallest pair arch first (deepest concavity), then shortest
        // middle arch first (most likely to nestle inside concavity)
        std::sort(groupPairs.begin(), groupPairs.end(),
            [&](const GroupPair& a, const GroupPair& b) {
                double hPairA = parts[archGroups[a.gA].indices[0]]->HeightCalculated();
                double hPairB = parts[archGroups[b.gA].indices[0]]->HeightCalculated();
                if (std::fabs(hPairA - hPairB) > 10) return hPairA > hPairB;
                // Same pair group — sort by middle arch height ascending
                double hMidA = parts[archGroups[a.gB].indices[0]]->HeightCalculated();
                double hMidB = parts[archGroups[b.gB].indices[0]]->HeightCalculated();
                return hMidA < hMidB;
            });

        // Track which pair groups (gA values) have found chains so we can
        // skip other middle arches for the same pair group
        std::set<int> pairGroupsWithChains;

        for (auto& gp : groupPairs) {
            int gA = gp.gA, gB = gp.gB;

            // If this pair group already found chains, skip to next pair group
            // (different middle arches give same remaining-parts problem)
            if (pairGroupsWithChains.count(gA)) continue;

            auto& polyA = *parts[archGroups[gA].indices[0]]; // "pair" arch type
            auto& polyB = *parts[archGroups[gB].indices[0]]; // "middle" arch type

                std::vector<float> d1Rotations = {0, 90, 180, 270};
                std::vector<float> d2Rotations = {0, 90, 180, 270};
                std::vector<float> bRotations = {0, 90, 180, 270};

                bool foundChainThisPair = false;

                for (float rotD1 : d1Rotations) {
                    if (foundChainThisPair) break;

                    // Time check
                    auto nowCheck = std::chrono::high_resolution_clock::now();
                    double elCheck = std::chrono::duration<double>(nowCheck - startTime).count();
                    if (elCheck > config.timeBudgetSeconds * 0.85) break;

                    NFP d1 = NfpWorker::rotatePolygon(polyA, rotD1);
                    PolygonBounds bb_d1 = GeometryUtil::getPolygonBounds(d1);

                    // Build d1 Clipper path once
                    Clipper2Lib::PathD pathD1;
                    for (int j = 0; j < d1.length(); j++)
                        pathD1.push_back({d1[j].x, d1[j].y});

                    for (float rotB : bRotations) {
                        if (foundChainThisPair) break;

                        // Time check at oB level too
                        auto nowB = std::chrono::high_resolution_clock::now();
                        double elB = std::chrono::duration<double>(nowB - startTime).count();
                        if (elB > config.timeBudgetSeconds * 0.85) break;

                        NFP b = NfpWorker::rotatePolygon(polyB, rotB);

                        // NFP(d1, b): positions for b[0] where b tangent to d1
                        auto nfp_d1_b = bg.getOuterNfp(d1, b, 0);
                        if (!nfp_d1_b || nfp_d1_b->length() == 0) continue;

                        // Pre-compute b bbox extents relative to b[0]
                        double bMinRX = 1e9, bMaxRX = -1e9, bMinRY = 1e9, bMaxRY = -1e9;
                        for (int j = 0; j < b.length(); j++) {
                            double rx = b[j].x - b[0].x, ry = b[j].y - b[0].y;
                            bMinRX = std::min(bMinRX, rx); bMaxRX = std::max(bMaxRX, rx);
                            bMinRY = std::min(bMinRY, ry); bMaxRY = std::max(bMaxRY, ry);
                        }

                        for (float rotD2 : d2Rotations) {
                            NFP d2 = NfpWorker::rotatePolygon(polyA, rotD2);

                            // NFP(b, d2): positions for d2[0] where d2 tangent to b
                            auto nfp_b_d2 = bg.getOuterNfp(b, d2, 0);
                            if (!nfp_b_d2 || nfp_b_d2->length() == 0) continue;

                            // Pre-compute d2 bbox extents relative to d2[0]
                            double d2MinRX = 1e9, d2MaxRX = -1e9;
                            double d2MinRY = 1e9, d2MaxRY = -1e9;
                            for (int j = 0; j < d2.length(); j++) {
                                double rx = d2[j].x - d2[0].x, ry = d2[j].y - d2[0].y;
                                d2MinRX = std::min(d2MinRX, rx);
                                d2MaxRX = std::max(d2MaxRX, rx);
                                d2MinRY = std::min(d2MinRY, ry);
                                d2MaxRY = std::max(d2MaxRY, ry);
                            }

                            double bestW = 1e9, bestH = 1e9;
                            double bestDxB = 0, bestDyB = 0;
                            double bestDxD2 = 0, bestDyD2 = 0;
                            bool foundAny = false;
                            int total = 0, bboxFail = 0, overlapFail = 0;

                            // Scan b positions on NFP(d1, b) boundary
                            for (int pi = 0; pi < nfp_d1_b->length(); pi++) {
                                double bRefX = (*nfp_d1_b)[pi].x;
                                double bRefY = (*nfp_d1_b)[pi].y;
                                double dxB = bRefX - b[0].x;
                                double dyB = bRefY - b[0].y;

                                // d1+b combined bbox
                                double pMinX = std::min(bb_d1.x, bRefX + bMinRX);
                                double pMaxX = std::max(bb_d1.x + bb_d1.width, bRefX + bMaxRX);
                                double pMinY = std::min(bb_d1.y, bRefY + bMinRY);
                                double pMaxY = std::max(bb_d1.y + bb_d1.height, bRefY + bMaxRY);

                                // Scan d2 positions on shifted NFP(b, d2)
                                for (int qi = 0; qi < nfp_b_d2->length(); qi++) {
                                    double d2RefX = (*nfp_b_d2)[qi].x + dxB;
                                    double d2RefY = (*nfp_b_d2)[qi].y + dyB;
                                    double dxD2 = d2RefX - d2[0].x;
                                    double dyD2 = d2RefY - d2[0].y;
                                    total++;

                                    // Full bbox
                                    double mnX = std::min(pMinX, d2RefX + d2MinRX);
                                    double mxX = std::max(pMaxX, d2RefX + d2MaxRX);
                                    double mnY = std::min(pMinY, d2RefY + d2MinRY);
                                    double mxY = std::max(pMaxY, d2RefY + d2MaxRY);
                                    double cw = mxX - mnX, ch = mxY - mnY;

                                    bool fH = cw <= sheetBounds.width && ch <= sheetBounds.height;
                                    bool fV = ch <= sheetBounds.width && cw <= sheetBounds.height;
                                    if (!fH && !fV) { bboxFail++; continue; }

                                    // Check D2 doesn't overlap D1
                                    Clipper2Lib::PathD pathD2;
                                    for (int j = 0; j < d2.length(); j++)
                                        pathD2.push_back({d2[j].x + dxD2, d2[j].y + dyD2});
                                    auto inter = Clipper2Lib::Intersect(
                                        {pathD1}, {pathD2}, Clipper2Lib::FillRule::NonZero, 2);
                                    double overlap = 0;
                                    for (auto& p : inter)
                                        overlap += std::fabs(Clipper2Lib::Area(p));
                                    if (overlap > 100.0) { overlapFail++; continue; }

                                    if (cw < bestW || (std::fabs(cw - bestW) < 1 && ch < bestH)) {
                                        bestW = cw; bestH = ch;
                                        bestDxB = dxB; bestDyB = dyB;
                                        bestDxD2 = dxD2; bestDyD2 = dyD2;
                                        foundAny = true;
                                    }
                                }
                            }

                            if (foundAny) {
                                validCrossChains.push_back({
                                    gA, gB, rotD1, rotD2, rotB,
                                    bestDxD2, bestDyD2, bestDxB, bestDyB,
                                    bestW, bestH
                                });
                                foundChainThisPair = true;
                                pairGroupsWithChains.insert(gA);
                            }
                        }
                    }
                } // end d1Rotations
        } // end for groupPairs

        if (progressCallback) {
            progressCallback("  Cross-type chains found: " +
                           std::to_string(validCrossChains.size()));
        }

        // Build solutions from valid cross-chains — verify with BLF, not analytical placement
        for (auto& cc : validCrossChains) {
            auto now = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double>(now - startTime).count();
            if (elapsed > config.timeBudgetSeconds * 0.95) break;

            int idx1 = archGroups[cc.groupA].indices[0];
            int idx2 = archGroups[cc.groupA].indices[1];
            int idx3 = archGroups[cc.groupB].indices[0];

            // Verify the 3 arches actually fit on 1 sheet using BLF
            // (chain analysis finds tangent positions but may allow small overlaps)
            std::vector<int> chainParts = {idx1, idx2, idx3};
            auto [chainPlaced, chainResults] = runSubsetBLF(chainParts, tryAllConfig, 1);

            if (chainPlaced != 3) {
                if (progressCallback) {
                    progressCallback("    Chain G" + std::to_string(cc.groupA) + "+G" +
                        std::to_string(cc.groupB) + ": BLF verification failed (" +
                        std::to_string(chainPlaced) + "/3 fit)");
                }
                continue;
            }

            if (progressCallback) {
                progressCallback("    Chain G" + std::to_string(cc.groupA) + "+G" +
                    std::to_string(cc.groupB) + ": 3 arches fit on 1 sheet (BLF verified)");
            }

            // Remaining parts
            std::vector<int> remaining;
            for (int i = 0; i < n; i++) {
                if (i != idx1 && i != idx2 && i != idx3) remaining.push_back(i);
            }
            if (remaining.empty()) continue;

            std::sort(remaining.begin(), remaining.end(),
                [&](int a, int b) { return areas[a] > areas[b]; });

            int maxRemSheets = std::max(1, bestSheetsUsed - 1);
            int minRemSheets = std::max(1, targetK - 1);

            for (int remSheets = minRemSheets; remSheets <= maxRemSheets; remSheets++) {
                auto [remPlaced, remResults] = runSubsetBLF(remaining, tryAllConfig, remSheets);

                if (progressCallback) {
                    progressCallback("    Chain remaining: " + std::to_string(remPlaced) +
                        "/" + std::to_string(remaining.size()) + " on " +
                        std::to_string(remSheets) + " sheets");
                }

                if (remPlaced == static_cast<int>(remaining.size())) {
                    // Combine: chain results on sheet 0, remaining on sheets 1+
                    std::vector<SolverResult::PartResult> combined;
                    for (auto& pr : chainResults) {
                        pr.sheetIndex = 0;
                        combined.push_back(pr);
                    }
                    int maxRemSheet = 0;
                    for (auto& pr : remResults) {
                        maxRemSheet = std::max(maxRemSheet, pr.sheetIndex);
                        pr.sheetIndex += 1;
                        combined.push_back(pr);
                    }
                    int totalSheets = maxRemSheet + 2;

                    if (static_cast<int>(combined.size()) == n &&
                        totalSheets <= bestSheetsUsed) {
                        bestSheetsUsed = totalSheets;
                        bestPartsPlaced = n;
                        bestPlacements = combined;
                        if (progressCallback) {
                            progressCallback("  Cross-chain solution: " +
                                std::to_string(totalSheets) + " sheets (3 arches on sheet 0, G" +
                                std::to_string(cc.groupA) + "+G" + std::to_string(cc.groupB) + ")");
                        }
                    }
                    break;
                }
            }
        }

        // Step 2: BLF-based per-sheet feasibility testing (fallback)
        // Also try BLF on arch subsets in case analytical approach missed something.

        struct ArchSubset {
            std::vector<int> indices;
            std::string label;
        };
        std::vector<ArchSubset> archSubsets;

        // (a) Cross-type: 2 from group A + 1 from group B
        for (int gA = 0; gA < nArchGroups; gA++) {
            if (archGroups[gA].indices.size() < 2) continue;
            for (int gB = 0; gB < nArchGroups; gB++) {
                if (gA == gB) continue;
                archSubsets.push_back({
                    {archGroups[gA].indices[0], archGroups[gA].indices[1],
                     archGroups[gB].indices[0]},
                    "2xG" + std::to_string(gA) + "+1xG" + std::to_string(gB)
                });
            }
        }

        // (b) Triple cross-type: 1 from each of 3 different groups
        for (int gA = 0; gA < nArchGroups; gA++)
            for (int gB = gA + 1; gB < nArchGroups; gB++)
                for (int gC = gB + 1; gC < nArchGroups; gC++) {
                    archSubsets.push_back({
                        {archGroups[gA].indices[0], archGroups[gB].indices[0],
                         archGroups[gC].indices[0]},
                        "G" + std::to_string(gA) + "+G" + std::to_string(gB) + "+G" + std::to_string(gC)
                    });
                }

        // (c) Same-type 3 (in case BLF finds what chain analysis missed)
        for (int gi = 0; gi < nArchGroups; gi++) {
            if (archGroups[gi].indices.size() >= 3) {
                archSubsets.push_back({
                    {archGroups[gi].indices[0], archGroups[gi].indices[1],
                     archGroups[gi].indices[2]},
                    "3xG" + std::to_string(gi)
                });
            }
        }

        if (progressCallback) {
            progressCallback("  Testing " + std::to_string(archSubsets.size()) +
                           " arch subsets for single-sheet packing...");
        }

        struct FeasiblePacking {
            std::vector<int> permutation;  // part indices in the order that worked
            int maxPanels;                 // how many additional panels fit
        };
        std::vector<FeasiblePacking> feasiblePackings;

        for (auto& subset : archSubsets) {
            auto now = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double>(now - startTime).count();
            if (elapsed > config.timeBudgetSeconds * 0.90) break;

            // Try all permutations of the 3-arch subset
            auto perm = subset.indices;
            std::sort(perm.begin(), perm.end());
            bool found = false;

            do {
                auto [placed, results] = runSubsetBLF(perm, tryAllConfig, 1);
                if (placed == static_cast<int>(perm.size())) {
                    found = true;
                    break;
                }
            } while (std::next_permutation(perm.begin(), perm.end()));

            if (!found) {
                if (progressCallback) {
                    progressCallback("    [" + subset.label + "] does not fit on 1 sheet");
                }
                continue;
            }

            // Found feasible! Test how many additional panels also fit
            auto archPerm = perm; // the permutation that worked
            int maxP = 0;
            for (int np = 1; np <= nPanel; np++) {
                auto testWP = archPerm;
                for (int p = 0; p < np; p++) testWP.push_back(panelIndices[p]);
                auto [pp, rp] = runSubsetBLF(testWP, tryAllConfig, 1);
                if (pp == static_cast<int>(testWP.size())) maxP = np;
                else break;
            }

            feasiblePackings.push_back({archPerm, maxP});

            if (progressCallback) {
                std::string partList;
                for (int idx : archPerm) {
                    if (!partList.empty()) partList += ",";
                    partList += "P" + std::to_string(parts[idx]->source.value_or(idx));
                }
                progressCallback("    [" + subset.label + "] fits: order [" + partList +
                               "] + " + std::to_string(maxP) + " panels");
            }
        }

        // Step 3: For each feasible packing, test if remaining parts fit on targetK-1 sheets
        if (progressCallback) {
            progressCallback("  " + std::to_string(feasiblePackings.size()) +
                           " feasible packings. Testing remaining parts on " +
                           std::to_string(targetK - 1) + " sheets...");
        }

        int phase3Attempt = 0;
        for (auto& fp : feasiblePackings) {
            // Try different panel counts on sheet 0 (more panels = less load on remaining)
            std::vector<int> panelCounts;
            panelCounts.push_back(fp.maxPanels);
            if (fp.maxPanels > 0) panelCounts.push_back(0);
            if (fp.maxPanels > 2) panelCounts.push_back(fp.maxPanels / 2);

            for (int np : panelCounts) {
                auto now = std::chrono::high_resolution_clock::now();
                double elapsed = std::chrono::duration<double>(now - startTime).count();
                if (elapsed > config.timeBudgetSeconds * 0.95) break;

                // Build set of parts on sheet 0
                std::vector<bool> onSheet0(n, false);
                for (int idx : fp.permutation) onSheet0[idx] = true;
                for (int p = 0; p < np; p++) onSheet0[panelIndices[p]] = true;

                // Build remaining parts list
                std::vector<int> remaining;
                for (int i = 0; i < n; i++) {
                    if (!onSheet0[i]) remaining.push_back(i);
                }

                if (remaining.empty()) continue;

                int remSheets = targetK - 1;
                if (remSheets <= 0) continue;

                auto [remPlaced, remResults] = runSubsetBLF(remaining, tryAllConfig, remSheets);

                if (remPlaced == static_cast<int>(remaining.size())) {
                    // All remaining parts fit! Re-run sheet 0 to get placement results
                    auto sheet0Parts = fp.permutation;
                    for (int p = 0; p < np; p++) sheet0Parts.push_back(panelIndices[p]);
                    auto [s0Placed, s0Results] = runSubsetBLF(sheet0Parts, tryAllConfig, 1);

                    if (s0Placed != static_cast<int>(sheet0Parts.size())) continue;

                    // Combine results: sheet 0 from dense packing, rest shifted by +1
                    std::vector<SolverResult::PartResult> combined;
                    for (auto& pr : s0Results) {
                        pr.sheetIndex = 0;
                        combined.push_back(pr);
                    }

                    int maxRemSheet = 0;
                    for (auto& pr : remResults) {
                        maxRemSheet = std::max(maxRemSheet, pr.sheetIndex);
                        pr.sheetIndex += 1;
                        combined.push_back(pr);
                    }
                    int totalSheets = maxRemSheet + 2; // sheet 0 + remaining sheets

                    if (static_cast<int>(combined.size()) == n) {
                        if (totalSheets < bestSheetsUsed ||
                            (totalSheets == bestSheetsUsed && n > bestPartsPlaced)) {
                            bestSheetsUsed = totalSheets;
                            bestPartsPlaced = n;
                            bestPlacements = combined;

                            if (progressCallback) {
                                progressCallback("  Found " + std::to_string(totalSheets) +
                                               "-sheet solution! (" + std::to_string(np) +
                                               " panels on dense sheet)");
                            }
                        }
                    }
                }
                phase3Attempt++;
            }
        }

        attempt += phase3Attempt;

        if (progressCallback) {
            progressCallback("Phase 3 done: " + std::to_string(phase3Attempt) +
                           " tests, best " + std::to_string(bestSheetsUsed) + " sheets");
        }

    }

    // Post-placement compaction + jostle on each sheet
    if (bestSheetsUsed > 0 && bestSheetsUsed <= maxSheets) {
        if (progressCallback) progressCallback("Post-placement compaction + jostle...");

        // Group best placements by sheet
        std::vector<std::vector<int>> sheetPartIndices(bestSheetsUsed);
        for (int pi = 0; pi < (int)bestPlacements.size(); pi++) {
            if (bestPlacements[pi].placed && bestPlacements[pi].sheetIndex < bestSheetsUsed)
                sheetPartIndices[bestPlacements[pi].sheetIndex].push_back(pi);
        }

        for (int si = 0; si < bestSheetsUsed; si++) {
            if ((int)sheetPartIndices[si].size() < 2) continue;

            // Build placed polygon list for this sheet
            std::vector<std::shared_ptr<NFP>> placedPolys;
            std::vector<PlacementItem> placements;
            for (int pi : sheetPartIndices[si]) {
                auto& pr = bestPlacements[pi];
                auto rotated = std::make_shared<NFP>(
                    NfpWorker::rotatePolygon(*parts[pr.partIndex], pr.rotation));
                PlacementItem item;
                item.id = pr.partIndex;
                item.source = pr.source;
                item.x = pr.x;
                item.y = pr.y;
                item.rotation = 0;  // rotation already applied
                placedPolys.push_back(rotated);
                placements.push_back(item);
            }

            // Save pre-compaction positions for rollback
            std::vector<PlacementItem> preCompactPlacements = placements;

            // Compaction (3-directional slide)
            SolverConfig compactConfig = config;
            compactConfig.compactionRounds = 10;
            Compaction::compact(placedPolys, placements, *sheets[0], compactConfig);

            // Jostle (4-directional breathing)
            SolverConfig jostleConfig = config;
            jostleConfig.jostleIterations = 20;
            JostleHeuristic::jostle(placedPolys, placements, *sheets[0], jostleConfig);

            // Validate: revert any parts pushed outside sheet bounds
            PolygonBounds sheetBounds = GeometryUtil::getPolygonBounds(*sheets[0]);
            for (int j = 0; j < (int)placements.size(); j++) {
                NFP shifted = Compaction::getShiftedPolygon(
                    *placedPolys[j], placements[j].x, placements[j].y, 0);
                PolygonBounds pb = GeometryUtil::getPolygonBounds(shifted);
                if (pb.x < sheetBounds.x - 1.0 ||
                    pb.x + pb.width > sheetBounds.x + sheetBounds.width + 1.0 ||
                    pb.y < sheetBounds.y - 1.0 ||
                    pb.y + pb.height > sheetBounds.y + sheetBounds.height + 1.0) {
                    // Part was pushed outside — revert to pre-compaction position
                    placements[j] = preCompactPlacements[j];
                }
            }

            // Write back updated positions
            for (int j = 0; j < (int)sheetPartIndices[si].size(); j++) {
                bestPlacements[sheetPartIndices[si][j]].x = placements[j].x;
                bestPlacements[sheetPartIndices[si][j]].y = placements[j].y;
            }
        }

        if (progressCallback) progressCallback("Post-placement compaction + jostle done.");
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    double elapsedMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();

    // Build result
    SolverResult result;
    result.solverUsed = SolverType::OverlapResolution;
    result.solverName = solverName(SolverType::OverlapResolution);
    result.totalParts = n;
    result.partsPlaced = bestPartsPlaced;
    result.sheetsUsed = bestSheetsUsed;
    result.fitness = (bestSheetsUsed <= maxSheets) ? bestSheetsUsed * sheetArea : 1e9;
    result.materialUtilization = (bestSheetsUsed > 0) ? totalPartsArea / (sheetArea * bestSheetsUsed) : 0;
    result.parts = bestPlacements;
    result.elapsedMs = elapsedMs;

    if (progressCallback) {
        progressCallback("Done: " + std::to_string(attempt) + " attempts in " +
                        std::to_string(static_cast<int>(elapsedMs)) + "ms, best " +
                        std::to_string(bestSheetsUsed) + " sheets (" +
                        std::to_string(bestPartsPlaced) + "/" + std::to_string(n) + " placed)");

        // Diagnostic: show per-sheet placement details
        std::vector<std::vector<const SolverResult::PartResult*>> sheetDetail(bestSheetsUsed);
        for (auto& pr : bestPlacements) {
            if (pr.placed && pr.sheetIndex < bestSheetsUsed) {
                sheetDetail[pr.sheetIndex].push_back(&pr);
            }
        }
        for (int si = 0; si < bestSheetsUsed; si++) {
            double sheetPartsArea = 0;
            std::string partList;
            for (auto* pr : sheetDetail[si]) {
                sheetPartsArea += areas[pr->partIndex];
                if (!partList.empty()) partList += ", ";
                partList += "P" + std::to_string(pr->source) + "@" +
                           std::to_string(static_cast<int>(pr->rotation)) + "°";
            }
            progressCallback("  Sheet " + std::to_string(si) + ": " +
                           std::to_string(sheetDetail[si].size()) + " parts (" + partList + ") " +
                           std::to_string(static_cast<int>(sheetPartsArea / sheetArea * 100)) + "% util");
        }
    }

    return result;
}

// ============================================================================
// Smart BLF: PairMatching + tryAllRotations BLF + Compaction + Jostle
// ============================================================================

SolverResult SolverOrchestrator::solveSmartBLF(
    const std::vector<std::shared_ptr<NFP>>& parts,
    const std::vector<std::shared_ptr<NFP>>& sheets,
    const SolverConfig& config,
    std::function<void(const std::string&)> progressCallback)
{
    auto startTime = std::chrono::high_resolution_clock::now();

    // Step 1: Pair pre-matching to reorder parts for interlocking
    if (progressCallback) progressCallback("SmartBLF: computing pair scores...");

    auto pairScores = PairPreMatching::computePairScores(parts, config.baseConfig.rotations);
    auto reorderIndices = PairPreMatching::reorderByPairMatching(parts, pairScores);

    std::vector<std::shared_ptr<NFP>> reorderedParts;
    reorderedParts.reserve(parts.size());
    for (int idx : reorderIndices) {
        reorderedParts.push_back(parts[idx]);
    }

    if (progressCallback) progressCallback("SmartBLF: running BLF with tryAllRotations...");

    // Step 2: Classic BLF with tryAllRotations enabled
    SolverConfig smartConfig = config;
    smartConfig.baseConfig.tryAllRotations = true;
    smartConfig.baseConfig.edgeSamples = 3;
    smartConfig.baseConfig.compactionPasses = 3;

    NestingContext context;
    NestingEngine::Config = smartConfig.baseConfig;

    for (auto& s : sheets) context.Sheets.push_back(s);
    for (auto& p : reorderedParts) context.Polygons.push_back(p);

    NfpWorker::UseParallel = true;
    context.StartNest();

    int maxIter = 1;
    for (int it = 0; it < maxIter; it++) {
        context.NestIterate(maxIter);
        if (progressCallback) {
            progressCallback("SmartBLF: GA iteration " + std::to_string(it + 1));
        }
    }

    // Build result from NestingContext
    SolverResult result;
    result.solverUsed = SolverType::SmartBLF;
    result.solverName = solverName(SolverType::SmartBLF);
    result.fitness = context.HasCurrent() ? context.Current().fitness.value_or(1e9) : 1e9;
    result.materialUtilization = context.MaterialUtilization;
    result.sheetsUsed = static_cast<int>(sheets.size()) - context.SheetsNotUsed;
    result.partsPlaced = context.PlacedPartsCount;
    result.totalParts = static_cast<int>(parts.size());

    // Map reordered indices back to original part indices
    for (size_t pi = 0; pi < context.Polygons.size(); pi++) {
        auto& poly = context.Polygons[pi];
        SolverResult::PartResult pr;
        // Find original index from reorder mapping
        int reorderedIdx = static_cast<int>(pi);
        pr.partIndex = (reorderedIdx < static_cast<int>(reorderIndices.size()))
                        ? reorderIndices[reorderedIdx] : reorderedIdx;
        pr.source = poly->source.value_or(-1);
        pr.x = poly->x;
        pr.y = poly->y;
        pr.rotation = poly->Rotation;
        pr.placed = poly->fitted();
        pr.sheetIndex = poly->sheet ? poly->sheet->Id : -1;
        result.parts.push_back(pr);
    }

    // Step 3: Post-placement compaction
    if (progressCallback) progressCallback("SmartBLF: applying compaction...");

    std::vector<std::shared_ptr<NFP>> placedPolys;
    std::vector<PlacementItem> placements;

    for (auto& pr : result.parts) {
        if (!pr.placed) continue;
        auto rotated = std::make_shared<NFP>(
            NfpWorker::rotatePolygon(*parts[pr.partIndex], pr.rotation));

        PlacementItem pi;
        pi.id = pr.partIndex;
        pi.source = pr.source;
        pi.x = pr.x;
        pi.y = pr.y;
        pi.rotation = 0;

        placedPolys.push_back(rotated);
        placements.push_back(pi);
    }

    SolverConfig compactConfig = config;
    compactConfig.compactionRounds = 10;

    if (!sheets.empty() && !placedPolys.empty()) {
        double moved = Compaction::compact(placedPolys, placements, *sheets[0], compactConfig);
        if (progressCallback) {
            progressCallback("SmartBLF: compaction moved " + std::to_string(moved));
        }
    }

    // Step 4: Jostle
    if (progressCallback) progressCallback("SmartBLF: applying jostle...");

    SolverConfig jostleConfig = config;
    jostleConfig.jostleIterations = 20;

    if (!sheets.empty() && !placedPolys.empty()) {
        double moved = JostleHeuristic::jostle(placedPolys, placements, *sheets[0], jostleConfig);
        if (progressCallback) {
            progressCallback("SmartBLF: jostle moved " + std::to_string(moved));
        }
    }

    // Update positions from compaction+jostle
    {
        size_t placedIdx = 0;
        for (auto& pr : result.parts) {
            if (!pr.placed) continue;
            if (placedIdx < placements.size()) {
                pr.x = placements[placedIdx].x;
                pr.y = placements[placedIdx].y;
                placedIdx++;
            }
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    result.elapsedMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();

    return result;
}

// ============================================================================
// Auto-select and dispatch
// ============================================================================

SolverResult SolverOrchestrator::solveAuto(
    const std::vector<std::shared_ptr<NFP>>& parts,
    const std::vector<std::shared_ptr<NFP>>& sheets,
    const SolverConfig& config,
    std::function<void(const std::string&)> progressCallback)
{
    int n = static_cast<int>(parts.size());

    SolverType selected;
    if (n <= config.exhaustiveMaxN &&
        ExhaustiveSolver::isFeasible(n, config.exhaustiveMaxRotations, config.timeBudgetSeconds)) {
        selected = SolverType::Exhaustive;
    } else if (n <= 20) {
        selected = SolverType::Jostle;
    } else {
        selected = SolverType::OverlapResolution;
    }

    if (progressCallback) {
        progressCallback("Auto-selected: " + solverName(selected) + " for N=" + std::to_string(n));
    }

    SolverConfig autoConfig = config;
    autoConfig.solver = selected;
    return solve(parts, sheets, autoConfig, progressCallback);
}

SolverResult SolverOrchestrator::solve(
    const std::vector<std::shared_ptr<NFP>>& parts,
    const std::vector<std::shared_ptr<NFP>>& sheets,
    const SolverConfig& config,
    std::function<void(const std::string&)> progressCallback)
{
    switch (config.solver) {
        case SolverType::Classic:
            return solveClassic(parts, sheets, config, progressCallback);
        case SolverType::CompactedBLF:
            return solveCompactedBLF(parts, sheets, config, progressCallback);
        case SolverType::Jostle:
            return solveJostle(parts, sheets, config, progressCallback);
        case SolverType::Exhaustive:
            return solveExhaustive(parts, sheets, config, progressCallback);
        case SolverType::OverlapResolution:
            return solveOverlapResolution(parts, sheets, config, progressCallback);
        case SolverType::SmartBLF:
            return solveSmartBLF(parts, sheets, config, progressCallback);
        case SolverType::Auto:
        default:
            return solveAuto(parts, sheets, config, progressCallback);
    }
}

} // namespace nest
