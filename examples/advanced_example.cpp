#include "NestingContext.h"
#include "NestingEngine.h"
#include "NfpWorker.h"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <map>

using namespace nest;

// ============================================================================
// SVG generation with multi-sheet + hole support
// ============================================================================

struct PlacedPoly {
    std::vector<std::pair<double,double>> pts;
    std::vector<std::vector<std::pair<double,double>>> holes;
    int source;
};

struct SheetRender {
    std::vector<std::pair<double,double>> outer;
    std::vector<std::vector<std::pair<double,double>>> holes;
    int source;
};

static std::vector<PlacedPoly> ComputePlacedPolygons(const NestingContext& context) {
    std::vector<PlacedPoly> placed;
    for (size_t i = 0; i < context.Polygons.size(); i++) {
        auto& poly = context.Polygons[i];
        if (!poly->fitted()) continue;

        double radians = poly->Rotation * M_PI / 180.0;
        double cosA = std::cos(radians);
        double sinA = std::sin(radians);

        PlacedPoly pp;
        pp.source = poly->source.has_value() ? poly->source.value() : -1;

        for (size_t j = 0; j < poly->Points.size(); j++) {
            double px = poly->Points[j].x;
            double py = poly->Points[j].y;
            double rx = px * cosA - py * sinA + poly->x;
            double ry = px * sinA + py * cosA + poly->y;
            pp.pts.push_back({rx, ry});
        }

        for (auto& child : poly->children) {
            std::vector<std::pair<double,double>> holePts;
            for (size_t j = 0; j < child->Points.size(); j++) {
                double px = child->Points[j].x;
                double py = child->Points[j].y;
                double rx = px * cosA - py * sinA + poly->x;
                double ry = px * sinA + py * cosA + poly->y;
                holePts.push_back({rx, ry});
            }
            pp.holes.push_back(holePts);
        }

        placed.push_back(pp);
    }
    return placed;
}

static std::vector<SheetRender> ComputeSheetRenderings(const NestingContext& context) {
    std::vector<SheetRender> sheets;
    for (auto& s : context.Sheets) {
        SheetRender sr;
        sr.source = s->source.has_value() ? s->source.value() : -1;
        for (auto& pt : s->Points) {
            sr.outer.push_back({pt.x + s->x, pt.y + s->y});
        }
        for (auto& child : s->children) {
            std::vector<std::pair<double,double>> holePts;
            for (auto& pt : child->Points) {
                holePts.push_back({pt.x + s->x, pt.y + s->y});
            }
            sr.holes.push_back(holePts);
        }
        sheets.push_back(sr);
    }
    return sheets;
}

static void GenerateSvg(
    const std::vector<SheetRender>& sheets,
    const std::vector<PlacedPoly>& placed,
    const std::string& outputPath)
{
    // Compute bounding box of all geometry
    double minX = 1e30, minY = 1e30, maxX = -1e30, maxY = -1e30;
    for (auto& s : sheets) {
        for (auto& p : s.outer) {
            minX = std::min(minX, p.first);  minY = std::min(minY, p.second);
            maxX = std::max(maxX, p.first);  maxY = std::max(maxY, p.second);
        }
    }
    for (auto& pp : placed) {
        for (auto& pt : pp.pts) {
            minX = std::min(minX, pt.first);  minY = std::min(minY, pt.second);
            maxX = std::max(maxX, pt.first);  maxY = std::max(maxY, pt.second);
        }
    }

    double margin = 20;
    double sceneW = maxX - minX;
    double sceneH = maxY - minY;
    double scale = std::min(1200.0 / sceneW, 600.0 / sceneH);
    int imgW = static_cast<int>(sceneW * scale + 2 * margin);
    int imgH = static_cast<int>(sceneH * scale + 2 * margin);

    auto tx = [&](double wx) { return (wx - minX) * scale + margin; };
    auto ty = [&](double wy) { return (wy - minY) * scale + margin; };

    auto polyPoints = [&](const std::vector<std::pair<double,double>>& pts) {
        std::ostringstream ss;
        for (size_t i = 0; i < pts.size(); i++) {
            if (i > 0) ss << " ";
            ss << std::fixed << std::setprecision(2) << tx(pts[i].first) << "," << ty(pts[i].second);
        }
        return ss.str();
    };

    auto pathData = [&](const std::vector<std::pair<double,double>>& outer,
                        const std::vector<std::vector<std::pair<double,double>>>& holes) {
        std::ostringstream ss;
        ss << "M " << std::fixed << std::setprecision(2) << tx(outer[0].first) << "," << ty(outer[0].second);
        for (size_t i = 1; i < outer.size(); i++)
            ss << " L " << tx(outer[i].first) << "," << ty(outer[i].second);
        ss << " Z";
        for (auto& hole : holes) {
            if (hole.empty()) continue;
            ss << " M " << tx(hole[0].first) << "," << ty(hole[0].second);
            for (size_t i = 1; i < hole.size(); i++)
                ss << " L " << tx(hole[i].first) << "," << ty(hole[i].second);
            ss << " Z";
        }
        return ss.str();
    };

    // Part colors (8 colors, wraps with modulo)
    const char* fills[] = {
        "rgba(66,133,244,0.7)",   // blue
        "rgba(234,67,53,0.7)",    // red
        "rgba(52,168,83,0.7)",    // green
        "rgba(251,188,4,0.7)",    // yellow
        "rgba(171,71,188,0.7)",   // purple
        "rgba(255,112,67,0.7)",   // deep orange
        "rgba(0,172,193,0.7)",    // cyan
        "rgba(121,85,72,0.7)",    // brown
    };
    const char* strokes[] = {
        "rgb(33,66,122)", "rgb(117,33,26)", "rgb(26,84,41)",
        "rgb(125,94,2)",  "rgb(85,35,94)",  "rgb(127,56,33)",
        "rgb(0,86,96)",   "rgb(60,42,36)",
    };
    int ncolors = 8;

    std::ofstream f(outputPath);
    f << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    f << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << imgW << "\" height=\"" << imgH << "\">\n";
    f << "<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n";

    // Draw sheets (with holes)
    for (auto& s : sheets) {
        if (s.holes.empty()) {
            f << "<polygon points=\"" << polyPoints(s.outer)
              << "\" fill=\"#e8e8e8\" stroke=\"black\" stroke-width=\"2\"/>\n";
        } else {
            f << "<path d=\"" << pathData(s.outer, s.holes)
              << "\" fill=\"#e8e8e8\" stroke=\"black\" stroke-width=\"2\" fill-rule=\"evenodd\"/>\n";
        }
        // Sheet label near top-center
        double labelX = 0;
        double topY = 1e30;
        for (auto& pt : s.outer) {
            labelX += pt.first;
            topY = std::min(topY, pt.second);
        }
        labelX /= s.outer.size();
        double labelY = topY + 12;
        f << "<text x=\"" << std::fixed << std::setprecision(2) << tx(labelX)
          << "\" y=\"" << ty(labelY)
          << "\" text-anchor=\"middle\" dominant-baseline=\"central\" "
          << "font-family=\"Arial\" font-size=\"13\" fill=\"#888\" font-style=\"italic\">"
          << "Sheet " << s.source << "</text>\n";
    }

    // Draw placed polygons
    for (auto& pp : placed) {
        int ci = ((pp.source % ncolors) + ncolors) % ncolors;

        if (pp.holes.empty()) {
            f << "<polygon points=\"" << polyPoints(pp.pts)
              << "\" fill=\"" << fills[ci] << "\" stroke=\"" << strokes[ci] << "\" stroke-width=\"1.5\"/>\n";
        } else {
            f << "<path d=\"" << pathData(pp.pts, pp.holes)
              << "\" fill=\"" << fills[ci] << "\" stroke=\"" << strokes[ci]
              << "\" stroke-width=\"1.5\" fill-rule=\"evenodd\"/>\n";
        }

        // Label at centroid
        double cx = 0, cy = 0;
        for (auto& pt : pp.pts) { cx += pt.first; cy += pt.second; }
        cx /= pp.pts.size(); cy /= pp.pts.size();
        f << "<text x=\"" << std::fixed << std::setprecision(2) << tx(cx)
          << "\" y=\"" << ty(cy)
          << "\" text-anchor=\"middle\" dominant-baseline=\"central\" "
          << "font-family=\"Arial\" font-size=\"10\" font-weight=\"bold\" fill=\"black\">"
          << "P" << pp.source << "</text>\n";
    }

    // Sheet outlines on top
    for (auto& s : sheets) {
        if (s.holes.empty()) {
            f << "<polygon points=\"" << polyPoints(s.outer)
              << "\" fill=\"none\" stroke=\"black\" stroke-width=\"2\"/>\n";
        } else {
            f << "<path d=\"" << pathData(s.outer, s.holes)
              << "\" fill=\"none\" stroke=\"black\" stroke-width=\"2\" fill-rule=\"evenodd\"/>\n";
        }
    }

    f << "</svg>\n";
    f.close();
}

// ============================================================================
// Main — Multi-sheet test with irregular sheets, holes, and overflow
// ============================================================================

int main(int argc, char* argv[]) {
    std::cout << "OpenNest2 C++ Multi-Sheet Test" << std::endl;
    std::cout << "==============================" << std::endl;
    std::cout << "Testing: multiple sheets, irregular shapes, sheet holes, overflow\n" << std::endl;

    int maxIterations = 10;
    if (argc > 1) {
        int userIter = std::atoi(argv[1]);
        if (userIter > 0) maxIterations = userIter;
    }

    NestingContext context;

    // === Sheet 0: Rectangle 170x120 — small, fills up fast (source=0) ===
    {
        auto s = std::make_shared<NFP>();
        s->source = 0;
        s->AddPoint(Point(0, 0));
        s->AddPoint(Point(170, 0));
        s->AddPoint(Point(170, 120));
        s->AddPoint(Point(0, 120));
        context.Sheets.push_back(s);
    }

    // === Sheet 1: Irregular hexagon ~220x160 with hole (source=1) ===
    {
        auto s = std::make_shared<NFP>();
        s->source = 1;
        s->AddPoint(Point(35, 0));
        s->AddPoint(Point(185, 0));
        s->AddPoint(Point(220, 60));
        s->AddPoint(Point(200, 160));
        s->AddPoint(Point(20, 160));
        s->AddPoint(Point(0, 60));

        // Hole: 60x60 square inside the hexagon
        auto hole = std::make_shared<NFP>();
        hole->AddPoint(Point(80, 55));
        hole->AddPoint(Point(140, 55));
        hole->AddPoint(Point(140, 115));
        hole->AddPoint(Point(80, 115));
        s->children.push_back(hole);

        context.Sheets.push_back(s);
    }

    // === Sheet 2: L-shaped sheet 200x160 (source=2) ===
    {
        auto s = std::make_shared<NFP>();
        s->source = 2;
        s->AddPoint(Point(0, 0));
        s->AddPoint(Point(200, 0));
        s->AddPoint(Point(200, 80));
        s->AddPoint(Point(100, 80));
        s->AddPoint(Point(100, 160));
        s->AddPoint(Point(0, 160));
        context.Sheets.push_back(s);
    }

    // Position sheets side by side
    context.ReorderSheets();

    // === Parts: 15 total (8 source types, some duplicated) ===

    // Source 0: rect 80x50 (x2)
    for (int c = 0; c < 2; c++) {
        auto p = std::make_shared<NFP>();
        p->source = 0;
        p->AddPoint(Point(0, 0)); p->AddPoint(Point(80, 0));
        p->AddPoint(Point(80, 50)); p->AddPoint(Point(0, 50));
        context.Polygons.push_back(p);
    }

    // Source 1: square 55x55 (x2)
    for (int c = 0; c < 2; c++) {
        auto p = std::make_shared<NFP>();
        p->source = 1;
        p->AddPoint(Point(0, 0)); p->AddPoint(Point(55, 0));
        p->AddPoint(Point(55, 55)); p->AddPoint(Point(0, 55));
        context.Polygons.push_back(p);
    }

    // Source 2: triangle 70x55 (x2)
    for (int c = 0; c < 2; c++) {
        auto p = std::make_shared<NFP>();
        p->source = 2;
        p->AddPoint(Point(0, 0)); p->AddPoint(Point(70, 0));
        p->AddPoint(Point(35, 55));
        context.Polygons.push_back(p);
    }

    // Source 3: pentagon r=28 (x2)
    for (int c = 0; c < 2; c++) {
        auto p = std::make_shared<NFP>();
        p->source = 3;
        double r = 28;
        for (int i = 0; i < 5; i++) {
            double a = 2.0 * M_PI * i / 5.0 - M_PI / 2.0;
            p->AddPoint(Point(r * std::cos(a), r * std::sin(a)));
        }
        context.Polygons.push_back(p);
    }

    // Source 4: L-shape 65x45 (x2)
    for (int c = 0; c < 2; c++) {
        auto p = std::make_shared<NFP>();
        p->source = 4;
        p->AddPoint(Point(0, 0)); p->AddPoint(Point(65, 0));
        p->AddPoint(Point(65, 20)); p->AddPoint(Point(30, 20));
        p->AddPoint(Point(30, 45)); p->AddPoint(Point(0, 45));
        context.Polygons.push_back(p);
    }

    // Source 5: rect 90x35 (x2)
    for (int c = 0; c < 2; c++) {
        auto p = std::make_shared<NFP>();
        p->source = 5;
        p->AddPoint(Point(0, 0)); p->AddPoint(Point(90, 0));
        p->AddPoint(Point(90, 35)); p->AddPoint(Point(0, 35));
        context.Polygons.push_back(p);
    }

    // Source 6: small square 40x40 (x2)
    for (int c = 0; c < 2; c++) {
        auto p = std::make_shared<NFP>();
        p->source = 6;
        p->AddPoint(Point(0, 0)); p->AddPoint(Point(40, 0));
        p->AddPoint(Point(40, 40)); p->AddPoint(Point(0, 40));
        context.Polygons.push_back(p);
    }

    // Source 7: rect 70x25 (x1)
    {
        auto p = std::make_shared<NFP>();
        p->source = 7;
        p->AddPoint(Point(0, 0)); p->AddPoint(Point(70, 0));
        p->AddPoint(Point(70, 25)); p->AddPoint(Point(0, 25));
        context.Polygons.push_back(p);
    }

    // --- Print summary ---
    const char* sheetNames[] = { "rect 170x120", "hexagon+hole ~220x160", "L-shape 200x160" };
    const char* partNames[]  = { "rect 80x50", "square 55x55", "triangle 70x55",
                                 "pentagon r=28", "L-shape 65x45", "rect 90x35",
                                 "square 40x40", "rect 70x25" };

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "Sheets: " << context.Sheets.size() << std::endl;
    for (size_t i = 0; i < context.Sheets.size(); i++) {
        auto& s = context.Sheets[i];
        int src = s->source.value();
        std::cout << "  S" << src << ": " << sheetNames[src]
                  << " (" << s->Points.size() << " pts"
                  << (s->children.empty() ? "" : ", " + std::to_string(s->children.size()) + " hole(s)")
                  << ")" << std::endl;
    }

    std::cout << "Parts:  " << context.Polygons.size() << std::endl;
    std::map<int, int> srcCount;
    for (auto& p : context.Polygons)
        srcCount[p->source.value()]++;
    for (auto& [src, cnt] : srcCount) {
        std::cout << "  P" << src << ": " << partNames[src] << " x" << cnt << std::endl;
    }
    std::cout << "Iterations: " << maxIterations << std::endl << std::endl;

    // --- Configure ---
    context.config.rotations = 4;
    context.config.spacing = 3;
    context.config.clipperScale = 1e7;
    context.config.exploreConcave = true;
    context.config.clipByRects = true;
    context.config.populationSize = 120;
    context.config.mutationRate = 10;
    context.config.edgeSamples = 2;
    context.config.compactionPasses = 2;
    context.config.gravityWeight = 0.1;
    NfpWorker::UseParallel = true;

    // --- Run nesting ---
    std::cout << "Starting nesting..." << std::endl << std::endl;
    context.StartNest();

    auto totalStart = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < maxIterations; i++) {
        auto iterStart = std::chrono::high_resolution_clock::now();
        context.NestIterate(maxIterations);
        auto iterEnd = std::chrono::high_resolution_clock::now();
        auto iterMs = std::chrono::duration_cast<std::chrono::milliseconds>(iterEnd - iterStart).count();

        std::cout << std::setprecision(2);
        if (context.HasCurrent()) {
            auto& cur = context.Current();
            std::cout << "  Iteration " << context.Iterations
                      << ": fitness=" << (cur.fitness.has_value() ? cur.fitness.value() : 0.0)
                      << "  (" << iterMs << "ms)" << std::endl;
        } else {
            std::cout << "  Iteration " << context.Iterations
                      << ": no placement  (" << iterMs << "ms)" << std::endl;
        }
    }
    auto totalEnd = std::chrono::high_resolution_clock::now();
    auto totalMs = std::chrono::duration_cast<std::chrono::milliseconds>(totalEnd - totalStart).count();

    std::cout << std::endl;
    std::cout << "Nesting completed in " << totalMs << "ms" << std::endl;
    std::cout << std::setprecision(1);
    std::cout << "Material utilization: " << context.MaterialUtilization * 100 << "%" << std::endl;
    std::cout << "Placed parts: " << context.PlacedPartsCount << " / " << context.Polygons.size() << std::endl;
    std::cout << "Sheets not used: " << context.SheetsNotUsed << std::endl << std::endl;

    // --- Results table ---
    std::cout << "Results per polygon:" << std::endl;
    std::cout << "  " << std::left
              << std::setw(8)  << "Source"
              << std::setw(12) << "X"
              << std::setw(12) << "Y"
              << std::setw(10) << "Rotation"
              << std::setw(8)  << "Fitted"
              << std::setw(8)  << "Sheet"
              << std::endl;

    std::cout << std::setprecision(3);
    for (size_t i = 0; i < context.Polygons.size(); i++) {
        auto& poly = context.Polygons[i];
        std::string sheetStr = poly->fitted() ? ("S" + std::to_string(poly->sheet->source.value())) : "-";
        std::cout << "  " << std::left
                  << std::setw(8) << ("P" + std::to_string(poly->source.value()))
                  << std::setw(12) << poly->x
                  << std::setw(12) << poly->y
                  << std::setw(10) << std::setprecision(1) << poly->Rotation
                  << std::setw(8) << (poly->fitted() ? "yes" : "no")
                  << std::setw(8) << sheetStr
                  << std::endl;
        std::cout << std::setprecision(3);
    }

    // --- Generate SVG ---
    auto sheetRenderings = ComputeSheetRenderings(context);
    auto placedPolys = ComputePlacedPolygons(context);
    std::string outputPath = "advanced_result.svg";
    GenerateSvg(sheetRenderings, placedPolys, outputPath);
    std::cout << std::endl << "Result image saved to: " << outputPath << std::endl;

    return 0;
}
