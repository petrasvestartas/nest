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

using namespace nest;

/// Generate an SVG file showing the nesting result.
static void GenerateSvg(
    const std::vector<std::pair<double,double>>& sheetPoints,
    const NestingContext& context,
    const std::string& outputPath)
{
    // Compute bounding box of all geometry
    double minX = 1e30, minY = 1e30, maxX = -1e30, maxY = -1e30;
    for (auto& p : sheetPoints) {
        minX = std::min(minX, p.first);
        minY = std::min(minY, p.second);
        maxX = std::max(maxX, p.first);
        maxY = std::max(maxY, p.second);
    }

    // Collect placed polygon vertices
    struct PlacedPoly {
        std::vector<std::pair<double,double>> pts;
        int source;
    };
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
            minX = std::min(minX, rx);
            minY = std::min(minY, ry);
            maxX = std::max(maxX, rx);
            maxY = std::max(maxY, ry);
        }
        placed.push_back(pp);
    }

    double margin = 20;
    double sceneW = maxX - minX;
    double sceneH = maxY - minY;
    double scale = std::min(800.0 / sceneW, 600.0 / sceneH);
    int imgW = static_cast<int>(sceneW * scale + 2 * margin);
    int imgH = static_cast<int>(sceneH * scale + 2 * margin);

    auto toPixelX = [&](double wx) { return (wx - minX) * scale + margin; };
    auto toPixelY = [&](double wy) { return (wy - minY) * scale + margin; };

    auto pointsStr = [&](const std::vector<std::pair<double,double>>& pts) {
        std::ostringstream ss;
        for (size_t i = 0; i < pts.size(); i++) {
            if (i > 0) ss << " ";
            ss << std::fixed << std::setprecision(2)
               << toPixelX(pts[i].first) << "," << toPixelY(pts[i].second);
        }
        return ss.str();
    };

    // Colors per source
    const char* fills[] = {
        "rgba(66,133,244,0.7)",   // blue
        "rgba(234,67,53,0.7)",    // red
        "rgba(52,168,83,0.7)",    // green
        "rgba(251,188,4,0.7)",    // yellow
    };
    const char* strokes[] = {
        "rgb(33,66,122)",
        "rgb(117,33,26)",
        "rgb(26,84,41)",
        "rgb(125,94,2)",
    };
    int ncolors = 4;

    std::ofstream f(outputPath);
    f << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    f << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << imgW << "\" height=\"" << imgH << "\">\n";
    f << "<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n";

    // Sheet fill
    f << "<polygon points=\"" << pointsStr(sheetPoints)
      << "\" fill=\"#f0f0f0\" stroke=\"black\" stroke-width=\"2\"/>\n";

    // Placed polygons
    for (auto& pp : placed) {
        int ci = pp.source % ncolors;
        f << "<polygon points=\"" << pointsStr(pp.pts)
          << "\" fill=\"" << fills[ci] << "\" stroke=\"" << strokes[ci] << "\" stroke-width=\"1.5\"/>\n";

        // Label at centroid
        double cx = 0, cy = 0;
        for (auto& pt : pp.pts) { cx += pt.first; cy += pt.second; }
        cx /= pp.pts.size(); cy /= pp.pts.size();
        f << "<text x=\"" << std::fixed << std::setprecision(2) << toPixelX(cx)
          << "\" y=\"" << toPixelY(cy)
          << "\" text-anchor=\"middle\" dominant-baseline=\"central\" "
          << "font-family=\"Arial\" font-size=\"12\" font-weight=\"bold\" fill=\"black\">"
          << "P" << pp.source << "</text>\n";
    }

    // Sheet outline on top
    f << "<polygon points=\"" << pointsStr(sheetPoints)
      << "\" fill=\"none\" stroke=\"black\" stroke-width=\"2\"/>\n";

    f << "</svg>\n";
    f.close();
}

/// Create a polygon NFP with first point translated to origin (matching Rhino to_xy transform).
static std::shared_ptr<NFP> CreatePolygon(const std::vector<std::pair<double,double>>& points, int sourceId) {
    double ox = points[0].first;
    double oy = points[0].second;

    auto nfp = std::make_shared<NFP>();
    nfp->source = sourceId;

    int count = static_cast<int>(points.size());
    // Skip closing point if polygon is closed
    if (count > 1 &&
        std::fabs(points[0].first - points[count - 1].first) < 1e-10 &&
        std::fabs(points[0].second - points[count - 1].second) < 1e-10)
        count--;

    for (int i = 0; i < count; i++)
        nfp->AddPoint(Point(points[i].first - ox, points[i].second - oy));

    return nfp;
}

/// Create a sheet NFP (NOT origin-centered — sheets define absolute coordinate space).
static std::shared_ptr<NFP> CreateSheet(const std::vector<std::pair<double,double>>& points, int sourceId) {
    auto nfp = std::make_shared<NFP>();
    nfp->source = sourceId;

    int count = static_cast<int>(points.size());
    if (count > 1 &&
        std::fabs(points[0].first - points[count - 1].first) < 1e-10 &&
        std::fabs(points[0].second - points[count - 1].second) < 1e-10)
        count--;

    for (int i = 0; i < count; i++)
        nfp->AddPoint(Point(points[i].first, points[i].second));

    return nfp;
}

int main(int argc, char* argv[]) {
    std::cout << "OpenNest2 C++ Console Nesting" << std::endl;
    std::cout << "=============================" << std::endl << std::endl;

    // --- Define input geometry ---

    // Sheet: rectangular boundary
    std::vector<std::pair<double,double>> sheetPoints = {
        { 79.854516, -19.170351 },
        { 309.362728, -19.170351 },
        { 309.362728, 174.558717 },
        { 79.854516, 174.558717 },
    };

    // Parts to nest (4 polygons)
    std::vector<std::vector<std::pair<double,double>>> partPoints = {
        // Poly 0: rectangle
        {
            { -20.916485, -15.927695 },
            { 25.619321, -15.927695 },
            { 25.619321, 3.927557 },
            { -20.916485, 3.927557 },
        },
        // Poly 1: rectangle
        {
            { -10.548209, 12.099552 },
            { 17.13848, 12.099552 },
            { 17.13848, 21.620576 },
            { -10.548209, 21.620576 },
        },
        // Poly 2: rectangle
        {
            { 33.052118, 0.885478 },
            { 79.245274, 0.885478 },
            { 79.245274, 45.360619 },
            { 33.052118, 45.360619 },
        },
        // Poly 3: quadrilateral (non-axis-aligned)
        {
            { -40.515726, 50.288638 },
            { -49.871105, 34.614765 },
            { -27.595222, 20.35251 },
            { -23.121731, 35.653932 },
        },
    };

    int maxIterations = 5;

    // Parse command-line args
    if (argc > 1) {
        int userIter = std::atoi(argv[1]);
        if (userIter > 0) maxIterations = userIter;
    }

    // --- Build NestingContext ---
    NestingContext context;

    // Add sheet (NOT origin-centered — defines absolute coordinate space)
    auto sheet = CreateSheet(sheetPoints, 0);
    context.Sheets.push_back(sheet);

    // Add parts (origin-centered: first point translated to 0,0)
    // 2 copies of each part — duplicates of the same source must be consecutive
    int copies = 2;
    for (size_t i = 0; i < partPoints.size(); i++) {
        for (int copy = 0; copy < copies; copy++) {
            auto poly = CreatePolygon(partPoints[i], static_cast<int>(i));
            context.Polygons.push_back(poly);
        }
    }

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "Sheet:  " << sheet->WidthCalculated() << " x " << sheet->HeightCalculated() << std::endl;
    std::cout << "Parts:  " << context.Polygons.size() << std::endl;
    std::cout << "Iterations: " << maxIterations << std::endl << std::endl;

    // --- Configure nesting ---
    context.config.rotations = 4;
    context.config.spacing = 0;
    context.config.clipperScale = 1e7;
    context.config.exploreConcave = true;
    context.config.clipByRects = true;
    context.config.populationSize = 120;
    context.config.mutationRate = 10;
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

    // --- Extract and display results ---
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
        std::cout << "  " << std::left
                  << std::setw(8) << (poly->source.has_value() ? std::to_string(poly->source.value()) : "-")
                  << std::setw(12) << poly->x
                  << std::setw(12) << poly->y
                  << std::setw(10) << std::setprecision(1) << poly->Rotation
                  << std::setw(8) << (poly->fitted() ? "yes" : "no")
                  << std::setw(8) << (poly->sheet && poly->sheet->source.has_value() ? std::to_string(poly->sheet->source.value()) : "-")
                  << std::endl;
        std::cout << std::setprecision(3);
    }

    // --- Print final vertex positions for placed polygons ---
    std::cout << std::endl << "Placed polygon vertices (rotated + translated):" << std::endl;
    for (size_t i = 0; i < context.Polygons.size(); i++) {
        auto& poly = context.Polygons[i];
        if (!poly->fitted()) continue;

        double radians = poly->Rotation * M_PI / 180.0;
        double cosA = std::cos(radians);
        double sinA = std::sin(radians);

        std::cout << "  Polygon " << i << " (source=" << (poly->source.has_value() ? std::to_string(poly->source.value()) : "-") << "):" << std::endl;
        for (size_t j = 0; j < poly->Points.size(); j++) {
            double px = poly->Points[j].x;
            double py = poly->Points[j].y;
            double rx = px * cosA - py * sinA;
            double ry = px * sinA + py * cosA;
            std::cout << std::setprecision(6);
            std::cout << "    (" << (rx + poly->x) << ", " << (ry + poly->y) << ")" << std::endl;
        }
    }

    // --- Generate SVG output ---
    std::string outputPath = "nesting_result.svg";
    GenerateSvg(sheetPoints, context, outputPath);
    std::cout << std::endl << "Result image saved to: " << outputPath << std::endl;

    return 0;
}
