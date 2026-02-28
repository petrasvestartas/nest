#include "NestingContext.h"
#include "NestingEngine.h"
#include "NfpWorker.h"
#include "SolverOrchestrator.h"
#include "SolverConfig.h"
#include "Compaction.h"
#include "JostleHeuristic.h"
#include "ContinuousRotation.h"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <sstream>
#include <vector>
#include <algorithm>

using namespace nest;

// ============================================================================
// Polygon builder: concatenate polyline segments into closed polygons
// ============================================================================

/// A single polyline segment (row of points)
struct Segment {
    std::vector<std::pair<double, double>> points;
};

/// Check if two points are the same (within tolerance)
static bool samePoint(const std::pair<double, double>& a,
                      const std::pair<double, double>& b, double tol = 0.01) {
    return std::fabs(a.first - b.first) < tol && std::fabs(a.second - b.second) < tol;
}

/// Build closed polygons from a list of segments.
/// Segments that share endpoints are chained together.
static std::vector<std::vector<std::pair<double, double>>> buildPolygons(
    std::vector<Segment>& segments)
{
    std::vector<std::vector<std::pair<double, double>>> polygons;
    std::vector<bool> used(segments.size(), false);

    while (true) {
        // Find first unused segment
        int startIdx = -1;
        for (size_t i = 0; i < segments.size(); i++) {
            if (!used[i]) { startIdx = static_cast<int>(i); break; }
        }
        if (startIdx < 0) break;

        // Start building a polygon
        std::vector<std::pair<double, double>> poly;
        auto& startSeg = segments[startIdx];
        for (auto& pt : startSeg.points) poly.push_back(pt);
        used[startIdx] = true;

        // Keep extending
        bool extended = true;
        while (extended) {
            extended = false;
            auto& lastPt = poly.back();
            auto& firstPt = poly.front();

            // Check if closed
            if (poly.size() > 3 && samePoint(lastPt, firstPt)) {
                break;
            }

            for (size_t i = 0; i < segments.size(); i++) {
                if (used[i]) continue;
                auto& seg = segments[i];
                if (seg.points.empty()) continue;

                if (samePoint(lastPt, seg.points.front())) {
                    // Append segment (skip first point = duplicate)
                    for (size_t j = 1; j < seg.points.size(); j++) {
                        poly.push_back(seg.points[j]);
                    }
                    used[i] = true;
                    extended = true;
                    break;
                } else if (samePoint(lastPt, seg.points.back())) {
                    // Append reversed segment
                    for (int j = static_cast<int>(seg.points.size()) - 2; j >= 0; j--) {
                        poly.push_back(seg.points[j]);
                    }
                    used[i] = true;
                    extended = true;
                    break;
                }
            }
        }

        // Remove closing duplicate if present
        if (poly.size() > 1 && samePoint(poly.front(), poly.back())) {
            poly.pop_back();
        }

        if (poly.size() >= 3) {
            polygons.push_back(poly);
        }
    }

    return polygons;
}

/// Create an NFP polygon from points, origin-centered (first point = 0,0)
static std::shared_ptr<NFP> CreatePolygon(
    const std::vector<std::pair<double, double>>& points, int sourceId)
{
    if (points.empty()) return nullptr;

    double ox = points[0].first;
    double oy = points[0].second;

    auto nfp = std::make_shared<NFP>();
    nfp->source = sourceId;

    for (size_t i = 0; i < points.size(); i++) {
        nfp->AddPoint(Point(points[i].first - ox, points[i].second - oy));
    }

    return nfp;
}

/// Create a sheet NFP (NOT origin-centered)
static std::shared_ptr<NFP> CreateSheet(
    const std::vector<std::pair<double, double>>& points, int sourceId)
{
    auto nfp = std::make_shared<NFP>();
    nfp->source = sourceId;

    int count = static_cast<int>(points.size());
    if (count > 1 && samePoint(points[0], points[count - 1])) count--;

    for (int i = 0; i < count; i++) {
        nfp->AddPoint(Point(points[i].first, points[i].second));
    }

    return nfp;
}

// ============================================================================
// SVG output
// ============================================================================

static void GenerateSolverSvg(
    const std::vector<std::shared_ptr<NFP>>& sheets,
    const std::vector<std::shared_ptr<NFP>>& parts,
    const SolverResult& result,
    const std::string& outputPath)
{
    double minX = 1e30, minY = 1e30, maxX = -1e30, maxY = -1e30;

    // Sheet bounds
    for (auto& sheet : sheets) {
        for (int i = 0; i < sheet->length(); i++) {
            minX = std::min(minX, sheet->Points[i].x);
            minY = std::min(minY, sheet->Points[i].y);
            maxX = std::max(maxX, sheet->Points[i].x);
            maxY = std::max(maxY, sheet->Points[i].y);
        }
    }

    // Placed part bounds
    struct PlacedPoly {
        std::vector<std::pair<double, double>> pts;
        int source;
    };
    std::vector<PlacedPoly> placed;

    for (auto& pr : result.parts) {
        if (!pr.placed) continue;
        auto& poly = parts[pr.partIndex];

        double rad = pr.rotation * M_PI / 180.0;
        double cosA = std::cos(rad);
        double sinA = std::sin(rad);

        PlacedPoly pp;
        pp.source = pr.source;

        for (int j = 0; j < poly->length(); j++) {
            double px = poly->Points[j].x;
            double py = poly->Points[j].y;
            double rx = px * cosA - py * sinA + pr.x;
            double ry = px * sinA + py * cosA + pr.y;
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
    if (sceneW < 1) sceneW = 1;
    if (sceneH < 1) sceneH = 1;
    double scale = std::min(1200.0 / sceneW, 800.0 / sceneH);
    int imgW = static_cast<int>(sceneW * scale + 2 * margin);
    int imgH = static_cast<int>(sceneH * scale + 2 * margin);

    auto toPixelX = [&](double wx) { return (wx - minX) * scale + margin; };
    auto toPixelY = [&](double wy) { return (wy - minY) * scale + margin; };

    auto pointsStr = [&](const std::vector<std::pair<double, double>>& pts) {
        std::ostringstream ss;
        for (size_t i = 0; i < pts.size(); i++) {
            if (i > 0) ss << " ";
            ss << std::fixed << std::setprecision(2)
               << toPixelX(pts[i].first) << "," << toPixelY(pts[i].second);
        }
        return ss.str();
    };

    const char* fills[] = {
        "rgba(66,133,244,0.6)", "rgba(234,67,53,0.6)",
        "rgba(52,168,83,0.6)",  "rgba(251,188,4,0.6)",
        "rgba(156,39,176,0.6)", "rgba(0,188,212,0.6)",
        "rgba(255,87,34,0.6)",  "rgba(121,85,72,0.6)",
        "rgba(96,125,139,0.6)", "rgba(139,195,74,0.6)",
    };
    const char* strokes[] = {
        "rgb(33,66,122)",   "rgb(117,33,26)",
        "rgb(26,84,41)",    "rgb(125,94,2)",
        "rgb(78,19,88)",    "rgb(0,94,106)",
        "rgb(127,43,17)",   "rgb(60,42,36)",
        "rgb(48,62,69)",    "rgb(69,97,37)",
    };
    int ncolors = 10;

    std::ofstream f(outputPath);
    f << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    f << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << imgW << "\" height=\"" << imgH << "\">\n";
    f << "<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n";

    // Sheet fills
    for (auto& sheet : sheets) {
        std::vector<std::pair<double, double>> sheetPts;
        for (int i = 0; i < sheet->length(); i++) {
            sheetPts.push_back({sheet->Points[i].x, sheet->Points[i].y});
        }
        f << "<polygon points=\"" << pointsStr(sheetPts)
          << "\" fill=\"#f0f0f0\" stroke=\"black\" stroke-width=\"1.5\"/>\n";
    }

    // Placed polygons
    for (auto& pp : placed) {
        int ci = (pp.source >= 0 ? pp.source : 0) % ncolors;
        f << "<polygon points=\"" << pointsStr(pp.pts)
          << "\" fill=\"" << fills[ci] << "\" stroke=\"" << strokes[ci]
          << "\" stroke-width=\"1\"/>\n";
    }

    // Title
    f << "<text x=\"10\" y=\"" << (imgH - 10)
      << "\" font-family=\"Arial\" font-size=\"14\" fill=\"#333\">"
      << result.solverName << " | "
      << result.partsPlaced << "/" << result.totalParts << " placed | "
      << std::fixed << std::setprecision(1) << result.materialUtilization * 100 << "% util | "
      << std::setprecision(0) << result.elapsedMs << "ms</text>\n";

    f << "</svg>\n";
    f.close();
}

// ============================================================================
// Sample data from user
// ============================================================================

static void buildSampleSheets(std::vector<std::shared_ptr<NFP>>& sheets) {
    // 5 rectangular sheets, all 2440x1220, normalized to origin
    double sheetData[][5][2] = {
        {{5440,0},{3000,0},{3000,1220},{5440,1220},{5440,0}},
        {{0,0},{2440,0},{2440,1220},{0,1220},{0,0}},
        {{8262,0},{5822,0},{5822,1220},{8262,1220},{8262,0}},
        {{11084,0},{8644,0},{8644,1220},{11084,1220},{11084,0}},
        {{13906,0},{11466,0},{11466,1220},{13906,1220},{13906,0}},
    };

    for (int s = 0; s < 5; s++) {
        std::vector<std::pair<double, double>> pts;
        for (int i = 0; i < 5; i++) {
            pts.push_back({sheetData[s][i][0], sheetData[s][i][1]});
        }
        sheets.push_back(CreateSheet(pts, 100 + s));
    }
}

static void buildSamplePolygons(std::vector<std::shared_ptr<NFP>>& polygons) {
    // ========================================================================
    // Polygon type A: Large arch shape (3 copies at Y offsets -1236, -2236, -3236)
    // Each is 4 segments that form a closed polygon
    // ========================================================================
    double archA_seg1[][2] = {
        {6279.178492,-1236.292576},{6274.715632,-1199.971948},{6268.599022,-1163.89302},{6260.844367,-1128.130335},{6251.467374,-1092.758438},{6239.496411,-1054.959638},{6225.671883,-1017.798614},{6210.027732,-981.366257},{6192.603221,-945.751119},{6173.440215,-911.040508},{6152.585674,-877.319092},{6130.089358,-844.669926},{6106.007155,-813.172252},{6082.42434,-785.190958},{6057.590014,-758.314229},{6031.55543,-732.598353},{6004.375171,-708.096571},{5976.105413,-684.860405},{5946.805409,-662.937547},{5916.535828,-642.373932},{5885.359929,-623.212278},{5850.632452,-604.079937},{5815.002114,-586.686537},{5778.555669,-571.074848},{5741.3824,-557.283371},{5703.572786,-545.347386},{5665.219566,-535.293535},{5626.416361,-527.14574},{5587.258047,-520.925662},{5550.88523,-516.911527},{5514.367114,-514.560258},{5477.779795,-513.874736},{5441.199368,-514.85784},{5404.701895,-517.510455},{5368.36331,-521.825481},{5332.259357,-527.79312},{5296.465151,-535.400763},{5258.117978,-545.477753},{5220.315631,-557.437406},{5183.15071,-571.251071},{5146.713483,-586.883749},{5111.093102,-604.297636},{5076.376568,-623.450054},{5042.649428,-644.295514},{5009.993549,-666.782145},{4980.874494,-688.944807},{4952.796432,-712.412344},{4925.818165,-737.136366},{4899.995514,-763.065},{4875.382682,-790.144694},{4852.030362,-818.31865},{4829.987534,-847.528569},{4809.299809,-877.71337},{4788.464567,-911.446793},{4769.321366,-946.168419},{4751.917527,-981.793755},{4736.295267,-1018.235479},{4722.493813,-1055.404927},{4710.545501,-1093.210907},{4700.479793,-1131.56119},{4692.321524,-1170.361951},{4686.503365,-1206.490171},{4682.338317,-1242.846126},{4679.000369,-1315.938384}
    };
    double archA_seg2[][2] = {
        {4679.000369,-1315.938384},{4726.620806,-1315.938384},{4774.241244,-1315.938384},{4821.861682,-1315.938384},{4869.48212,-1315.938384},{4964.722995,-1315.938384}
    };
    double archA_seg3[][2] = {
        {4964.722995,-1315.938384},{4966.871862,-1268.883855},{4973.298741,-1222.220791},{4978.550782,-1197.242065},{4985.030772,-1172.553342},{4992.722725,-1148.215024},{5001.607674,-1124.286486},{5011.664799,-1100.826435},{5022.868852,-1077.891948},{5035.19265,-1055.539242},{5048.605733,-1033.822711},{5106.992816,-960.015646},{5177.806139,-898.03173},{5198.828981,-883.555534},{5220.541468,-870.135874},{5242.890896,-857.806141},{5265.822192,-846.595621},{5289.279349,-836.531787},{5313.204973,-827.638978},{5337.540951,-819.939724},{5362.227672,-813.452471},{5455.403072,-800.227574},{5502.506198,-800.036},{5549.431119,-804.13385},{5574.640028,-808.138143},{5599.620328,-813.383441},{5624.310941,-819.855798},{5648.651598,-827.539815},{5672.582596,-836.41834},{5696.045687,-846.468662},{5718.983393,-857.665993},{5741.339866,-869.982801},{5817.958188,-924.628676},{5883.385707,-992.273141},{5898.889082,-1012.550366},{5913.371513,-1033.568887},{5926.79702,-1055.277689},{5939.133568,-1077.623304},{5950.350928,-1100.551225},{5960.422148,-1124.005246},{5969.32195,-1147.928361},{5977.028485,-1172.262056},{5994.868043,-1264.664883}
    };
    double archA_seg4[][2] = {
        {5994.868043,-1264.664883},{6042.253118,-1259.936165},{6089.638192,-1255.207447},{6137.023267,-1250.47873},{6184.408342,-1245.750012},{6279.178492,-1236.292576}
    };

    // Build arch type A polygons at 3 Y offsets
    double yOffsets[] = {0, -1000, -2000};
    for (int copy = 0; copy < 3; copy++) {
        double yOff = yOffsets[copy];
        std::vector<Segment> segs;

        auto addSeg = [&](auto& data, int count) {
            Segment seg;
            for (int i = 0; i < count; i++) {
                seg.points.push_back({data[i][0], data[i][1] + yOff});
            }
            segs.push_back(seg);
        };

        addSeg(archA_seg1, 64);
        addSeg(archA_seg2, 6);
        addSeg(archA_seg3, 43);
        addSeg(archA_seg4, 6);

        auto polys = buildPolygons(segs);
        for (auto& pts : polys) {
            polygons.push_back(CreatePolygon(pts, 0 + copy));
        }
    }

    // ========================================================================
    // Polygon type B: Medium arch (1 copy)
    // ========================================================================
    double archB_seg1[][2] = {
        {4568.304928,-1522.405068},{4578.040446,-1487.13056},{4585.945753,-1451.401335},{4591.999369,-1415.312196},{4596.185937,-1378.959143},{4598.600858,-1339.385131},{4598.803574,-1299.738025},{4596.792246,-1260.141451},{4592.574327,-1220.718821},{4586.161857,-1181.593189},{4577.575927,-1142.886391},{4566.842156,-1104.71939},{4553.995211,-1067.210858},{4540.296522,-1033.27831},{4524.868055,-1000.096469},{4507.750114,-967.753806},{4488.988881,-936.335861},{4468.633442,-905.926537},{4446.738595,-876.606116},{4423.361855,-848.452961},{4398.5659,-821.54139},{4370.173732,-793.867953},{4340.2815,-767.822064},{4308.981605,-743.485867},{4276.372372,-720.934316},{4242.554855,-700.238654},{4207.635098,-681.462474},{4171.721435,-664.665372},{4134.926376,-649.898598},{4100.28315,-638.112218},{4065.078765,-628.126057},{4029.406539,-619.967374},{3993.361415,-613.657055},{3957.038986,-609.212725},{3920.535838,-606.645315},{3883.948876,-605.962445},{3847.375313,-607.165184},{3807.876211,-610.593772},{3768.629997,-616.221258},{3729.759225,-624.031313},{3691.384869,-633.998425},{3653.626877,-646.092593},{3616.602662,-660.275042},{3580.428105,-676.502652},{3545.215496,-694.723687},{3513.665229,-713.261554},{3483.111786,-733.400056},{3453.636752,-755.086324},{3425.317951,-778.2621},{3398.231158,-802.8665},{3372.447728,-828.83352},{3348.036716,-856.09475},{3325.062421,-884.577269},{3301.863174,-916.728927},{3280.494298,-950.125109},{3261.023458,-984.662342},{3243.51035,-1020.232352},{3228.010594,-1056.724711},{3214.571488,-1094.025177},{3203.236045,-1132.01783},{3194.038451,-1170.583855},{3187.472591,-1206.583318},{3182.769854,-1242.873214},{3179.000369,-1315.938384}
    };
    double archB_seg2[][2] = {
        {3179.000369,-1315.938384},{3228.771881,-1315.938384},{3278.543393,-1315.938384},{3328.314904,-1315.938384},{3378.086416,-1315.938384},{3477.62944,-1315.938384}
    };
    double archB_seg3[][2] = {
        {3477.62944,-1315.938384},{3486.342386,-1231.72112},{3515.006012,-1144.608648},{3562.256637,-1066.011167},{3620.343898,-1004.412999},{3689.811587,-956.011504},{3774.500965,-920.827767},{3864.880594,-905.2808},{3949.463786,-909.042139},{4031.485405,-930.040202},{4113.438096,-971.197735},{4184.236059,-1029.488677},{4236.626419,-1095.999686},{4274.290572,-1171.827623},{4296.643007,-1260.768923},{4298.735545,-1352.452113},{4282.581509,-1435.563547}
    };
    double archB_seg4[][2] = {
        {4282.581509,-1435.563547},{4330.202079,-1450.037134},{4377.822649,-1464.51072},{4425.443219,-1478.984307},{4473.063788,-1493.457894},{4568.304928,-1522.405068}
    };

    {
        std::vector<Segment> segs;
        auto addSeg = [&](auto& data, int count) {
            Segment seg;
            for (int i = 0; i < count; i++) seg.points.push_back({data[i][0], data[i][1]});
            segs.push_back(seg);
        };
        addSeg(archB_seg1, 64);
        addSeg(archB_seg2, 6);
        addSeg(archB_seg3, 17);
        addSeg(archB_seg4, 6);
        auto polys = buildPolygons(segs);
        for (auto& pts : polys) polygons.push_back(CreatePolygon(pts, 3));
    }

    // ========================================================================
    // Polygon type C: Small-medium arch (3 copies at Y offsets 0, -1000, -2000)
    // ========================================================================
    double archC_seg1[][2] = {
        {2791.167483,-1729.815439},{2814.074863,-1701.711259},{2835.335996,-1672.34193},{2854.880638,-1641.803406},{2872.645488,-1610.196382},{2890.101116,-1574.226878},{2905.252328,-1537.227628},{2918.037048,-1499.345514},{2928.405934,-1460.732164},{2936.316061,-1421.541157},{2941.737456,-1381.92913},{2944.646776,-1342.053835},{2945.034251,-1302.074421},{2943.202324,-1265.863402},{2939.300938,-1229.816577},{2933.3419,-1194.052296},{2925.345821,-1158.687688},{2915.33775,-1123.839002},{2903.351675,-1089.620186},{2889.42589,-1056.143823},{2873.60698,-1023.51935},{2854.029143,-988.659452},{2832.290733,-955.104349},{2808.476917,-922.988847},{2782.684054,-892.439988},{2755.013778,-863.580659},{2725.577616,-836.524853},{2694.491799,-811.381758},{2661.881443,-788.250212},{2631.099277,-769.091609},{2599.271058,-751.726158},{2566.500714,-736.211697},{2532.896194,-722.598137},{2498.567257,-710.931242},{2463.626871,-701.248124},{2428.189342,-693.581577},{2392.371125,-687.955792},{2352.572209,-684.141279},{2312.611921,-682.8456},{2272.649562,-684.075738},{2232.844448,-687.825057},{2193.355487,-694.080217},{2154.339934,-702.814678},{2115.953777,-713.995297},{2078.349579,-727.575725},{2045.051213,-741.921898},{2012.628494,-758.150319},{1981.188222,-776.208648},{1950.832983,-796.036716},{1921.662993,-817.570503},{1893.773188,-840.738414},{1867.255724,-865.465294},{1842.196922,-891.669314},{1816.354062,-922.175882},{1792.487607,-954.252285},{1770.694185,-987.771686},{1751.059271,-1022.599472},{1733.662595,-1058.597536},{1718.572075,-1095.621587},{1705.849487,-1133.524611},{1695.543872,-1172.154884},{1688.324161,-1207.686133},{1683.149794,-1243.572351},{1679.000369,-1315.938384}
    };
    double archC_seg2[][2] = {
        {1679.000369,-1315.938384},{1724.221281,-1315.938384},{1769.442192,-1315.938384},{1814.663104,-1315.938384},{1859.884016,-1315.938384},{1905.104928,-1315.938384},{1995.546752,-1315.938384}
    };
    double archC_seg3[][2] = {
        {1995.546752,-1315.938384},{2003.818503,-1244.046634},{2031.576202,-1169.268928},{2077.145028,-1103.803849},{2131.463058,-1055.98755},{2195.221356,-1021.757054},{2272.468791,-1001.88172},{2352.232129,-1001.947088},{2422.494664,-1019.26826},{2486.987288,-1052.094298},{2547.388594,-1104.189186},{2592.850057,-1169.728867},{2618.719477,-1237.313036},{2628.563692,-1309.006402},{2620.249534,-1388.335274},{2592.369311,-1463.067383},{2551.630308,-1522.876911}
    };
    double archC_seg4[][2] = {
        {2551.630308,-1522.876911},{2585.849905,-1552.439558},{2620.069501,-1582.002205},{2654.289097,-1611.564852},{2688.508694,-1641.127498},{2722.72829,-1670.690145},{2791.167483,-1729.815439}
    };

    {
        std::vector<Segment> segs;
        auto addSeg = [&](auto& data, int count) {
            Segment seg;
            for (int i = 0; i < count; i++) seg.points.push_back({data[i][0], data[i][1]});
            segs.push_back(seg);
        };
        addSeg(archC_seg1, 64);
        addSeg(archC_seg2, 7);
        addSeg(archC_seg3, 17);
        addSeg(archC_seg4, 7);
        auto polys = buildPolygons(segs);
        for (auto& pts : polys) polygons.push_back(CreatePolygon(pts, 4));
    }

    // ========================================================================
    // Polygon type D: Small arch (3 copies at Y offsets 0, -1000, -2000)
    // ========================================================================
    double archD_seg1[][2] = {
        {1131.527459,-1798},{1160.069469,-1775.660978},{1187.206587,-1751.634779},{1212.838448,-1726.008897},{1236.871972,-1698.878319},{1261.177825,-1667.660847},{1283.367836,-1634.905614},{1303.750854,-1599.999972},{1321.731492,-1563.798147},{1350.164261,-1488.169983},{1367.876613,-1411.094482},{1372.554653,-1375.152902},{1375.048345,-1338.994072},{1375.347071,-1302.750599},{1373.451108,-1266.555511},{1369.366113,-1230.541726},{1363.108361,-1194.841304},{1354.699461,-1159.585528},{1344.171733,-1124.903435},{1314.019644,-1051.792471},{1273.53159,-981.872816},{1250.102545,-949.461604},{1224.561249,-918.687486},{1197.020116,-889.689403},{1167.603797,-862.595406},{1139.434014,-839.788734},{1109.93516,-818.729326},{1079.214838,-799.495477},{1047.386505,-782.156632},{1014.566301,-766.777637},{980.875261,-753.413761},{946.436553,-742.11533},{911.377012,-732.922437},{833.302426,-720.324139},{792.954003,-717.898776},{752.533925,-718.20249},{712.226761,-721.236459},{672.216266,-726.984145},{595.448014,-745.984044},{561.266975,-758.039619},{527.878442,-772.14228},{495.405502,-788.241591},{463.966765,-806.277258},{433.678307,-826.184233},{404.650525,-847.888318},{376.990749,-871.310926},{350.799729,-896.365066},{299.181907,-956.28143},{275.986973,-989.385304},{255.079457,-1023.979329},{236.556203,-1059.906581},{220.500376,-1097.002236},{207.247528,-1134.280421},{196.486785,-1172.352842},{188.857878,-1207.785592},{183.387931,-1243.615178},{179.000369,-1315.938384}
    };
    double archD_seg2[][2] = {
        {179.000369,-1315.938384},{225.95087,-1315.938384},{272.901371,-1315.938384},{319.851872,-1315.938384},{366.802373,-1315.938384},{413.752875,-1315.938384},{507.653877,-1315.938384}
    };
    double archD_seg3[][2] = {
        {507.653877,-1315.938384},{515.533925,-1251.23324},{541.938027,-1184.370454},{572.996301,-1139.978223},{609.340477,-1105.017102},{664.871695,-1070.883037},{712.476877,-1054.241904},{766.110058,-1046.575702},{820.193022,-1049.788543},{869.009407,-1062.44325},{927.16874,-1091.876944},{978.823644,-1136.921918},{1010.546713,-1180.84158},{1032.724708,-1229.850721},{1045.919116,-1293.68457},{1043.407035,-1358.819365},{1029.594704,-1410.809554},{1005.324144,-1459.677231},{936.899056,-1533.173806}
    };
    double archD_seg4[][2] = {
        {936.899056,-1533.173806},{964.703114,-1571.00612},{992.507171,-1608.838433},{1020.311229,-1646.670746},{1048.115287,-1684.50306},{1075.919344,-1722.335373},{1131.527459,-1798}
    };

    double archD_yOffsets[] = {0, -1000, -2000};
    for (int copy = 0; copy < 3; copy++) {
        double yOff = archD_yOffsets[copy];
        std::vector<Segment> segs;
        auto addSeg = [&](auto& data, int count) {
            Segment seg;
            for (int i = 0; i < count; i++) seg.points.push_back({data[i][0], data[i][1] + yOff});
            segs.push_back(seg);
        };
        addSeg(archD_seg1, 57);
        addSeg(archD_seg2, 7);
        addSeg(archD_seg3, 19);
        addSeg(archD_seg4, 7);
        auto polys = buildPolygons(segs);
        for (auto& pts : polys) polygons.push_back(CreatePolygon(pts, 5 + copy));
    }

    // ========================================================================
    // Polygon type E: Rectangular panels (8 copies)
    // Each panel has 6 segments forming a closed octagonal shape
    // ========================================================================
    double panelYStarts[] = {-970, -1300, -1630, -1960, -2290, -2620, -2950, -3280};
    double panelHeight = 330; // -970 to -1100 = 130 body, then extended

    for (int p = 0; p < 8; p++) {
        double yBase = panelYStarts[p];
        double yMid = yBase - panelHeight; // lower Y (more negative)

        // Build the panel polygon directly
        // Top-left slant → top edge → top-right slant → right edge → bottom edge → left edge
        std::vector<std::pair<double, double>> pts;

        // Top-left slant (going up = less negative Y)
        pts.push_back({6717, yBase});
        pts.push_back({6723.5, yBase + 40});
        pts.push_back({6730.0, yBase + 80});
        pts.push_back({6736.5, yBase + 120});
        pts.push_back({6749.5, yBase + 200});

        // Top edge (going right)
        double yTop = yBase + 200;
        for (double x = 6795.75; x < 7305; x += 46.25) {
            pts.push_back({x, yTop});
        }
        pts.push_back({7304.5, yTop});

        // Top-right slant (going down)
        pts.push_back({7311.0, yBase + 160});
        pts.push_back({7317.5, yBase + 120});
        pts.push_back({7324.0, yBase + 80});
        pts.push_back({7337.0, yBase});

        // Right edge (going down to bottom)
        double yBottom = yBase - 130;
        pts.push_back({7337.0, yBase - 60});
        pts.push_back({7337.0, yBase - 110});
        pts.push_back({7337.0, yBottom});

        // Bottom edge (going left)
        for (double x = 7289.307692; x > 6717; x -= 47.692308) {
            pts.push_back({x, yBottom});
        }
        pts.push_back({6717, yBottom});

        // Left edge (going up)
        pts.push_back({6717, yBase - 110});
        pts.push_back({6717, yBase - 60});

        polygons.push_back(CreatePolygon(pts, 8 + p));
    }
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
    std::cout << "OpenNest2 Advanced Solvers Demo" << std::endl;
    std::cout << "===============================" << std::endl << std::endl;

    // Build geometry
    std::vector<std::shared_ptr<NFP>> sheets;
    std::vector<std::shared_ptr<NFP>> polygons;

    buildSampleSheets(sheets);
    buildSamplePolygons(polygons);

    std::cout << "Sheets:   " << sheets.size() << std::endl;
    std::cout << "Polygons: " << polygons.size() << std::endl << std::endl;

    // Print polygon info
    for (size_t i = 0; i < polygons.size(); i++) {
        auto& p = polygons[i];
        std::cout << "  Part " << std::setw(2) << i
                  << " (src=" << p->source.value_or(-1) << ")"
                  << "  " << std::setw(7) << std::fixed << std::setprecision(1)
                  << p->WidthCalculated() << " x "
                  << std::setw(7) << p->HeightCalculated()
                  << "  area=" << std::setw(10) << std::setprecision(0)
                  << std::fabs(GeometryUtil::polygonArea(*p))
                  << "  pts=" << p->length()
                  << std::endl;
    }
    std::cout << std::endl;

    // ========================================================================
    // Run solvers and compare
    // ========================================================================

    SolverConfig config;
    config.baseConfig.rotations = 4;
    config.baseConfig.spacing = 0;
    config.baseConfig.clipperScale = 1e7;
    config.baseConfig.exploreConcave = true;
    config.baseConfig.clipByRects = true;
    config.baseConfig.populationSize = 120;
    config.baseConfig.mutationRate = 10;
    config.compactionRounds = 5;
    config.jostleIterations = 10;
    config.timeBudgetSeconds = 30;

    struct RunResult {
        std::string name;
        SolverResult result;
    };
    std::vector<RunResult> results;

    // --- Classic GA ---
    if (false) {
        std::cout << "Running Classic (GA+BLF)..." << std::endl;
        config.solver = SolverType::Classic;
        auto r = SolverOrchestrator::solve(polygons, sheets, config,
            [](const std::string& msg) { std::cout << "  " << msg << std::endl; });
        results.push_back({"Classic", r});
        std::cout << "  -> fitness=" << std::setprecision(2) << r.fitness
                  << "  placed=" << r.partsPlaced << "/" << r.totalParts
                  << "  " << std::setprecision(0) << r.elapsedMs << "ms" << std::endl << std::endl;

        GenerateSolverSvg(sheets, polygons, r, "solver_classic.svg");
    }

    // --- Compacted BLF ---
    if (false) {
        std::cout << "Running Compacted BLF..." << std::endl;
        config.solver = SolverType::CompactedBLF;
        auto r = SolverOrchestrator::solve(polygons, sheets, config,
            [](const std::string& msg) { std::cout << "  " << msg << std::endl; });
        results.push_back({"CompactedBLF", r});
        std::cout << "  -> fitness=" << std::setprecision(2) << r.fitness
                  << "  placed=" << r.partsPlaced << "/" << r.totalParts
                  << "  " << std::setprecision(0) << r.elapsedMs << "ms" << std::endl << std::endl;

        GenerateSolverSvg(sheets, polygons, r, "solver_compacted.svg");
    }

    // --- Jostle ---
    if (false) {
        std::cout << "Running Jostle Heuristic..." << std::endl;
        config.solver = SolverType::Jostle;
        auto r = SolverOrchestrator::solve(polygons, sheets, config,
            [](const std::string& msg) { std::cout << "  " << msg << std::endl; });
        results.push_back({"Jostle", r});
        std::cout << "  -> fitness=" << std::setprecision(2) << r.fitness
                  << "  placed=" << r.partsPlaced << "/" << r.totalParts
                  << "  " << std::setprecision(0) << r.elapsedMs << "ms" << std::endl << std::endl;

        GenerateSolverSvg(sheets, polygons, r, "solver_jostle.svg");
    }

    // --- Overlap Resolution ---
    {
        std::cout << "Running Overlap Resolution..." << std::endl;
        config.solver = SolverType::OverlapResolution;
        config.timeBudgetSeconds = 60;
        auto r = SolverOrchestrator::solve(polygons, sheets, config,
            [](const std::string& msg) { std::cout << "  " << msg << std::endl; });
        results.push_back({"OverlapRes", r});
        std::cout << "  -> fitness=" << std::setprecision(2) << r.fitness
                  << "  placed=" << r.partsPlaced << "/" << r.totalParts
                  << "  " << std::setprecision(0) << r.elapsedMs << "ms" << std::endl << std::endl;

        GenerateSolverSvg(sheets, polygons, r, "solver_overlap.svg");
    }

    // --- Smart BLF ---
    if (false) {
        std::cout << "Running Smart BLF..." << std::endl;
        config.solver = SolverType::SmartBLF;
        config.timeBudgetSeconds = 30;
        auto r = SolverOrchestrator::solve(polygons, sheets, config,
            [](const std::string& msg) { std::cout << "  " << msg << std::endl; });
        results.push_back({"SmartBLF", r});
        std::cout << "  -> fitness=" << std::setprecision(2) << r.fitness
                  << "  placed=" << r.partsPlaced << "/" << r.totalParts
                  << "  sheets=" << r.sheetsUsed
                  << "  " << std::setprecision(0) << r.elapsedMs << "ms" << std::endl << std::endl;

        GenerateSolverSvg(sheets, polygons, r, "solver_smartblf.svg");
    }

    // ========================================================================
    // Comparison table
    // ========================================================================

    std::cout << std::endl;
    std::cout << "===== SOLVER COMPARISON =====" << std::endl;
    std::cout << std::left
              << std::setw(16) << "Solver"
              << std::setw(12) << "Fitness"
              << std::setw(10) << "Placed"
              << std::setw(12) << "Util %"
              << std::setw(10) << "Sheets"
              << std::setw(12) << "Time (ms)"
              << std::endl;
    std::cout << std::string(72, '-') << std::endl;

    for (auto& rr : results) {
        std::cout << std::left
                  << std::setw(16) << rr.name
                  << std::setw(12) << std::fixed << std::setprecision(2) << rr.result.fitness
                  << std::setw(10) << (std::to_string(rr.result.partsPlaced) + "/" + std::to_string(rr.result.totalParts))
                  << std::setw(12) << std::setprecision(1) << (rr.result.materialUtilization * 100)
                  << std::setw(10) << rr.result.sheetsUsed
                  << std::setw(12) << std::setprecision(0) << rr.result.elapsedMs
                  << std::endl;
    }
    std::cout << std::endl;

    // Find best
    double bestFitness = 1e9;
    std::string bestName;
    for (auto& rr : results) {
        if (rr.result.fitness < bestFitness) {
            bestFitness = rr.result.fitness;
            bestName = rr.name;
        }
    }
    std::cout << "Best solver: " << bestName << " (fitness=" << std::setprecision(2) << bestFitness << ")" << std::endl;
    std::cout << std::endl;
    std::cout << "SVG files: solver_classic.svg, solver_compacted.svg, solver_jostle.svg, solver_overlap.svg, solver_smartblf.svg" << std::endl;

    return 0;
}
