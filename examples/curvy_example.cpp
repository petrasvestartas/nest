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
#include <vector>
#include <mutex>
#include <algorithm>

using namespace nest;

// ============================================================================
// Helper: create NFP from raw {x,y} array, normalized to near-origin
// ============================================================================

static std::shared_ptr<NFP> makePolygon(const std::vector<std::pair<double,double>>& pts, int source) {
    auto poly = std::make_shared<NFP>();
    poly->source = source;
    double minX = 1e30, minY = 1e30;
    for (auto& p : pts) {
        if (p.first < minX) minX = p.first;
        if (p.second < minY) minY = p.second;
    }
    for (auto& p : pts)
        poly->AddPoint(Point(p.first - minX, p.second - minY));
    return poly;
}

// ============================================================================
// Polygon data: 5 unique shapes from real production dataset
// Each built by concatenating curve segments (shared endpoints removed)
// ============================================================================

static std::vector<std::pair<double,double>> archA_data() {
    return {
        // Seg1: outer curve (64 pts)
        {6279.178492,-1236.292576},{6274.715632,-1199.971948},{6268.599022,-1163.89302},{6260.844367,-1128.130335},
        {6251.467374,-1092.758438},{6239.496411,-1054.959638},{6225.671883,-1017.798614},{6210.027732,-981.366257},
        {6192.603221,-945.751119},{6173.440215,-911.040508},{6152.585674,-877.319092},{6130.089358,-844.669926},
        {6106.007155,-813.172252},{6082.42434,-785.190958},{6057.590014,-758.314229},{6031.55543,-732.598353},
        {6004.375171,-708.096571},{5976.105413,-684.860405},{5946.805409,-662.937547},{5916.535828,-642.373932},
        {5885.359929,-623.212278},{5850.632452,-604.079937},{5815.002114,-586.686537},{5778.555669,-571.074848},
        {5741.3824,-557.283371},{5703.572786,-545.347386},{5665.219566,-535.293535},{5626.416361,-527.14574},
        {5587.258047,-520.925662},{5550.88523,-516.911527},{5514.367114,-514.560258},{5477.779795,-513.874736},
        {5441.199368,-514.85784},{5404.701895,-517.510455},{5368.36331,-521.825481},{5332.259357,-527.79312},
        {5296.465151,-535.400763},{5258.117978,-545.477753},{5220.315631,-557.437406},{5183.15071,-571.251071},
        {5146.713483,-586.883749},{5111.093102,-604.297636},{5076.376568,-623.450054},{5042.649428,-644.295514},
        {5009.993549,-666.782145},{4980.874494,-688.944807},{4952.796432,-712.412344},{4925.818165,-737.136366},
        {4899.995514,-763.065},{4875.382682,-790.144694},{4852.030362,-818.31865},{4829.987534,-847.528569},
        {4809.299809,-877.71337},{4788.464567,-911.446793},{4769.321366,-946.168419},{4751.917527,-981.793755},
        {4736.295267,-1018.235479},{4722.493813,-1055.404927},{4710.545501,-1093.210907},{4700.479793,-1131.56119},
        {4692.321524,-1170.361951},{4686.503365,-1206.490171},{4682.338317,-1242.846126},{4679.000369,-1315.938384},
        // Seg2: bottom straight (skip first, 5 pts)
        {4726.620806,-1315.938384},{4774.241244,-1315.938384},{4821.861682,-1315.938384},{4869.48212,-1315.938384},
        {4964.722995,-1315.938384},
        // Seg3: inner curve (skip first, 42 pts)
        {4966.871862,-1268.883855},{4973.298741,-1222.220791},{4978.550782,-1197.242065},{4985.030772,-1172.553342},
        {4992.722725,-1148.215024},{5001.607674,-1124.286486},{5011.664799,-1100.826435},{5022.868852,-1077.891948},
        {5035.19265,-1055.539242},{5048.605733,-1033.822711},{5106.992816,-960.015646},{5177.806139,-898.03173},
        {5198.828981,-883.555534},{5220.541468,-870.135874},{5242.890896,-857.806141},{5265.822192,-846.595621},
        {5289.279349,-836.531787},{5313.204973,-827.638978},{5337.540951,-819.939724},{5362.227672,-813.452471},
        {5455.403072,-800.227574},{5502.506198,-800.036},{5549.431119,-804.13385},{5574.640028,-808.138143},
        {5599.620328,-813.383441},{5624.310941,-819.855798},{5648.651598,-827.539815},{5672.582596,-836.41834},
        {5696.045687,-846.468662},{5718.983393,-857.665993},{5741.339866,-869.982801},{5817.958188,-924.628676},
        {5883.385707,-992.273141},{5898.889082,-1012.550366},{5913.371513,-1033.568887},{5926.79702,-1055.277689},
        {5939.133568,-1077.623304},{5950.350928,-1100.551225},{5960.422148,-1124.005246},{5969.32195,-1147.928361},
        {5977.028485,-1172.262056},{5994.868043,-1264.664883},
        // Seg4: top straight (skip first and last, 4 pts)
        {6042.253118,-1259.936165},{6089.638192,-1255.207447},{6137.023267,-1250.47873},{6184.408342,-1245.750012},
    };
}

static std::vector<std::pair<double,double>> archB_data() {
    return {
        // Seg1: outer curve (64 pts)
        {4568.304928,-1522.405068},{4578.040446,-1487.13056},{4585.945753,-1451.401335},{4591.999369,-1415.312196},
        {4596.185937,-1378.959143},{4598.600858,-1339.385131},{4598.803574,-1299.738025},{4596.792246,-1260.141451},
        {4592.574327,-1220.718821},{4586.161857,-1181.593189},{4577.575927,-1142.886391},{4566.842156,-1104.71939},
        {4553.995211,-1067.210858},{4540.296522,-1033.27831},{4524.868055,-1000.096469},{4507.750114,-967.753806},
        {4488.988881,-936.335861},{4468.633442,-905.926537},{4446.738595,-876.606116},{4423.361855,-848.452961},
        {4398.5659,-821.54139},{4370.173732,-793.867953},{4340.2815,-767.822064},{4308.981605,-743.485867},
        {4276.372372,-720.934316},{4242.554855,-700.238654},{4207.635098,-681.462474},{4171.721435,-664.665372},
        {4134.926376,-649.898598},{4100.28315,-638.112218},{4065.078765,-628.126057},{4029.406539,-619.967374},
        {3993.361415,-613.657055},{3957.038986,-609.212725},{3920.535838,-606.645315},{3883.948876,-605.962445},
        {3847.375313,-607.165184},{3807.876211,-610.593772},{3768.629997,-616.221258},{3729.759225,-624.031313},
        {3691.384869,-633.998425},{3653.626877,-646.092593},{3616.602662,-660.275042},{3580.428105,-676.502652},
        {3545.215496,-694.723687},{3513.665229,-713.261554},{3483.111786,-733.400056},{3453.636752,-755.086324},
        {3425.317951,-778.2621},{3398.231158,-802.8665},{3372.447728,-828.83352},{3348.036716,-856.09475},
        {3325.062421,-884.577269},{3301.863174,-916.728927},{3280.494298,-950.125109},{3261.023458,-984.662342},
        {3243.51035,-1020.232352},{3228.010594,-1056.724711},{3214.571488,-1094.025177},{3203.236045,-1132.01783},
        {3194.038451,-1170.583855},{3187.472591,-1206.583318},{3182.769854,-1242.873214},{3179.000369,-1315.938384},
        // Seg2: bottom (skip first, 5 pts)
        {3228.771881,-1315.938384},{3278.543393,-1315.938384},{3328.314904,-1315.938384},{3378.086416,-1315.938384},
        {3477.62944,-1315.938384},
        // Seg3: inner curve (skip first, 16 pts)
        {3486.342386,-1231.72112},{3515.006012,-1144.608648},{3562.256637,-1066.011167},{3620.343898,-1004.412999},
        {3689.811587,-956.011504},{3774.500965,-920.827767},{3864.880594,-905.2808},{3949.463786,-909.042139},
        {4031.485405,-930.040202},{4113.438096,-971.197735},{4184.236059,-1029.488677},{4236.626419,-1095.999686},
        {4274.290572,-1171.827623},{4296.643007,-1260.768923},{4298.735545,-1352.452113},{4282.581509,-1435.563547},
        // Seg4: top (skip first and last, 4 pts)
        {4330.202079,-1450.037134},{4377.822649,-1464.51072},{4425.443219,-1478.984307},{4473.063788,-1493.457894},
    };
}

static std::vector<std::pair<double,double>> archC_data() {
    return {
        // Seg1: outer curve (64 pts)
        {2791.167483,-1729.815439},{2814.074863,-1701.711259},{2835.335996,-1672.34193},{2854.880638,-1641.803406},
        {2872.645488,-1610.196382},{2890.101116,-1574.226878},{2905.252328,-1537.227628},{2918.037048,-1499.345514},
        {2928.405934,-1460.732164},{2936.316061,-1421.541157},{2941.737456,-1381.92913},{2944.646776,-1342.053835},
        {2945.034251,-1302.074421},{2943.202324,-1265.863402},{2939.300938,-1229.816577},{2933.3419,-1194.052296},
        {2925.345821,-1158.687688},{2915.33775,-1123.839002},{2903.351675,-1089.620186},{2889.42589,-1056.143823},
        {2873.60698,-1023.51935},{2854.029143,-988.659452},{2832.290733,-955.104349},{2808.476917,-922.988847},
        {2782.684054,-892.439988},{2755.013778,-863.580659},{2725.577616,-836.524853},{2694.491799,-811.381758},
        {2661.881443,-788.250212},{2631.099277,-769.091609},{2599.271058,-751.726158},{2566.500714,-736.211697},
        {2532.896194,-722.598137},{2498.567257,-710.931242},{2463.626871,-701.248124},{2428.189342,-693.581577},
        {2392.371125,-687.955792},{2352.572209,-684.141279},{2312.611921,-682.8456},{2272.649562,-684.075738},
        {2232.844448,-687.825057},{2193.355487,-694.080217},{2154.339934,-702.814678},{2115.953777,-713.995297},
        {2078.349579,-727.575725},{2045.051213,-741.921898},{2012.628494,-758.150319},{1981.188222,-776.208648},
        {1950.832983,-796.036716},{1921.662993,-817.570503},{1893.773188,-840.738414},{1867.255724,-865.465294},
        {1842.196922,-891.669314},{1816.354062,-922.175882},{1792.487607,-954.252285},{1770.694185,-987.771686},
        {1751.059271,-1022.599472},{1733.662595,-1058.597536},{1718.572075,-1095.621587},{1705.849487,-1133.524611},
        {1695.543872,-1172.154884},{1688.324161,-1207.686133},{1683.149794,-1243.572351},{1679.000369,-1315.938384},
        // Seg2: bottom (skip first, 6 pts)
        {1724.221281,-1315.938384},{1769.442192,-1315.938384},{1814.663104,-1315.938384},{1859.884016,-1315.938384},
        {1905.104928,-1315.938384},{1995.546752,-1315.938384},
        // Seg3: inner curve (skip first, 16 pts)
        {2003.818503,-1244.046634},{2031.576202,-1169.268928},{2077.145028,-1103.803849},{2131.463058,-1055.98755},
        {2195.221356,-1021.757054},{2272.468791,-1001.88172},{2352.232129,-1001.947088},{2422.494664,-1019.26826},
        {2486.987288,-1052.094298},{2547.388594,-1104.189186},{2592.850057,-1169.728867},{2618.719477,-1237.313036},
        {2628.563692,-1309.006402},{2620.249534,-1388.335274},{2592.369311,-1463.067383},{2551.630308,-1522.876911},
        // Seg4: top (skip first and last, 5 pts)
        {2585.849905,-1552.439558},{2620.069501,-1582.002205},{2654.289097,-1611.564852},{2688.508694,-1641.127498},
        {2722.72829,-1670.690145},
    };
}

static std::vector<std::pair<double,double>> archD_data() {
    return {
        // Seg1: outer curve (57 pts)
        {1131.527459,-1798},{1160.069469,-1775.660978},{1187.206587,-1751.634779},{1212.838448,-1726.008897},
        {1236.871972,-1698.878319},{1261.177825,-1667.660847},{1283.367836,-1634.905614},{1303.750854,-1599.999972},
        {1321.731492,-1563.798147},{1350.164261,-1488.169983},{1367.876613,-1411.094482},{1372.554653,-1375.152902},
        {1375.048345,-1338.994072},{1375.347071,-1302.750599},{1373.451108,-1266.555511},{1369.366113,-1230.541726},
        {1363.108361,-1194.841304},{1354.699461,-1159.585528},{1344.171733,-1124.903435},{1314.019644,-1051.792471},
        {1273.53159,-981.872816},{1250.102545,-949.461604},{1224.561249,-918.687486},{1197.020116,-889.689403},
        {1167.603797,-862.595406},{1139.434014,-839.788734},{1109.93516,-818.729326},{1079.214838,-799.495477},
        {1047.386505,-782.156632},{1014.566301,-766.777637},{980.875261,-753.413761},{946.436553,-742.11533},
        {911.377012,-732.922437},{833.302426,-720.324139},{792.954003,-717.898776},{752.533925,-718.20249},
        {712.226761,-721.236459},{672.216266,-726.984145},{595.448014,-745.984044},{561.266975,-758.039619},
        {527.878442,-772.14228},{495.405502,-788.241591},{463.966765,-806.277258},{433.678307,-826.184233},
        {404.650525,-847.888318},{376.990749,-871.310926},{350.799729,-896.365066},{299.181907,-956.28143},
        {275.986973,-989.385304},{255.079457,-1023.979329},{236.556203,-1059.906581},{220.500376,-1097.002236},
        {207.247528,-1134.280421},{196.486785,-1172.352842},{188.857878,-1207.785592},{183.387931,-1243.615178},
        {179.000369,-1315.938384},
        // Seg2: bottom (skip first, 6 pts)
        {225.95087,-1315.938384},{272.901371,-1315.938384},{319.851872,-1315.938384},{366.802373,-1315.938384},
        {413.752875,-1315.938384},{507.653877,-1315.938384},
        // Seg3: inner curve (skip first, 18 pts)
        {515.533925,-1251.23324},{541.938027,-1184.370454},{572.996301,-1139.978223},{609.340477,-1105.017102},
        {664.871695,-1070.883037},{712.476877,-1054.241904},{766.110058,-1046.575702},{820.193022,-1049.788543},
        {869.009407,-1062.44325},{927.16874,-1091.876944},{978.823644,-1136.921918},{1010.546713,-1180.84158},
        {1032.724708,-1229.850721},{1045.919116,-1293.68457},{1043.407035,-1358.819365},{1029.594704,-1410.809554},
        {1005.324144,-1459.677231},{936.899056,-1533.173806},
        // Seg4: top (skip first and last, 5 pts)
        {964.703114,-1571.00612},{992.507171,-1608.838433},{1020.311229,-1646.670746},{1048.115287,-1684.50306},
        {1075.919344,-1722.335373},
    };
}

static std::vector<std::pair<double,double>> stadium_data() {
    // Concatenated in polygon order: seg1 → seg2 → seg3 → seg5 → seg4 → seg6
    return {
        // Seg1: left curve up (5 pts)
        {6717,-970},{6723.5,-930},{6730.0,-890},{6736.5,-850},{6749.5,-770},
        // Seg2: top straight (skip first, 11 pts)
        {6795.75,-770},{6842.0,-770},{6888.25,-770},{6934.5,-770},{6980.75,-770},
        {7027.0,-770},{7073.25,-770},{7119.5,-770},{7165.75,-770},{7212.0,-770},{7304.5,-770},
        // Seg3: right curve down (skip first, 4 pts)
        {7311.0,-810},{7317.5,-850},{7324.0,-890},{7337.0,-970},
        // Seg5: right side vertical (skip first, 3 pts)
        {7337.0,-1030},{7337.0,-1080},{7337.0,-1100},
        // Seg4: bottom straight (skip first, 12 pts)
        {7289.307692,-1100},{7241.615385,-1100},{7193.923077,-1100},{7146.230769,-1100},
        {7098.538462,-1100},{7050.846154,-1100},{7003.153846,-1100},{6955.461538,-1100},
        {6907.769231,-1100},{6860.076923,-1100},{6812.384615,-1100},{6717,-1100},
        // Seg6: left side vertical (skip first and last, 2 pts)
        {6717,-1080},{6717,-1030},
    };
}

// ============================================================================
// Helper: build sheets for solver (positioned at origin, solver handles layout)
// ============================================================================

static std::vector<std::shared_ptr<NFP>> makeSheets(int count) {
    std::vector<std::shared_ptr<NFP>> sheets;
    for (int i = 0; i < count; i++) {
        auto s = std::make_shared<NFP>();
        s->source = i;
        s->AddPoint(Point(0, 0));
        s->AddPoint(Point(2440, 0));
        s->AddPoint(Point(2440, 1220));
        s->AddPoint(Point(0, 1220));
        sheets.push_back(s);
    }
    return sheets;
}

// ============================================================================
// Main — Multi-solver packing search using research paper heuristics
//
// Research heuristics applied (from nesting-research.md):
// 1. Post-placement compaction: slide parts left/down/diagonal (Compaction.cpp)
// 2. Jostle breathing: 4-dir oscillation (Abeysooriya 2018, JostleHeuristic.cpp)
// 3. Pair pre-matching: find interlocking pairs (PairPreMatching.cpp)
// 4. Multi-rotation evaluation: 16 angles for curve interlocking
// 5. SmartBLF: combines all of the above (PairMatching+BLF+Compaction+Jostle)
// ============================================================================

int main(int argc, char* argv[]) {
    std::cout << "OpenNest2 C++ Production Nesting — Research Heuristic Solver Comparison" << std::endl;
    std::cout << "=======================================================================" << std::endl;
    std::cout << "4 arch types + stadium panels on 2440x1220 sheets" << std::endl;
    std::cout << "Using: Compaction + Jostle breathing + Pair pre-matching + SmartBLF\n" << std::endl;

    double timeBudget = 60.0;  // seconds per solver
    if (argc > 1) {
        double userTime = std::atof(argv[1]);
        if (userTime > 0) timeBudget = userTime;
    }

    // === Build polygons ===
    std::vector<std::shared_ptr<NFP>> polygons;

    struct PartType {
        const char* name;
        int source;
        int count;
        std::function<std::vector<std::pair<double,double>>()> data;
    };

    std::vector<PartType> types = {
        {"Arch A", 0, 3, archA_data},
        {"Arch B", 1, 1, archB_data},
        {"Arch C", 2, 1, archC_data},
        {"Arch D", 3, 3, archD_data},
        {"Stadium", 4, 8, stadium_data},
    };

    for (auto& t : types) {
        for (int c = 0; c < t.count; c++) {
            auto pts = t.data();
            polygons.push_back(makePolygon(pts, t.source));
        }
    }

    // --- Print summary ---
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "Parts:  " << polygons.size() << std::endl;
    for (auto& t : types) {
        auto pts = t.data();
        auto sample = makePolygon(pts, t.source);
        auto bounds = GeometryUtil::getPolygonBounds(*sample);
        double area = std::fabs(GeometryUtil::polygonArea(*sample));
        std::cout << "  P" << t.source << ": " << t.name
                  << " x" << t.count
                  << " (" << sample->length() << " pts"
                  << ", " << std::setprecision(0) << bounds.width << "x" << bounds.height
                  << ", area=" << area << ")" << std::endl;
        std::cout << std::setprecision(1);
    }

    double totalPartArea = 0;
    for (auto& poly : polygons)
        totalPartArea += std::fabs(GeometryUtil::polygonArea(*poly));
    double sheetArea = 2440.0 * 1220.0;
    std::cout << "Total part area: " << std::setprecision(0) << totalPartArea
              << " (" << std::setprecision(1) << (totalPartArea / sheetArea) << " sheets equivalent)" << std::endl;
    std::cout << "Time budget: " << timeBudget << "s per solver" << std::endl;
    std::cout << std::endl;

    NfpWorker::UseParallel = true;

    // === Configure GA — aggressive settings for curved shapes ===
    NestConfig baseConfig;
    baseConfig.rotations = 16;              // 22.5° steps — curve interlocking
    baseConfig.spacing = 5;
    baseConfig.clipperScale = 1e7;
    baseConfig.exploreConcave = true;
    baseConfig.clipByRects = true;
    baseConfig.populationSize = 250;        // large GA population
    baseConfig.mutationRate = 25;
    baseConfig.edgeSamples = 4;             // dense NFP edge sampling
    baseConfig.compactionPasses = 4;        // BLF-internal compaction
    baseConfig.gravityWeight = 0.1;
    baseConfig.tryAllRotations = true;      // evaluate all 16 rotations per placement
    baseConfig.seed = 42;
    baseConfig.placementType = PlacementTypeEnum::gravity;

    int numSeeds = 6;
    int itersPerSeed = 100;

    // === Run multiple configurations and compare ===
    struct RunResult {
        std::string name;
        int sheetsUsed;
        int partsPlaced;
        int totalParts;
        double fitness;
        double utilization;
        double elapsedMs;
        // Per-part placement data
        struct PartPlace { int partIdx; int source; double x, y; float rotation; int sheet; bool placed; };
        std::vector<PartPlace> parts;
    };
    std::vector<RunResult> results;

    // Helper: convert NestingContext result to RunResult
    auto contextToResult = [&](const NestingContext& ctx, const std::string& name,
                               const std::vector<std::shared_ptr<NFP>>& polys,
                               int nSheets, double elapsed) -> RunResult {
        RunResult rr;
        rr.name = name;
        rr.totalParts = static_cast<int>(polys.size());
        rr.partsPlaced = 0;
        rr.sheetsUsed = 0;
        rr.fitness = ctx.HasCurrent() ? ctx.Current().fitness.value_or(1e9) : 1e9;
        rr.utilization = ctx.MaterialUtilization;
        rr.elapsedMs = elapsed;

        for (size_t i = 0; i < ctx.Polygons.size(); i++) {
            auto& p = ctx.Polygons[i];
            RunResult::PartPlace pp;
            pp.partIdx = static_cast<int>(i);
            pp.source = p->source.value_or(-1);
            pp.rotation = p->Rotation;
            pp.sheet = p->sheet ? p->sheet->Id : -1;
            pp.placed = p->fitted();

            // NestingContext stores absolute coords: poly->x = ssitem.x + sheet->x
            // Subtract sheet origin to get sheet-relative coordinates
            if (pp.placed && p->sheet) {
                pp.x = p->x - p->sheet->x;
                pp.y = p->y - p->sheet->y;
            } else {
                pp.x = p->x;
                pp.y = p->y;
            }

            if (pp.placed) {
                rr.partsPlaced++;
                rr.sheetsUsed = std::max(rr.sheetsUsed, pp.sheet + 1);
            }
            rr.parts.push_back(pp);
        }
        return rr;
    };

    // Run strategy with given config
    auto runStrategy = [&](const std::string& name, int nSheets, NestConfig cfg) {
        std::cout << "=== " << name << ": " << nSheets << " sheets, "
                  << numSeeds << " seeds x " << itersPerSeed << " iter ===" << std::endl;
        auto sheets = makeSheets(nSheets);
        auto start = std::chrono::high_resolution_clock::now();
        auto best = NestingContext::RunParallelSeeds(polygons, sheets, cfg, numSeeds, itersPerSeed,
            [](int seed, int iter, const NestingContext& ctx) {
                if (iter % 25 == 0)
                    std::cout << "  seed=" << seed << " iter=" << iter
                              << " placed=" << ctx.PlacedPartsCount << "/" << ctx.Polygons.size() << std::endl;
            });
        double elapsed = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - start).count();
        auto rr = contextToResult(best, name, polygons, nSheets, elapsed);
        std::cout << "  -> placed=" << rr.partsPlaced << "/" << rr.totalParts
                  << "  sheets=" << rr.sheetsUsed << "  fitness=" << std::setprecision(0) << rr.fitness
                  << "  " << rr.elapsedMs << "ms" << std::endl << std::endl;
        results.push_back(rr);
    };

    // Strategy 1: 3 sheets, gravity (tight spacing)
    {
        NestConfig cfg = baseConfig;
        cfg.spacing = 2;
        cfg.placementType = PlacementTypeEnum::gravity;
        runStrategy("3s-gravity-tight", 3, cfg);
    }

    // Strategy 2: 4 sheets, gravity, spacing=5 (baseline)
    {
        NestConfig cfg = baseConfig;
        cfg.placementType = PlacementTypeEnum::gravity;
        runStrategy("4s-gravity-s5", 4, cfg);
    }

    // Strategy 3: 4 sheets, gravity, tight spacing=2
    {
        NestConfig cfg = baseConfig;
        cfg.spacing = 2;
        cfg.placementType = PlacementTypeEnum::gravity;
        runStrategy("4s-gravity-s2", 4, cfg);
    }

    // Strategy 4: 4 sheets, squeeze, tight spacing=2
    {
        NestConfig cfg = baseConfig;
        cfg.spacing = 2;
        cfg.placementType = PlacementTypeEnum::squeeze;
        runStrategy("4s-squeeze-s2", 4, cfg);
    }

    // === Comparison table ===
    std::cout << "========================================" << std::endl;
    std::cout << "STRATEGY COMPARISON:" << std::endl;
    std::cout << std::left
              << std::setw(16) << "Strategy"
              << std::setw(10) << "Placed"
              << std::setw(10) << "Sheets"
              << std::setw(14) << "Fitness"
              << std::setw(12) << "Time(ms)"
              << std::endl;
    std::cout << std::string(62, '-') << std::endl;

    for (auto& rr : results) {
        std::cout << std::left
                  << std::setw(16) << rr.name
                  << std::setw(10) << (std::to_string(rr.partsPlaced) + "/" + std::to_string(rr.totalParts))
                  << std::setw(10) << rr.sheetsUsed
                  << std::setw(14) << std::fixed << std::setprecision(0) << rr.fitness
                  << std::setw(12) << rr.elapsedMs
                  << std::endl;
    }
    std::cout << std::endl;

    // === Pick the best result ===
    // Priority: (1) all 16 placed, (2) fewer sheets, (3) lower fitness
    int bestIdx = -1;
    for (int i = 0; i < static_cast<int>(results.size()); i++) {
        if (results[i].partsPlaced < results[i].totalParts) continue; // must place all
        if (bestIdx < 0) { bestIdx = i; continue; }
        auto& best = results[bestIdx];
        auto& cur = results[i];
        if (cur.sheetsUsed < best.sheetsUsed) { bestIdx = i; continue; }
        if (cur.sheetsUsed > best.sheetsUsed) continue;
        if (cur.fitness < best.fitness) { bestIdx = i; continue; }
    }
    // Fallback: pick most parts placed
    if (bestIdx < 0) {
        bestIdx = 0;
        for (int i = 1; i < static_cast<int>(results.size()); i++) {
            if (results[i].partsPlaced > results[bestIdx].partsPlaced) bestIdx = i;
        }
    }

    auto& winner = results[bestIdx];
    std::cout << "*** BEST: " << winner.name << " ***" << std::endl;
    std::cout << "  Placed: " << winner.partsPlaced << "/" << winner.totalParts << std::endl;
    std::cout << "  Sheets used: " << winner.sheetsUsed << std::endl;
    std::cout << "  Fitness: " << std::setprecision(0) << winner.fitness << std::endl;
    std::cout << std::endl;

    // --- Per-part results ---
    const char* partNames[] = {"ArchA", "ArchB", "ArchC", "ArchD", "Stadium"};
    for (auto& pr : winner.parts) {
        std::cout << "  P" << pr.partIdx << " " << partNames[((pr.source % 5) + 5) % 5]
                  << " sheet=" << pr.sheet << " x=" << std::setprecision(1) << pr.x
                  << " y=" << pr.y << " rot=" << pr.rotation
                  << (pr.placed ? "" : " NOT PLACED") << std::endl;
    }

    // --- Generate SVG ---
    // Build SVG from RunResult
    int numSheets = winner.sheetsUsed;
    const double sheetW = 2440, sheetH = 1220, gap = 50;
    std::vector<std::pair<double,double>> sheetOrigins;
    for (int i = 0; i < numSheets; i++)
        sheetOrigins.push_back({i * (sheetW + gap), 0});

    struct PlacedPoly { std::vector<std::pair<double,double>> pts; int source; };
    std::vector<PlacedPoly> placed;
    for (auto& pr : winner.parts) {
        if (!pr.placed) continue;
        auto& poly = polygons[pr.partIdx];
        double rad = pr.rotation * M_PI / 180.0;
        double cosA = std::cos(rad), sinA = std::sin(rad);
        double ox = sheetOrigins[pr.sheet].first;
        double oy = sheetOrigins[pr.sheet].second;
        PlacedPoly pp; pp.source = pr.source;
        for (int j = 0; j < poly->length(); j++) {
            double px = poly->Points[j].x, py = poly->Points[j].y;
            pp.pts.push_back({px*cosA - py*sinA + pr.x + ox, px*sinA + py*cosA + pr.y + oy});
        }
        placed.push_back(pp);
    }

    double minX=0, minY=0, maxX=0, maxY=sheetH;
    for (int i = 0; i < numSheets; i++) maxX = std::max(maxX, sheetOrigins[i].first + sheetW);
    for (auto& pp : placed) for (auto& pt : pp.pts) {
        minX = std::min(minX, pt.first); minY = std::min(minY, pt.second);
        maxX = std::max(maxX, pt.first); maxY = std::max(maxY, pt.second);
    }
    double margin=20, sceneW=maxX-minX, sceneH=maxY-minY;
    double scale=std::min(2000.0/sceneW, 800.0/sceneH);
    int imgW=static_cast<int>(sceneW*scale+2*margin), imgH=static_cast<int>(sceneH*scale+2*margin);
    auto tx=[&](double x){return (x-minX)*scale+margin;};
    auto ty=[&](double y){return (y-minY)*scale+margin;};
    const char* fills[]={"rgba(66,133,244,0.6)","rgba(234,67,53,0.6)","rgba(52,168,83,0.6)","rgba(251,188,4,0.6)","rgba(171,71,188,0.6)"};
    const char* strokes[]={"rgb(33,66,122)","rgb(117,33,26)","rgb(26,84,41)","rgb(125,94,2)","rgb(85,35,94)"};

    auto writeSvg = [&](const std::string& path) {
        std::ofstream f(path);
        f << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << imgW << "\" height=\"" << imgH << "\">\n";
        f << "<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n";
        for (int i = 0; i < numSheets; i++) {
            double sx = sheetOrigins[i].first, sy = sheetOrigins[i].second;
            f << "<polygon points=\"" << std::fixed << std::setprecision(2)
              << tx(sx) << "," << ty(sy) << " " << tx(sx+sheetW) << "," << ty(sy) << " "
              << tx(sx+sheetW) << "," << ty(sy+sheetH) << " " << tx(sx) << "," << ty(sy+sheetH)
              << "\" fill=\"#e8e8e8\" stroke=\"black\" stroke-width=\"2\"/>\n";
            f << "<text x=\"" << tx(sx+sheetW/2) << "\" y=\"" << ty(sy+sheetH+30)
              << "\" text-anchor=\"middle\" font-family=\"Arial\" font-size=\"11\" fill=\"#666\">Sheet " << i << "</text>\n";
        }
        for (auto& pp : placed) {
            int ci=((pp.source%5)+5)%5;
            f << "<polygon points=\"";
            for (size_t i=0;i<pp.pts.size();i++) { if(i)f<<" "; f<<std::fixed<<std::setprecision(2)<<tx(pp.pts[i].first)<<","<<ty(pp.pts[i].second); }
            f << "\" fill=\"" << fills[ci] << "\" stroke=\"" << strokes[ci] << "\" stroke-width=\"1\"/>\n";
            double cx=0,cy=0; for(auto&pt:pp.pts){cx+=pt.first;cy+=pt.second;} cx/=pp.pts.size(); cy/=pp.pts.size();
            f << "<text x=\""<<std::fixed<<std::setprecision(2)<<tx(cx)<<"\" y=\""<<ty(cy)
              <<"\" text-anchor=\"middle\" dominant-baseline=\"central\" font-family=\"Arial\" font-size=\"9\" fill=\"black\">P"<<pp.source<<"</text>\n";
        }
        for (int i = 0; i < numSheets; i++) {
            double sx = sheetOrigins[i].first, sy = sheetOrigins[i].second;
            f << "<polygon points=\"" << std::fixed << std::setprecision(2)
              << tx(sx) << "," << ty(sy) << " " << tx(sx+sheetW) << "," << ty(sy) << " "
              << tx(sx+sheetW) << "," << ty(sy+sheetH) << " " << tx(sx) << "," << ty(sy+sheetH)
              << "\" fill=\"none\" stroke=\"black\" stroke-width=\"2\"/>\n";
        }
        f << "<text x=\"10\" y=\"" << (imgH - 10)
          << "\" font-family=\"Arial\" font-size=\"12\" fill=\"#333\">"
          << winner.name << " | " << winner.partsPlaced << "/" << winner.totalParts << " placed | "
          << winner.sheetsUsed << " sheets | " << std::setprecision(0) << winner.fitness << " fitness | "
          << winner.elapsedMs << "ms</text>\n";
        f << "</svg>\n"; f.close();
    };

    writeSvg("curvy_result.svg");
    writeSvg("../../curvy_result.svg");
    std::cout << std::endl << "SVG written to: curvy_result.svg and ../../curvy_result.svg" << std::endl;

    return 0;
}
