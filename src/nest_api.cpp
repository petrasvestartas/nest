#include "nest_api.h"
#include "NestingContext.h"
#include "NfpWorker.h"
#include "NestConfig.h"

using namespace nest;

// ---------------------------------------------------------------------------
// Context lifecycle
// ---------------------------------------------------------------------------

void* nest_create_context() {
    return new NestingContext();
}

void nest_destroy_context(void* ctx) {
    delete static_cast<NestingContext*>(ctx);
}

// ---------------------------------------------------------------------------
// Geometry input
// ---------------------------------------------------------------------------

void nest_add_sheet(void* ctx, const double* pts, int count) {
    auto* context = static_cast<NestingContext*>(ctx);

    // Skip duplicate closing point if present
    int n = count;
    if (n > 1 &&
        std::fabs(pts[0] - pts[(n-1)*2]) < 1e-10 &&
        std::fabs(pts[1] - pts[(n-1)*2+1]) < 1e-10)
        n--;

    auto nfp = std::make_shared<NFP>();
    nfp->source = static_cast<int>(context->Sheets.size()); // unique source per sheet
    for (int i = 0; i < n; i++)
        nfp->AddPoint(Point(pts[i*2], pts[i*2+1]));

    context->Sheets.push_back(nfp);
}

void nest_add_sheet_hole(void* ctx, const double* pts, int count) {
    auto* context = static_cast<NestingContext*>(ctx);
    if (context->Sheets.empty()) return;

    int n = count;
    if (n > 1 &&
        std::fabs(pts[0] - pts[(n-1)*2]) < 1e-10 &&
        std::fabs(pts[1] - pts[(n-1)*2+1]) < 1e-10)
        n--;

    auto hole = std::make_shared<NFP>();
    for (int i = 0; i < n; i++)
        hole->AddPoint(Point(pts[i*2], pts[i*2+1]));
    context->Sheets.back()->children.push_back(hole);
}

void nest_add_part(void* ctx, const double* pts, int count, int source_id) {
    auto* context = static_cast<NestingContext*>(ctx);

    // Skip duplicate closing point if present
    int n = count;
    if (n > 1 &&
        std::fabs(pts[0] - pts[(n-1)*2]) < 1e-10 &&
        std::fabs(pts[1] - pts[(n-1)*2+1]) < 1e-10)
        n--;

    // First-point-center: subtract first point from all points
    double ox = pts[0];
    double oy = pts[1];

    auto nfp = std::make_shared<NFP>();
    nfp->source = source_id;
    for (int i = 0; i < n; i++)
        nfp->AddPoint(Point(pts[i*2] - ox, pts[i*2+1] - oy));

    context->Polygons.push_back(nfp);
}

void nest_add_part_hole(void* ctx, const double* pts, int count) {
    auto* context = static_cast<NestingContext*>(ctx);
    if (context->Polygons.empty()) return;

    // The last part was first-point centered; apply the same offset to the hole.
    auto& parent = context->Polygons.back();
    // Recover the part origin: parent's first point is always (0,0) after centering,
    // so the original first-point was pts[0],pts[1] of the PART call.
    // We can't recover it here, so hole pts must already be in the part's local space.
    int n = count;
    if (n > 1 &&
        std::fabs(pts[0] - pts[(n-1)*2]) < 1e-10 &&
        std::fabs(pts[1] - pts[(n-1)*2+1]) < 1e-10)
        n--;

    auto hole = std::make_shared<NFP>();
    for (int i = 0; i < n; i++)
        hole->AddPoint(Point(pts[i*2], pts[i*2+1]));
    parent->children.push_back(hole);
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

void nest_set_rotations(void* ctx, int n) {
    static_cast<NestingContext*>(ctx)->config.rotations = n;
}

void nest_set_spacing(void* ctx, double spacing) {
    static_cast<NestingContext*>(ctx)->config.spacing = spacing;
}

void nest_set_placement_type(void* ctx, int type) {
    auto* context = static_cast<NestingContext*>(ctx);
    if (type == 0)      context->config.placementType = PlacementTypeEnum::box;
    else if (type == 1) context->config.placementType = PlacementTypeEnum::gravity;
    else                context->config.placementType = PlacementTypeEnum::squeeze;
}

void nest_set_curve_tolerance(void* ctx, double tol) {
    static_cast<NestingContext*>(ctx)->config.curveTolerance = tol;
}

void nest_set_mutation_rate(void* ctx, int rate) {
    static_cast<NestingContext*>(ctx)->config.mutationRate = rate;
}

void nest_set_population_size(void* ctx, int size) {
    static_cast<NestingContext*>(ctx)->config.populationSize = size;
}

void nest_set_seed(void* ctx, int seed) {
    static_cast<NestingContext*>(ctx)->config.seed = seed;
}

void nest_set_use_parallel(void* ctx, int use) {
    (void)ctx; // NfpWorker::UseParallel is static
    NfpWorker::UseParallel = (use != 0);
}

void nest_set_explore_concave(void* ctx, int use) {
    static_cast<NestingContext*>(ctx)->config.exploreConcave = (use != 0);
}

void nest_set_edge_samples(void* ctx, int n) {
    static_cast<NestingContext*>(ctx)->config.edgeSamples = n;
}

void nest_set_compaction_passes(void* ctx, int n) {
    static_cast<NestingContext*>(ctx)->config.compactionPasses = n;
}

void nest_set_gravity_weight(void* ctx, double weight) {
    static_cast<NestingContext*>(ctx)->config.gravityWeight = weight;
}

// ---------------------------------------------------------------------------
// Execution
// ---------------------------------------------------------------------------

void nest_start(void* ctx) {
    auto* context = static_cast<NestingContext*>(ctx);
    context->ReorderSheets(); // lay sheets side-by-side before nesting
    context->StartNest();
}

void nest_iterate(void* ctx, int max_iterations) {
    static_cast<NestingContext*>(ctx)->NestIterate(max_iterations);
}

// ---------------------------------------------------------------------------
// Results
// ---------------------------------------------------------------------------

int nest_get_iterations(void* ctx) {
    return static_cast<NestingContext*>(ctx)->Iterations;
}

double nest_get_material_utilization(void* ctx) {
    return static_cast<NestingContext*>(ctx)->MaterialUtilization;
}

int nest_get_placed_count(void* ctx) {
    return static_cast<NestingContext*>(ctx)->PlacedPartsCount;
}

int nest_get_total_count(void* ctx) {
    return static_cast<int>(static_cast<NestingContext*>(ctx)->Polygons.size());
}

int nest_get_sheets_not_used(void* ctx) {
    return static_cast<NestingContext*>(ctx)->SheetsNotUsed;
}

int nest_get_part_count(void* ctx) {
    return static_cast<int>(static_cast<NestingContext*>(ctx)->Polygons.size());
}

int nest_get_sheet_count(void* ctx) {
    return static_cast<int>(static_cast<NestingContext*>(ctx)->Sheets.size());
}

double nest_get_sheet_x(void* ctx, int sheet_idx) {
    auto* context = static_cast<NestingContext*>(ctx);
    if (sheet_idx < 0 || sheet_idx >= static_cast<int>(context->Sheets.size())) return 0.0;
    return context->Sheets[sheet_idx]->x;
}

double nest_get_sheet_y(void* ctx, int sheet_idx) {
    auto* context = static_cast<NestingContext*>(ctx);
    if (sheet_idx < 0 || sheet_idx >= static_cast<int>(context->Sheets.size())) return 0.0;
    return context->Sheets[sheet_idx]->y;
}

void nest_get_part_result(void* ctx, int idx,
    double* out_x, double* out_y, float* out_rotation,
    int* out_source, int* out_fitted, int* out_sheet_id)
{
    auto* context = static_cast<NestingContext*>(ctx);
    if (idx < 0 || idx >= static_cast<int>(context->Polygons.size())) {
        *out_x = 0; *out_y = 0; *out_rotation = 0;
        *out_source = -1; *out_fitted = 0; *out_sheet_id = -1;
        return;
    }

    auto& poly = context->Polygons[idx];
    *out_x        = poly->x;
    *out_y        = poly->y;
    *out_rotation = poly->Rotation;
    *out_source   = poly->source.has_value() ? poly->source.value() : -1;
    *out_fitted   = poly->fitted() ? 1 : 0;
    *out_sheet_id = (poly->sheet != nullptr) ? poly->sheet->Id : -1;
}
