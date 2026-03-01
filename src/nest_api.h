#pragma once
#ifdef _WIN32
  #ifdef OPENNEST_API_EXPORTS
    #define NEST_API __declspec(dllexport)
  #else
    #define NEST_API __declspec(dllimport)
  #endif
#else
  #define NEST_API __attribute__((visibility("default")))
#endif

extern "C" {
  /// Create a new NestingContext. Returns opaque handle.
  NEST_API void* nest_create_context();

  /// Destroy context and free all memory.
  NEST_API void  nest_destroy_context(void* ctx);

  // --- Geometry input (call before nest_start) ---

  /// Add a sheet boundary. pts = interleaved x,y pairs, count = number of points.
  /// Points are used as-is (not origin-centered).
  NEST_API void nest_add_sheet(void* ctx, const double* pts, int count);

  /// Add a hole to the most recently added sheet. pts = interleaved x,y pairs.
  NEST_API void nest_add_sheet_hole(void* ctx, const double* pts, int count);

  /// Add a part polygon. pts = interleaved x,y pairs, count = number of points.
  /// Points are first-point-centered (first point subtracted from all).
  NEST_API void nest_add_part(void* ctx, const double* pts, int count, int source_id);

  /// Add a hole to the most recently added part. pts = interleaved x,y pairs.
  /// Hole coordinates are in the same space as the parent part (before centering).
  NEST_API void nest_add_part_hole(void* ctx, const double* pts, int count);

  // --- Configuration ---
  NEST_API void nest_set_rotations        (void* ctx, int n);
  NEST_API void nest_set_spacing          (void* ctx, double spacing);
  NEST_API void nest_set_placement_type   (void* ctx, int type); // 0=box,1=gravity,2=squeeze
  NEST_API void nest_set_curve_tolerance  (void* ctx, double tol);
  NEST_API void nest_set_mutation_rate    (void* ctx, int rate);
  NEST_API void nest_set_population_size  (void* ctx, int size);
  NEST_API void nest_set_seed             (void* ctx, int seed);
  NEST_API void nest_set_use_parallel     (void* ctx, int use);  // 1=true, 0=false
  NEST_API void nest_set_explore_concave  (void* ctx, int use);  // 1=true, 0=false
  NEST_API void nest_set_edge_samples     (void* ctx, int n);
  NEST_API void nest_set_compaction_passes(void* ctx, int n);
  NEST_API void nest_set_gravity_weight   (void* ctx, double weight);

  // --- Execution ---
  /// Calls ReorderSheets() to lay sheets side-by-side, then StartNest().
  NEST_API void nest_start  (void* ctx);
  NEST_API void nest_iterate(void* ctx, int max_iterations);

  // --- Results (call after nest_iterate) ---
  NEST_API int    nest_get_iterations          (void* ctx);
  NEST_API double nest_get_material_utilization(void* ctx);
  NEST_API int    nest_get_placed_count        (void* ctx);
  NEST_API int    nest_get_total_count         (void* ctx);
  NEST_API int    nest_get_sheets_not_used     (void* ctx);
  NEST_API int    nest_get_part_count          (void* ctx);

  /// Number of sheets registered with the context.
  NEST_API int    nest_get_sheet_count(void* ctx);

  /// X offset of sheet at sheet_idx (set by ReorderSheets, called inside nest_start).
  NEST_API double nest_get_sheet_x(void* ctx, int sheet_idx);

  /// Y offset of sheet at sheet_idx (always 0 with the default layout).
  NEST_API double nest_get_sheet_y(void* ctx, int sheet_idx);

  /// Get result for polygon at idx (0-based index into Polygons array).
  /// out_x, out_y       - placement position
  /// out_rotation       - rotation in degrees
  /// out_source         - original source_id passed to nest_add_part
  /// out_fitted         - 1 if placed, 0 if not
  /// out_sheet_id       - index of the sheet it was placed on (0-based), -1 if not placed
  NEST_API void nest_get_part_result(void* ctx, int idx,
    double* out_x, double* out_y, float* out_rotation,
    int* out_source, int* out_fitted, int* out_sheet_id);
}
