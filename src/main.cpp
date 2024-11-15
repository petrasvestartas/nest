#include "stdafx.h"
#include "nfp.h"
#include "shapes.h"
#include "database_writer.h"

int main(int argc, char **argv)
{

    
    printf("    Create data-set.\n");
    std::vector<Clipper2Lib::Paths64> patterns;
    for(int i = 0; i <50; i++){

        if (i % 0 == 0){
            patterns.emplace_back(nest::pattern_1());
        } else {
            patterns.emplace_back(nest::pattern_2());
        }
    }
    std::vector<Clipper2Lib::Paths64> paths;
    paths.emplace_back(nest::paths_0());
    std::vector<Clipper2Lib::Paths64> placed_paths;
    std::vector<Clipper2Lib::Paths64> results_clipper;

    printf("    Run the non-fit-polygon algorithm.\n");
    nest::nfps(patterns, paths, placed_paths, results_clipper);

    printf("    Write the nesting results to the SQL database.\n");
    #ifdef _WIN32
    // Windows path
    write(patterns, paths, placed_paths, results_clipper, 10000, "C:/brg/2_code/database_viewer/cmake/src/viewer/database/database_viewer.db");
    #else
    // Unix path
    write(patterns, paths, placed_paths, results_clipper, 10000, "/Users/petras/brg/2_code/database_viewer/cmake/src/viewer/database/database_viewer.db");
    #endif

    printf("    Nest Ends!\n");
    return 0;

}