#include "stdafx.h"
#include "nfp.h"
#include "shapes.h"

int main(int argc, char **argv)
{

    
    printf("    Create data-set.\n");
    std::vector<Clipper2Lib::Paths64> patterns;
    for(int i = 0; i <15; i++){

        patterns.emplace_back(nest::pattern_1());
        // if (i % 2 == 0){
        //     patterns.emplace_back(nest::pattern_0());
        // } else {
        //     patterns.emplace_back(nest::pattern_1());
        // }
    }
    std::vector<Clipper2Lib::Paths64> paths;
    paths.emplace_back(nest::paths_0());
    std::vector<Clipper2Lib::Paths64> placed_paths;
    std::vector<Clipper2Lib::Paths64> results_clipper;

    printf("    Run the non-fit-polygon algorithm.\n");
    nest::nfps(patterns, paths, placed_paths, results_clipper);

    printf("    Write the nesting results to the SQL database.\n");
    write(patterns, paths, placed_paths, results_clipper, 10000,  "/Users/petras/brg/2_code/database_viewer/cmake/src/viewer/database/database_viewer.db");

    printf("    Nest Ends!\n");
    return 0;

}