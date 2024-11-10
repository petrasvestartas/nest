#pragma once
#include "../stdafx.h"
// minkowski + holes
// what to do with multiple minkowski sum outlines or I just need to take one item?
// multiple minkovskies


namespace nest
{
    static std::vector<int> placed_id;
    static std::vector<std::array<float, 3>> placed_rotationtranslation;
    static std::vector<bool> placed_pattern_or_path; // there can be multiple sheets
    static float scale = 10000;

    inline void nfp(Clipper2Lib::Paths64 &pattern, Clipper2Lib::Paths64 &result, int temp, std::vector<Clipper2Lib::Paths64>& placed_paths) 
    {

        Clipper2Lib::Point64 start_point = pattern[0][0];
        for (int i = 0; i < pattern.size(); i++)
            for (int j = 0; j < pattern[i].size(); j++)
                pattern[i][j] = Clipper2Lib::Point64(pattern[i][j].x - start_point.x, pattern[i][j].y - start_point.y);

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Compute minkowski sum for two polygons with holes
        // Do not union them, do it once iterated through all patterns
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        for (int i = 0; i < placed_paths.size(); i++)
        {
            // std::cout << placed_paths[i].size() << " \n";
            for (int j = 0; j < placed_paths[i].size(); j++) //
            {

                Clipper2Lib::Paths64 mink_diff = Clipper2Lib::MinkowskiDiff(pattern[0], placed_paths[i][j], true); // take only boundary of the element
                size_t start = -1;
                size_t end = -1;

                if (placed_pattern_or_path[i])
                {
                    start = j != 0 ? 1 : 0;
                    end = j != 0 ? mink_diff.size() : 1;
                }
                else
                {
                    start = j == 0 ? 1 : 0;
                    end = j == 0 ? mink_diff.size() : 1;
                }

                for (int w = start; w < end; w++)
                    result.emplace_back(mink_diff[w]);
            }
        }

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Boolean union
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        result = Clipper2Lib::Union(result, Clipper2Lib::FillRule::Negative);

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Choose the bottom left point
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        start_point = result[0][0];
        for (int i = 0; i < result.size(); i++)
            for (int j = 0; j < result[i].size(); j++)
                if (result[i][j].x < start_point.x || result[i][j].y < start_point.y)
                    start_point = result[i][j];

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Place one pattern in a loop
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        for (int i = 0; i < pattern.size(); i++)
            for (int j = 0; j < pattern[i].size(); j++)
                pattern[i][j] = Clipper2Lib::Point64(pattern[i][j].x + start_point.x, pattern[i][j].y + start_point.y);

        for (int i = 0; i < pattern.size(); i++)
            pattern[i].emplace_back(pattern[i][0]);

        placed_paths.emplace_back(pattern);
        std::array<float, 3> rot_tran = {0, start_point.x / scale, start_point.y / scale};
        placed_rotationtranslation.emplace_back(rot_tran);
        placed_pattern_or_path.emplace_back(true); // there can be multiple sheets
    }

    inline void nfps(

        std::vector<Clipper2Lib::Paths64>& patterns,
        std::vector<Clipper2Lib::Paths64>& paths,
        std::vector<Clipper2Lib::Paths64>& placed_paths,
        std::vector<Clipper2Lib::Paths64>& results_clipper
        ) {
   
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Iteration 0
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        auto start = std::chrono::high_resolution_clock::now();
        Clipper2Lib::Paths64 result_clipper;

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Iteration 0 - Initialize the first sheet
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        placed_id.emplace_back(0);
        placed_paths.emplace_back(paths[0]);

        for (int i = 0; i < placed_paths.size(); i++)
            for (int j = 0; j < placed_paths[i].size(); j++) //
                placed_paths[i][j].emplace_back(placed_paths[i][j][0]);

        std::array<float, 3> rot_tran = {0, 0, 0};
        placed_rotationtranslation.emplace_back(rot_tran);
        placed_pattern_or_path.emplace_back(false); // there can be multiple sheets

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Iterations
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        nfp(patterns[0], result_clipper, 0, placed_paths);


        
        for (int i = 1; i < patterns.size(); i++)
        {
            result_clipper.clear(); // clear preview
            nfp(patterns[i], result_clipper, i, placed_paths); // run nesting method
            
        }
        results_clipper.emplace_back(result_clipper);

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "      Time taken by nfps method: " << duration.count() << " ms" << std::endl;

    }
}