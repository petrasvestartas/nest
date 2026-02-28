#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>

#include "NFP.h"
#include "GeometryUtil.h"
#include "SolverConfig.h"

namespace nest {

/// Fine rotation sampling for nesting.
/// Generates candidate rotation angles using principal axis analysis
/// and uniform sampling, instead of just 4 discrete rotations.
class ContinuousRotation {
public:
    /// Compute the principal axis angle of a polygon using second moment of area tensor.
    /// Returns the angle in degrees [0, 180) that aligns the polygon's longest axis horizontally.
    static double principalAxisAngle(const NFP& polygon);

    /// Generate candidate rotation angles for a polygon.
    /// Combines principal axis alignment with uniform sampling.
    /// @param polygon The polygon to analyze
    /// @param numSteps Total number of rotation steps (e.g., 36 = every 10 degrees)
    /// @param usePrincipalAxis If true, include principal axis and offsets
    /// @param rotationLimit Maximum rotation range in degrees (default 360)
    /// @return Vector of candidate angles in degrees
    static std::vector<float> generateCandidateAngles(
        const NFP& polygon,
        int numSteps = 36,
        bool usePrincipalAxis = true,
        float rotationLimit = 360.0f);

    /// Find the rotation angle that minimizes the bounding box area.
    /// Quick heuristic pre-filter to identify promising rotations.
    /// @param polygon The polygon to rotate
    /// @param candidates Candidate angles to test
    /// @return Best angle in degrees
    static float bestBoundingBoxRotation(
        const NFP& polygon,
        const std::vector<float>& candidates);

    /// Sort candidate angles by bounding box area (ascending).
    /// Returns the sorted angles.
    static std::vector<float> rankByBoundingBox(
        const NFP& polygon,
        const std::vector<float>& candidates);

    /// Compute bounding box area for a polygon at a given rotation.
    static double boundingBoxArea(const NFP& polygon, float angleDeg);

private:
    /// Normalize angle to [0, 360) range
    static float normalizeAngle(float angle);

    /// Remove duplicate angles (within tolerance)
    static std::vector<float> deduplicateAngles(
        const std::vector<float>& angles, float tolerance = 0.5f);
};

} // namespace nest
