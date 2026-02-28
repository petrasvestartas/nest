#include "ContinuousRotation.h"
#include <cmath>
#include <set>

namespace nest {

float ContinuousRotation::normalizeAngle(float angle) {
    angle = std::fmod(angle, 360.0f);
    if (angle < 0) angle += 360.0f;
    return angle;
}

std::vector<float> ContinuousRotation::deduplicateAngles(
    const std::vector<float>& angles, float tolerance)
{
    std::vector<float> result;
    for (float a : angles) {
        bool dup = false;
        for (float r : result) {
            float diff = std::fabs(normalizeAngle(a) - normalizeAngle(r));
            if (diff > 180.0f) diff = 360.0f - diff;
            if (diff < tolerance) {
                dup = true;
                break;
            }
        }
        if (!dup) {
            result.push_back(a);
        }
    }
    return result;
}

double ContinuousRotation::principalAxisAngle(const NFP& polygon) {
    if (polygon.length() < 3) return 0;

    // Compute centroid
    double cx = 0, cy = 0;
    double area = 0;
    int n = polygon.length();

    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        double cross = polygon[i].x * polygon[j].y - polygon[j].x * polygon[i].y;
        area += cross;
        cx += (polygon[i].x + polygon[j].x) * cross;
        cy += (polygon[i].y + polygon[j].y) * cross;
    }

    area /= 2.0;
    if (std::fabs(area) < 1e-10) return 0;

    cx /= (6.0 * area);
    cy /= (6.0 * area);

    // Compute second moment of area tensor (Ixx, Iyy, Ixy)
    double Ixx = 0, Iyy = 0, Ixy = 0;
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        double x0 = polygon[i].x - cx;
        double y0 = polygon[i].y - cy;
        double x1 = polygon[j].x - cx;
        double y1 = polygon[j].y - cy;
        double cross = x0 * y1 - x1 * y0;

        Ixx += (y0 * y0 + y0 * y1 + y1 * y1) * cross;
        Iyy += (x0 * x0 + x0 * x1 + x1 * x1) * cross;
        Ixy += (x0 * y1 + 2 * x0 * y0 + 2 * x1 * y1 + x1 * y0) * cross;
    }

    Ixx /= 12.0;
    Iyy /= 12.0;
    Ixy /= 24.0;

    // Principal axis angle: 0.5 * atan2(2*Ixy, Iyy - Ixx)
    double angle = 0.5 * std::atan2(2.0 * Ixy, Iyy - Ixx);
    return angle * 180.0 / M_PI; // Convert to degrees
}

std::vector<float> ContinuousRotation::generateCandidateAngles(
    const NFP& polygon,
    int numSteps,
    bool usePrincipalAxis,
    float rotationLimit)
{
    std::vector<float> candidates;

    // Always include 0 degrees
    candidates.push_back(0.0f);

    // Uniform sampling
    if (numSteps > 0) {
        float step = rotationLimit / static_cast<float>(numSteps);
        for (int i = 1; i < numSteps; i++) {
            candidates.push_back(step * i);
        }
    }

    // Principal axis angles
    if (usePrincipalAxis) {
        double pa = principalAxisAngle(polygon);
        // Add principal axis and orthogonal
        candidates.push_back(static_cast<float>(-pa));
        candidates.push_back(static_cast<float>(-pa + 90.0));
        candidates.push_back(static_cast<float>(-pa + 180.0));
        candidates.push_back(static_cast<float>(-pa + 270.0));

        // Add fine offsets around principal axis
        for (float offset : {-5.0f, 5.0f, -10.0f, 10.0f}) {
            candidates.push_back(static_cast<float>(-pa + offset));
            candidates.push_back(static_cast<float>(-pa + 90.0 + offset));
        }
    }

    // Normalize all to [0, rotationLimit)
    for (auto& a : candidates) {
        a = normalizeAngle(a);
        if (a >= rotationLimit) a = std::fmod(a, rotationLimit);
    }

    // Deduplicate
    return deduplicateAngles(candidates, 0.5f);
}

double ContinuousRotation::boundingBoxArea(const NFP& polygon, float angleDeg) {
    if (polygon.length() < 2) return 0;

    double rad = angleDeg * M_PI / 180.0;
    double cosA = std::cos(rad);
    double sinA = std::sin(rad);

    double minX = std::numeric_limits<double>::max();
    double maxX = -std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxY = -std::numeric_limits<double>::max();

    for (int i = 0; i < polygon.length(); i++) {
        double rx = polygon[i].x * cosA - polygon[i].y * sinA;
        double ry = polygon[i].x * sinA + polygon[i].y * cosA;
        minX = std::min(minX, rx);
        maxX = std::max(maxX, rx);
        minY = std::min(minY, ry);
        maxY = std::max(maxY, ry);
    }

    return (maxX - minX) * (maxY - minY);
}

float ContinuousRotation::bestBoundingBoxRotation(
    const NFP& polygon,
    const std::vector<float>& candidates)
{
    float bestAngle = 0;
    double bestArea = std::numeric_limits<double>::max();

    for (float angle : candidates) {
        double area = boundingBoxArea(polygon, angle);
        if (area < bestArea) {
            bestArea = area;
            bestAngle = angle;
        }
    }

    return bestAngle;
}

std::vector<float> ContinuousRotation::rankByBoundingBox(
    const NFP& polygon,
    const std::vector<float>& candidates)
{
    std::vector<std::pair<double, float>> scored;
    scored.reserve(candidates.size());

    for (float angle : candidates) {
        double area = boundingBoxArea(polygon, angle);
        scored.push_back({area, angle});
    }

    std::sort(scored.begin(), scored.end());

    std::vector<float> result;
    result.reserve(scored.size());
    for (auto& [area, angle] : scored) {
        result.push_back(angle);
    }
    return result;
}

} // namespace nest
