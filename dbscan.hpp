#ifndef _DBSCAN_
#define _DBSCAN_

#include <vector>
#include <functional>
namespace dbscan
{

typedef int Label;
static const Label OUTLIER = -1;
static const Label UNDEFINED = 0;

using DistanceMatrix = std::vector<std::vector<double>>;
/**
 * @brief default distance function to two points of dimension 2 
 * 
 * @param a Point a
 * @param b Point b
 * 
 * @return Its euclidean distance
 */
template<typename Point>
double inline distance(const Point &a, const Point &b)
{
    return std::sqrt(std::pow((a.x - b.x), 2) + std::pow(a.y - b.y, 2));
};


template<typename Point>
inline std::vector<int>  rangeQuery(const std::vector<Point> &points,
                                    const double &eps,
                                    const int &q,
                                    DistanceMatrix &distanceMatrix)
{
    std::vector<int> neighbors;
    for (int p = 0; p < points.size(); ++p) {
        if (q == p)
            continue;
        if (distanceMatrix[q][p] < 0) {
            distanceMatrix[p][q] = distance(points[p], points[q]);
            distanceMatrix[q][p] = distanceMatrix[p][q];
        }
        if (distanceMatrix[q][p] < eps) {
            neighbors.push_back(p);
        }
    }
    return neighbors;
}
/**
 * @brief Calculates dbscan of list of points, as explained in https://en.wikipedia.org/wiki/DBSCAN
 * @details The only difference is that it computes the 
 *          distance matrix before hand, and you can provide a distance function to your points
 * @return Labels of the submitted points 
 */
template<typename Point>
std::vector<Label> dbscan(const std::vector<Point> &points,
                          const double &minNeighbors,
                          const double &eps,
                          std::function<double(const Point &a, const Point &b)> distanceFunction = nullptr)
{
    DistanceMatrix distanceMatrix(points.size(), std::vector<double>(points.size(), -1));
    if (distanceFunction == nullptr) {
        for (int i = 0; i < points.size(); ++i) {
            for (int j = i + 1; j < points.size(); ++j) {
                distanceMatrix[i][j] = distance(points[i], points[j]);
                distanceMatrix[j][i] = distanceMatrix[i][j];
            }
        }
    } else {
        for (int i = 0; i < points.size(); ++i) {
            for (int j = i + 1; j < points.size(); ++j) {
                distanceMatrix[i][j] = distanceFunction(points[i], points[j]);
                distanceMatrix[j][i] = distanceMatrix[i][j];
            }
        }
    }
    std::vector<Label> labels(points.size(), UNDEFINED);
    int C = 0;
    for (int i = 0; i < points.size(); ++i) {
        if (labels[i] != UNDEFINED)
            continue;
        auto neighbors = rangeQuery(points, eps, i, distanceMatrix);
        if (neighbors.size() < minNeighbors)
            continue;
        C += 1;
        labels[i] = C;
        int k = 0, j;
        while (k < neighbors.size()) {
            j = neighbors[k];
            if (labels[j] == OUTLIER)
                labels[j] = C;
            if (labels[j] != UNDEFINED) {
                ++k;
                continue;
            }
            labels[j] = C;
            auto neighborJ = rangeQuery(points, eps, j, distanceMatrix);
            if (neighborJ.size() >= minNeighbors)
                neighbors.insert(neighbors.end(), neighborJ.begin(), neighborJ.end());
            ++k;
        }
    }
    return labels;
};
}
#endif
