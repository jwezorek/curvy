#pragma once

#include <tuple>

double euclidean_distance(double x1, double y1, double x2, double y2);
double euclidean_distance(const std::tuple<double, double>& p1, const std::tuple<double, double>& p2);
double normalize(const double value, const double start, const double end);
double pi();