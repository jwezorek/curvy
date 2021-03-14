#pragma once

#include <tuple>
#include "circle.h"

double euclidean_distance(double x1, double y1, double x2, double y2);
double euclidean_distance(const std::tuple<double, double>& p1, const std::tuple<double, double>& p2);
double normalize(const double value, const double start, const double end);
std::tuple<int, int> to_scr_coords(double x, double y, double logical_sz, int pixel_sz);
std::tuple<double, double> from_scr_coords(int x, int y, double logical_sz, int pixel_sz);
std::tuple<double, double> from_scr_coords(const std::tuple<int,int>& pt, double logical_sz, int pixel_sz);
std::tuple<int, int, int, int> to_scr_coords(double x1, double y1, double x2, double y2, double logical_sz, int pixel_sz);
std::tuple<int, int, int, int> to_scr_coords(const std::tuple<double, double, double, double>& rect , double logical_sz, int pixel_sz);
double pi();
curvy::circle circle_through_point(const std::tuple<double, double> pt);
bool is_pt_on_circle(const curvy::circle& c, std::tuple<double, double> pt, double eps);