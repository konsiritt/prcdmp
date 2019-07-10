#ifndef COMMON_H
#define COMMON_H
#include <string>
#include <vector>
#include <numeric>
#include <jsoncpp/json/value.h>
#include <unistd.h>

bool fileExists (const std::string& name);

void moveJsonArrayToVec(Json::Value jsonArr, std::vector<double> &vec);

void moveJsonArrayToVec(Json::Value jsonArr, std::array<double,7> &stdarr);

double inner_product(double *first, double *second, int size);

double accumulate(double *arr, int size);

#endif // COMMON_H
