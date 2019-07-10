#include "UTILS/common.h"

bool fileExists (const std::string& name) {
    if ( access( name.c_str(), F_OK ) != -1 ){
        return true;
    } else {
        return false;
    }
}

void moveJsonArrayToVec(Json::Value jsonArr, std::vector<double> &vec)
{
    if (vec.size())
    {
        throw "Please provide empty vector to Function : moveJsonArrayToVec";
    }
    for( int i=0; i<jsonArr.size(); i++)
    {
        vec.push_back(jsonArr[i].asDouble());
    }
}

void moveJsonArrayToVec(Json::Value jsonArr, std::array<double,7> &arr)
{
    for(int i=0; i<jsonArr.size(); i++)
    {
        arr[i] = jsonArr[i].asDouble();
    }
}

double accumulate(double *arr, int size)
{

    double sum=0;
    for (int i=0; i<size; i++)
    {
        sum += arr[i];
    }
    return sum;
}

// for vectors of same size
double inner_product(double *first, double *second, int size)
{
    //double init = 0.0;

    double sum =0;
    //#pragma omp parallel for reduction (+:sum)
    for( int i=0; i<size; i++)
    {
        sum+= first[i] * second[i] ;
    }

    return sum;
}
