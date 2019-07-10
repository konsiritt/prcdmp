
#include <iostream>
#include <chrono>
#include <vector>
#include <omp.h>

using namespace std::chrono;

int main()
{
    std::cout<<"testing"<<std::endl;
    int dim =1000;
    double arr1[dim];
    double arr2[dim];

    for (int i=0; i<dim ; i++)
    {
        arr1[i] = i*2;
        arr2[i] = i*3;
    }

    double sum = 0;
    //
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    #pragma omp parallel for reduction (+:sum)
    for (int i=0; i<dim; i++)
    {
        sum+=arr1[i]*arr2[i];
    }
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>( t2 - t1 ).count();
    std::cout<<"It took that much time boy: "<<duration<<std::endl;
    std::cout<<"innter product: "<<sum<<std::endl;

    return 0;
}
