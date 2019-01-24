#ifndef COSTFUNCTION_H_
#define COSTFUNCTION_H_
#include<ceres/ceres.h>

class COST_FUNCTION{
    public:
    COST_FUNCTION(double x,double y):_x(x),_y(x) {};
    private:
    const double _x,_y;
    template<typename T>
    bool COST_FUNCTION::operator() (const T* const abcd, T* residual)
    {
            residual = T(_y) - 
            T(ceres::exp(
                abcd[0]*T(_x)*T(_x)*T(_x)   +
                abcd[1]*T(_x)*T(_x)     +
                abcd[2]*T(_x)      +
                abcd[3]   
                ));
            return true;
    }
};


#endif