#include<iostream>
#include<opencv2/core/core.hpp>
#include<ceres/ceres.h>
#include<fstream>

using namespace std;
/*
struct COST_FUNCTION
{
    COST_FUNCTION ( double x, double y ) : _x ( x ), _y ( y ) {}
    // 残差的计算
    template <typename T>
    bool operator() (
        const T* const abc,     // 模型参数，有3维
        T* residual ) const     // 残差
    {
        residual[0] = T ( _y ) - ceres::exp ( abc[0]*T ( _x ) *T ( _x )*T ( _x ) + abc[1]*T ( _x )*T ( _x ) + abc[2]*T ( _x )+abc[3] ); // y-exp(ax^2+bx+c)
        return true;
    }
    const double _x, _y;    // x,y数据
};
*/
struct COST_FUNCTION
{
    COST_FUNCTION ( double x, double y ) : _x ( x ), _y ( y ) {}
    template<typename T>
    bool operator() (const T* const abcd, T* residual )
    const{
        residual[0] = T(_y) - ceres::exp( abcd[0]*T(_x)*T(_x)*T(_x)+abcd[1]*T(_x)*T(_x)+abcd[2]*T(_x)+abcd[3]);
        return true;
    }
    const double _x,_y;
};

int main(int argc,char** argv){
    double A = 1.0, B =2.0 ,C =1.0 ,D =1.5 ,N =100.0 ,SIGMA = 1.0;
    ofstream fout("random-result.txt");
    if(!fout.is_open()){
        cerr<<"can not open file\n";
        exit(1);
    }
    cv::RNG rng;
    double abcd[4] = {0};

    vector<double> x_data,y_data;
    for(int i = 0;i<N;i++){
        double x,y;
        x = i/N;
        y = exp (A*x*x*x + B*x*x + C*x + D ) +  rng.gaussian(SIGMA);
        x_data.push_back(x);
        y_data.push_back(y);
        fout<<x_data[i]<<" "<<y_data[i]<<"\n";
        cout<<x_data[i]<<"\t"<<y_data[i]<<"\n";
    }

    ceres::Problem problem;
    for( int i=0;i<N;i++){
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<COST_FUNCTION,1,4>(
                new COST_FUNCTION(  x_data[i],  y_data[i])
            ),
            nullptr,
            abcd
        );
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem ,&summary);

    cout<<summary.BriefReport()<<endl;
    cout<<"estimated a,b,c,d = ";
    for (auto i:abcd){
        cout<<i<<" ";
        fout<<i<<" ";
    }
    
    cout<<"\n";
    fout.close();
    return 0;
}
