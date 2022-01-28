#include "controller/eigentest.h"
#include "Eigen/Dense"

#include <time.h>
#include <iostream>

using namespace Eigen;
using namespace std;

#define N 999

void eigen_test()
{
	double time1 = (double)clock() / CLOCKS_PER_SEC;

	MatrixXd m = MatrixXd::Random(N, N);            //初始化动态矩阵m,使用Random函数,初始化的值在[-1,1]区间内,矩阵大小N * N
	//cout << "m =" << endl << m << endl;
	MatrixXd n(N, N);
	n = m * m;
	n = m.inverse(); //逆矩阵
	auto a = m.determinant(); //行列式
	auto b = m.eigenvalues(); //特征值

	double time2 = (double)clock() / CLOCKS_PER_SEC;
	cout << "used time: " << double(time2 - time1) << endl;
}
