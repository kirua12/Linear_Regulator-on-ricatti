#include "ControlFuntion.h"

double ControlFuntion::U()
{
	//k와 x의 행렬 만들기
	Eigen::MatrixXd k;
	Eigen::MatrixXd x = Eigen::MatrixXd(2,1);

	//행동함수를 구할 때 x의 계수를 구하는 공식
	k = -c_R.inverse() * c_B.transpose()*c_K;

	//x의 행렬
	x << c_x1,
		c_x2;


	Eigen::MatrixXd result = -4 * k * x;

	return result(0,0);
}

double ControlFuntion::DifferX1()
{
	return c_x2;
}

double ControlFuntion::DifferX2()
{
	double result = 2 * c_x1 - c_x2 + U();
	return result;
}




//생성자 기본변수 입력
ControlFuntion::ControlFuntion()
{   //행렬의 크기 설정
	c_K = Eigen::MatrixXd(2, 2);
	c_A = Eigen::MatrixXd(2, 2);
	c_B = Eigen::MatrixXd(2, 1);
	c_Q = Eigen::MatrixXd(2, 2);
	c_R = Eigen::MatrixXd(1,1);

	c_t = 15;
	c_A << 0, 1,
		   2, -1;

	c_B << 0,
		   1;


	c_Q << 1, 0,
		   0, 0.5;

	c_R << 0.25;

	c_K << 0, 0,
		   0, 0;



	c_x1 = -4;
	c_x2 = 4;

}

Eigen::MatrixXd ControlFuntion::GetA()
{
	return c_A;
}

Eigen::MatrixXd ControlFuntion::GetQ()
{
	return c_Q;
}
Eigen::MatrixXd ControlFuntion::GetR()
{
	return c_R;
}
Eigen::MatrixXd ControlFuntion::GetB()
{
	return c_B;
}
Eigen::MatrixXd ControlFuntion::GetK()
{
	return c_K;
}

double ControlFuntion::GetX1() {
	return c_x1;
}

double ControlFuntion::GetX2() {
	return c_x2;
}



void ControlFuntion::SetK(Eigen::MatrixXd K) {
	c_K = K;
}


void ControlFuntion::SetX(Eigen::MatrixXd x) {
	c_x1 = x(0, 0);
	c_x2 = x(1, 0);

}






int ControlFuntion::GetInitialTime() {
	return  c_t;
}