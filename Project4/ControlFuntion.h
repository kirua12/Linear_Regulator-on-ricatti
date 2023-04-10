#pragma once
#include<Eigen\Dense>
#include<iostream>

class ControlFuntion
{
	//초기 x1, x2값
	double c_x1;
	double c_x2;

	// K,A,B,Q,R 행렬
	Eigen::MatrixXd c_K;
	Eigen::MatrixXd c_A;
	Eigen::MatrixXd c_B;
	Eigen::MatrixXd c_Q;
	Eigen::MatrixXd c_R;

	//종료시점의 시간
	int c_t;





	

	

	//리카티 공식


public:
	ControlFuntion();

	// x1과 x2의 미분 함수
	double DifferX1();
	double DifferX2();

	// u행동행렬
	double U();

	//get 함수
	Eigen::MatrixXd GetA();
	Eigen::MatrixXd GetB();
	Eigen::MatrixXd GetK();
	Eigen::MatrixXd GetR();
	Eigen::MatrixXd GetQ();
	double GetX1();
	double GetX2();
	//처음 시간 리턴함수
	int GetInitialTime();



	//Set 함수
	void SetK(Eigen::MatrixXd K);
	void SetX(Eigen::MatrixXd x);





};

