#pragma once
#include<Eigen\Dense>
#include<iostream>

class ControlFuntion
{
	//�ʱ� x1, x2��
	double c_x1;
	double c_x2;

	// K,A,B,Q,R ���
	Eigen::MatrixXd c_K;
	Eigen::MatrixXd c_A;
	Eigen::MatrixXd c_B;
	Eigen::MatrixXd c_Q;
	Eigen::MatrixXd c_R;

	//��������� �ð�
	int c_t;





	

	

	//��īƼ ����


public:
	ControlFuntion();

	// x1�� x2�� �̺� �Լ�
	double DifferX1();
	double DifferX2();

	// u�ൿ���
	double U();

	//get �Լ�
	Eigen::MatrixXd GetA();
	Eigen::MatrixXd GetB();
	Eigen::MatrixXd GetK();
	Eigen::MatrixXd GetR();
	Eigen::MatrixXd GetQ();
	double GetX1();
	double GetX2();
	//ó�� �ð� �����Լ�
	int GetInitialTime();



	//Set �Լ�
	void SetK(Eigen::MatrixXd K);
	void SetX(Eigen::MatrixXd x);





};

