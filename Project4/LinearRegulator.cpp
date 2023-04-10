#include "LinearRegulator.h"

LinearRegulator::LinearRegulator(ControlFuntion cf)
{
	c_control = cf;
}



//��īƼ ������ ���ϴ� ��
Eigen::MatrixXd LinearRegulator::RicattiEquation()
{
	//��īƼ�� �ʿ��� ���� ���� �Է�
	Eigen::MatrixXd A = c_control.GetA();
	Eigen::MatrixXd B = c_control.GetB();
	Eigen::MatrixXd Q = c_control.GetQ();
	Eigen::MatrixXd R = c_control.GetR();
	Eigen::MatrixXd K = c_control.GetK();




	//��īƼ ����
	Eigen::MatrixXd result = -K*A - A.transpose()*K - Q + K*B * R.inverse() * B.transpose()*K;

	return result;
}

void LinearRegulator::Calculate()
{
	// �ð��� ���� ����� �����ϴ� vector
	std::vector<double> k11;
	std::vector<double> k12;
	std::vector<double> k21;
	std::vector<double> k22;

	std::vector<double> x1;
	std::vector<double> x2;

	std::vector<double> u;

	Eigen::MatrixXd initial_k = c_control.GetK();



	//�ʱ갪�� �־��ش�
	k11.push_back(initial_k(0, 0));
	k12.push_back(initial_k(0, 1));
	k21.push_back(initial_k(1, 0));
	k22.push_back(initial_k(1, 1));

	x1.push_back(c_control.GetX1());
	x2.push_back(c_control.GetX2());

	u.push_back(c_control.U());






	// ������ ����� ã�ư��� ��� ����
	for (double t = c_control.GetInitialTime(); t > 0; t -= 0.01) {




		Eigen::MatrixXd pre_k = FindPreK();
		Eigen::MatrixXd pre_x = FindPreX();
		


		c_control.SetK(pre_k);
		c_control.SetX(pre_x);

		double pre_u = c_control.U();



		//���� ���� ������ vector�� ������
		k11.push_back(pre_k(0, 0));
		k12.push_back(pre_k(0, 1));
		k21.push_back(pre_k(1, 0));
		k22.push_back(pre_k(1, 1));

		x1.push_back(pre_x(0, 0));
		x2.push_back(pre_x(1, 0));

		u.push_back(pre_u);


	}


	//vector�� �� �մ� ������ txt���Ϸ� ������ִ� �Լ�
	MakeTxt(k11, "k11");
	MakeTxt(k12, "k12");
	MakeTxt(k21, "k21");
	MakeTxt(k22, "k22");

	MakeTxt(x1, "x1");
	MakeTxt(x2, "x2");

	MakeTxt(u, "u");





}

Eigen::MatrixXd LinearRegulator::FindPreK()
{

	//�̺� ������ ������ ��īƼ �������� ���� k���� ���� �� ����

	Eigen::MatrixXd k = c_control.GetK();

	Eigen::MatrixXd differ_k = RicattiEquation();

	Eigen::MatrixXd Pre_k = k - 0.01 * differ_k;


	return Pre_k;
}

Eigen::MatrixXd LinearRegulator::FindPreX()
{

	double x1 = c_control.GetX1();
	double x2 = c_control.GetX2();


	//�̺� ������ ������ ���� x���� ���� �� ����
 	double pre_x1 = x1 - 0.01 * c_control.DifferX1();
	double pre_x2 = x2 - 0.01 * c_control.DifferX2();


	Eigen::MatrixXd x = Eigen::MatrixXd(2, 1);
	x << pre_x1,
		pre_x2;



	return x;
}
