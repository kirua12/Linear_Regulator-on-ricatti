#pragma once

#include "ControlFuntion.h"
#include <vector>
#include <string>
#include <fstream>


class LinearRegulator
{
	ControlFuntion c_control;

	template<typename T>
	void MakeTxt(std::vector<T> const& v, std::string textname);

public:

	LinearRegulator(ControlFuntion cf);

	//��īƼ ���� �Լ�
	Eigen::MatrixXd RicattiEquation();

	// 0~ ���� ����ð������� ���� ���� ����ϴ� �Լ�
	void Calculate();

	//������ k���� x�� ���� ���ϴ� �Լ�
	Eigen::MatrixXd FindPreK();
	Eigen::MatrixXd FindPreX();

	


};


//vector�� ����Ǿ����ִ� ���� �����ϴ� �Լ�
template<typename T>
inline void LinearRegulator::MakeTxt(std::vector<T> const& v, std::string textname)
{
	//������ �̸����� �����
	std::string f_name = textname + ".txt";
	std::ofstream file(f_name);

	if (file.is_open()) {
		for (int i = 0;i < v.size();i++) {
			//vector�� ���� ���Ͽ� ���� enter�� ����.

			file << v.at(i);
			file << "\n";


		}
	}


	file.close();

}

