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

	//리카티 공식 함수
	Eigen::MatrixXd RicattiEquation();

	// 0~ 최종 제어시간까지의 제어 값을 계산하는 함수
	void Calculate();

	//이전의 k값과 x의 값을 구하는 함수
	Eigen::MatrixXd FindPreK();
	Eigen::MatrixXd FindPreX();

	


};


//vector의 저장되어져있는 값을 저장하는 함수
template<typename T>
inline void LinearRegulator::MakeTxt(std::vector<T> const& v, std::string textname)
{
	//설정한 이름으로 만들고
	std::string f_name = textname + ".txt";
	std::ofstream file(f_name);

	if (file.is_open()) {
		for (int i = 0;i < v.size();i++) {
			//vector의 값을 파일에 쓰고 enter를 쓴다.

			file << v.at(i);
			file << "\n";


		}
	}


	file.close();

}

