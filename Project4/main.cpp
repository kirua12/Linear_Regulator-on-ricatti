#include "ControlFuntion.h"
#include "LinearRegulator.h"

int main() {
	ControlFuntion cf;
	LinearRegulator lr(cf);

	lr.Calculate();

}