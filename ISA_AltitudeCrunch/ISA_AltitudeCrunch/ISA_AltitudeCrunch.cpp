// ISA_AltitudeCrunch.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include "rapidcsv.h"




int main()
{


	rapidcsv::Document doc("ISA_temp_profile.csv", rapidcsv::LabelParams(0,-1));

	std::vector<std::string> names = doc.GetColumn<std::string>("Names");
	std::cout << "Read" << names.size() << "values" << std::endl;




}
