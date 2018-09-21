// ISA_AltitudeCrunch.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include "rapidcsv.h"




int main()
{

	// Load in ISA Profile Data from file:
	rapidcsv::Document doc("ISA_temp_profile.csv", rapidcsv::LabelParams(0,-1));

	std::vector<std::string> names = doc.GetColumn<std::string>("Names");
	std::vector<float> heights = doc.GetColumn<float>("Height");
	std::vector<float> temps = doc.GetColumn<float>("Temperature");
	std::vector<float> rates = doc.GetColumn<float>("Lapse");
	std::vector<float> pressures = doc.GetColumn<float>("Pressure");


	// Prompt User Input
	std::cout << "Input Desired Altitude: ";
	float usr_alt;
	std::cin >> usr_alt;


	
	std::cout << "CSV File Loaded" << std::endl;




}

