// ISA_AltitudeCrunch.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include "rapidcsv.h"

float prompt_user_input()
{
	// Prompt User Input, Validate
	std::cout << "Input Desired Altitude (km) : ";
	float usr_alt;
	std::cin >> usr_alt;

	while (1)
	{
		if (std::cin.fail())
		{
			std::cin.clear();
			std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			std::cout << "You have entered wrong input, try again" << std::endl;
			std::cin >> usr_alt;

		}
		if (!std::cin.fail())
			break;
	}

	return usr_alt;

}

float find_closet_height(float target, std::vector<float> heights)
{

	int i = 0;

	while (heights[++i] < target);

	return heights[--i];


}

int main()
{

	// Load in ISA Profile Data from file:
	rapidcsv::Document doc("ISA_temp_profile.csv", rapidcsv::LabelParams(0,-1));

	std::vector<std::string> names = doc.GetColumn<std::string>("Names");
	std::vector<float> heights = doc.GetColumn<float>("Height");
	std::vector<float> temps = doc.GetColumn<float>("Temperature");
	std::vector<float> rates = doc.GetColumn<float>("Lapse");
	std::vector<float> pressures = doc.GetColumn<float>("Pressure");

	float user_alt = prompt_user_input();
	float base_height = find_closet_height(user_alt, heights);

	std::cout << "f" << std::endl;






}



