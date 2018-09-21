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

struct base_params
{
	float closet_height;
	int index;
};

base_params find_closet_height(float target, std::vector<float> heights)
{

	int i = 0;

	while (heights[++i] < target);

	
	
	return { heights[--i], --i };


}

float calc_temp(std::vector<float> temps, std::vector<float> lapse, std::vector<float> height, int high_index, int low_index)
{
	float T = temps[low_index] + (lapse[low_index] * (height[high_index] - height[low_index]));
	return T;

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
	float gravity = 9.81;
	float r_value = 287;

	float user_alt = prompt_user_input();

	// Return closet height as well as corresponding index
	base_params params = find_closet_height(user_alt, heights);

	// Calculate upwards from sea level to base value index
	float base_temp = temps[params.index];
	float base_p = pressures[0];
	for (int i = 0; i <= params.index; i++)
	{

		// Calculate Temperature for next Index
		float high_temp = calc_temp(temps, rates, heights, params.index, i);
		// Use this to calcuate Pressure

		// If we are not isothermal:
		if (rates[i] != 0)
		{
			float heigh_p = (high_temp / base_temp) ^ (0 - gravity / (rates[i] * r_value)) * base_p;

		}



	}







}



