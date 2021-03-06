// ISA_AltitudeCrunch.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include "rapidcsv.h"

float prompt_user_input()
{
	// Prompt User Input, Validate
	std::cout << "Input Desired Altitude (m) : ";
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


int main()
{

	// Load in ISA Profile Data from file:
	rapidcsv::Document doc("ISA_temp_profile.csv", rapidcsv::LabelParams(0,-1));

	std::vector<std::string> names = doc.GetColumn<std::string>("Names");
	std::vector<float> heights = doc.GetColumn<float>("Height");
	std::vector<float> temps = doc.GetColumn<float>("Temperature");
	std::vector<float> rates = doc.GetColumn<float>("Lapse");
	std::vector<float> pressures = doc.GetColumn<float>("Pressure");
	float gravity = 9.80665;
	float r_value = 287.00;

	float user_alt = prompt_user_input();

	// Return closet height as well as corresponding index
	base_params params = find_closet_height(user_alt, heights);

	if (params.index == -1)
	{
		params.index = 0;
	}

	// Calculate upwards from sea level to base value index
	float base_temp = temps[0];
	float base_p = pressures[0];
	float base_d = 1.225;
	float high_p = 0;
	float high_d = 0;
	float high_temp = 0;
	int index = 0;
	for (int i = 0; i <= params.index; i++)
	{
		// If we are not isothermal:
		if (rates[i] != 0)
		{
			// Calculate Temperature for next Index
			high_temp = base_temp + ((rates[i] / 1000) * (heights[i+1] - heights[i]));
			high_p = pow( (high_temp / base_temp), ( (-1 * gravity) / ((rates[i]/1000) * r_value))) * base_p;
			high_d = high_p / (r_value * high_temp);

		}
		// If we are in an isothermal layer
		else
		{
			high_temp = base_temp;
			high_p = exp(((-1 * gravity) / (r_value*base_temp)) * (heights[i + 1] - heights[i])) * base_p;
			high_d = exp(((-1 * gravity) / (r_value*base_temp)) * (heights[i + 1] - heights[i])) * base_d;
		}
		// Make the highs the bases now and move onto next iteration
		base_temp = high_temp;
		base_p = high_p;
		base_d = high_d;
		index = i;

	}

	float target_t, target_p, target_d;

	// Calculate to target alt
	if (rates[index] != 0)
	{
		target_t = base_temp + ((rates[params.index] / 1000) * (user_alt- params.closet_height));
		target_p = pow((target_t / base_temp), ((-1 * gravity) / ((rates[params.index] / 1000) * r_value))) * base_p;
		target_d = target_p / (r_value * target_t);
		
	}
	else
	{
		target_t = base_temp;
		target_p = exp(((-1 * gravity) / (r_value*target_t)) * (user_alt - params.closet_height)) * base_p;
		target_d = exp(((-1 * gravity) / (r_value*target_t)) * (user_alt - params.closet_height)) * base_d;
	}

	std::cout << "Temperature " << target_t << std::endl;
	std::cout << "Pressure " << target_p << std::endl;
	std::cout << "Density " << target_d << std::endl;







}



