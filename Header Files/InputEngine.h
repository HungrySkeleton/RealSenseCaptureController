#pragma once
#include <chrono>
#include <conio.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <time.h>
#include <windows.h>

using namespace std;



void welcome_message(string version){
	printf("Welcome to AIRM Consulting's capture controller v.%s for Intel Realsense cameras.\n", version);
	printf("Answer the following prompts and allow camera to properly shut down after capture\n");
	printf("Use \"exit\" at any point during setup to close the camera.\n");
}


int get_sleep_time() {
	int sleep_interval;
	printf("Enter in the time between captures in seconds\n");
	cin >> sleep_interval;
	return sleep_interval;
}

string make_lowercase(string in_string)
{
	std::transform(in_string.begin(), in_string.end(), in_string.begin(),
		[](unsigned char c) { return std::tolower(c); });
	return in_string;
}

int check_good_input(string in_string, vector<string> options)
{
	int num_options = options.size();
	int good_input = 0;
	for (int ii = 0; ii < num_options; ii += 1)
	{
		if (in_string == options[ii])
		{
			good_input = 1;
			break;
		}
	}

	if (good_input == 0) { printf("\nMake sure to choose one of the \"options\" provided.\n"); }
	return good_input;
}

bool has_suffix(std::string str, std::string suffix)
{
	return str.size() >= suffix.size() &&
		str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

string strip_quotes(string str_in)
{
	string stripped = "test";

	if (has_suffix(str_in, "\""))
	{
		size_t str_len = str_in.length();
		stripped = str_in.substr(1, str_len - 2);
	}
	else
	{
		stripped = str_in;
	}

	return stripped;
}


string add_slash(string directory)
{
	if (!(has_suffix(directory, "/") || has_suffix(directory, "\\")))
	{
		directory += "/"; //ensure slash on end of directory
	}
	return directory;
}

string begin_loop_prompt()
{
	string start_loop;
	int good_response;
	good_response = 0;
	while (good_response == 0)
	{
		printf("\nEnter \"go\" to begin capturing\n");
		cin >> start_loop;
		start_loop = make_lowercase(start_loop);
		if (start_loop == "exit") { return "exit"; }
		good_response = check_good_input(start_loop, { "go" });
	}
	return start_loop;
}

