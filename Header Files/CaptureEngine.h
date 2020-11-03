#pragma comment(lib, "k4a.lib")
#pragma warning(disable:4996)
#include <algorithm>
#include <chrono>
#include <conio.h>
#include <fstream>
#include <iostream>
/*#include <k4a/k4a.h>
#include <k4a/k4atypes.h> */
#include <limits>
#include <malloc.h>
#include <math.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <thread>
#include <time.h>
#include <vector>
#include <Windows.h>
#include "inputEngine.h"

using namespace std;
using namespace cv;
using namespace std::this_thread;     // sleep_for, sleep_until
using namespace std::chrono_literals; // ns, us, ms, s, h, etc.
using std::chrono::system_clock;

class CAMERA
{
public:
	k4a_device_t device;
	string serial;
	k4a_wired_sync_mode_t role;
	k4a_depth_mode_t depth_mode;
	k4a_color_resolution_t resolution;
	k4a_image_format_t image_format;
	k4a_fps_t fps;
	int delay;
	k4a_calibration_t calibration;
	k4a_transformation_t intr_transform;
	k4a_capture_t recent_capture;
	k4a_device_configuration_t config;
};

inline std::pair<int, int> GetDepthModeRange(const k4a_depth_mode_t depthMode)
/*this and the following function are obtained from the link below, which contains other similar useful data
https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/examples/viewer/opengl/k4astaticimageproperties.h*/
{
	switch (depthMode)
	{
	case K4A_DEPTH_MODE_NFOV_2X2BINNED:
		return { (int)500, (int)5800 };
	case K4A_DEPTH_MODE_NFOV_UNBINNED:
		return { (int)500, (int)4000 };
	case K4A_DEPTH_MODE_WFOV_2X2BINNED:
		return { (int)250, (int)3000 };
	case K4A_DEPTH_MODE_WFOV_UNBINNED:
		return { (int)250, (int)2500 };
	case K4A_DEPTH_MODE_PASSIVE_IR:
	default:
		throw std::logic_error("Invalid depth mode!");
	}
}

inline std::pair<int, int> GetIrLevels(const k4a_depth_mode_t depthMode)
{
	switch (depthMode)
	{
	case K4A_DEPTH_MODE_PASSIVE_IR:
		return { (int)0, (int)100 };
	case K4A_DEPTH_MODE_OFF:
		throw std::logic_error("Invalid depth mode!");
	default:
		return { (int)0, (int)1000 };
	}
}

void close_device(CAMERA camera)
{
	if (camera.intr_transform != NULL) { k4a_transformation_destroy(camera.intr_transform); }
	k4a_device_stop_cameras(camera.device);
	k4a_device_close(camera.device);
	return;
}

void exit(CAMERA camera)
{
	if (camera.device != NULL)
	{
		printf("\nShutting down camera %s ...", camera.serial);
		close_device(camera);
		printf("... Device closed!\n");
	}
	else
	{
		printf("\nDevice %s is alread closed.\n", camera.serial);
	}
	return;
}

void exit_all(std::vector<CAMERA> devices)
{
	size_t num_devices = devices.size();
	for (int ii = 0; ii < num_devices; ii += 1)
	{
		exit(devices[ii]);
	}
	return;
}

int count_devices()
{
	uint32_t device_count = k4a_device_get_installed_count();
	printf("\n%i device(s) attached.\n", device_count);
	if (device_count == 0)
	{
		printf("Make sure your device is plugged into a USB 3.0 port and powered!\n");
		return 0;
	}
	return device_count;
}

string get_serial(k4a_device_t device)
{
	size_t serial_number_length = 0;
	if (K4A_BUFFER_RESULT_TOO_SMALL != k4a_device_get_serialnum(device, NULL, &serial_number_length))
	{
		printf("Failed to get serial number length\n");
		k4a_device_close(device);
		return NULL;
	}
	char* serial_number = (char*)(malloc(serial_number_length));
	if (serial_number == NULL)
	{
		printf("Failed to allocate memory for serial number (%zu bytes)\n", serial_number_length);
		return NULL;
	}
	if (K4A_BUFFER_RESULT_SUCCEEDED != k4a_device_get_serialnum(device, serial_number, &serial_number_length))
	{
		printf("Failed to get serial number\n");
		free(serial_number);
		return NULL;
	}
	return string(serial_number);
}

k4a_device_t open_single_device(int device_index)
{
	k4a_device_t device = NULL;
	if (K4A_FAILED(k4a_device_open(device_index, &device)))
	{
		printf("%d: Failed to open device\n", device_index);
		return NULL;
	}
	string serial = get_serial(device);
	if (!serial.length()) { return NULL; }
	//printf("Opened device %d: %s\n", device_index, serial);
	return device;
}

k4a_wired_sync_mode_t assign_role(k4a_device_t device)
{
	bool sync_in;
	bool sync_out;
	k4a_device_get_sync_jack(device, &sync_in, &sync_out);
	k4a_wired_sync_mode_t role;
	if (sync_in == 0 && sync_out == 0) { role = K4A_WIRED_SYNC_MODE_STANDALONE; }
	if (sync_in == 0 && sync_out == 1) { role = K4A_WIRED_SYNC_MODE_MASTER; }
	if (sync_in == 1 && sync_out == 0) { role = K4A_WIRED_SYNC_MODE_SUBORDINATE; }
	if (sync_in == 1 && sync_out == 1) { role = K4A_WIRED_SYNC_MODE_SUBORDINATE; }
	return role;
}

k4a_calibration_t calibrate_device(CAMERA camera)
//obtains a calibration structure for the device, which is required to apply transformations
{
	k4a_calibration_t calibration_structure;
	switch (k4a_device_get_calibration(camera.device, camera.depth_mode, camera.resolution, &calibration_structure))
	{
	case K4A_WAIT_RESULT_SUCCEEDED:
		break;
	case K4A_RESULT_FAILED:
		printf("Calibration failed :(\n");
		exit(camera);
	}
	return calibration_structure;
}

k4a_transformation_t get_transformation(CAMERA camera)
{
	k4a_transformation_t transformation_handle = k4a_transformation_create(&camera.calibration);
	if (!transformation_handle) //NULL returned if not created
	{
		printf("Null transformation handle :(\n");
		exit(camera);
	}
	return transformation_handle;
}

k4a_device_configuration_t configure_start(CAMERA camera)
{
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = camera.fps;
	config.depth_mode = camera.depth_mode;
	config.color_format = camera.image_format;
	config.color_resolution = camera.resolution;
	config.synchronized_images_only = true;
	config.wired_sync_mode = camera.role;
	config.depth_delay_off_color_usec = camera.delay;
	if (K4A_FAILED(k4a_device_start_cameras(camera.device, &config)))
	{
		printf("Failed to start cameras!\n");
		return config;
	}
	else { printf("\nCamera %s started successfully!\n", camera.serial); return config; }
}

std::vector<CAMERA> set_exposure(std::vector<CAMERA> cameras, uint32_t value_usec)
{
	int num_devices = cameras.size();
	k4a_result_t worked;
	for (int ii = 0; ii < num_devices; ii += 1)
	{
		if (value_usec == 0)
		{
			worked = k4a_device_set_color_control(cameras[ii].device, K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_AUTO, value_usec);
		}
		else
		{
			worked = k4a_device_set_color_control(cameras[ii].device, K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_MANUAL, value_usec);
		}
		if (worked == K4A_RESULT_FAILED)
		{
			printf("Failed to set exposure for %s", cameras[ii].serial);
			return {};
		}
	}
	return cameras;
}

void print_role(CAMERA camera)
{
	printf("This camera is ");
	if (camera.role == K4A_WIRED_SYNC_MODE_MASTER) { printf("master\n"); }
	if (camera.role == K4A_WIRED_SYNC_MODE_SUBORDINATE) { printf("subordinate\n"); }
	if (camera.role == K4A_WIRED_SYNC_MODE_STANDALONE) { printf("standalone\n"); }
	return;
}

int get_delay(int num_devices, int this_device, k4a_depth_mode_t depth_mode)
{
	//all numbers in microseconds
	int MIN_RECOMMENDED = 160;
	int PULSE_WIDTH = 125;
	int IDLE_TIME = 1450;
	if (depth_mode == K4A_DEPTH_MODE_WFOV_UNBINNED) { IDLE_TIME = 2390; }
	int TOTAL_WINDOW = PULSE_WIDTH + IDLE_TIME;
	int allowable = floor(TOTAL_WINDOW / MIN_RECOMMENDED);
	if (num_devices > allowable)
	{
		printf("Too many devices connected! Images may have error if more than %d are connected.\n", allowable);
	}
	int base_offset = MIN_RECOMMENDED / 2;
	int factor = this_device;
	if (this_device % 2 == 0)
	{
		base_offset *= -1;
		factor += 1;
	}
	int this_offset = factor * base_offset;
	return this_offset;
}

vector<CAMERA> default_color_settings(vector<CAMERA> cameras)
{
	int num_devices = cameras.size();
	k4a_result_t worked;
	for (int ii = 0; ii < num_devices; ii += 1)
	{
		worked = k4a_device_set_color_control(cameras[ii].device, K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_AUTO, 0);
		if (worked == K4A_RESULT_FAILED)
		{
			printf("Failed to set default exposure for %s", cameras[ii].serial);
			return {};
		}
		worked = k4a_device_set_color_control(cameras[ii].device, K4A_COLOR_CONTROL_WHITEBALANCE, K4A_COLOR_CONTROL_MODE_AUTO, 0);
		if (worked == K4A_RESULT_FAILED)
		{
			printf("Failed to set default white balance for %s", cameras[ii].serial);
			return {};
		}
		worked = k4a_device_set_color_control(cameras[ii].device, K4A_COLOR_CONTROL_BRIGHTNESS, K4A_COLOR_CONTROL_MODE_MANUAL, 128);
		if (worked == K4A_RESULT_FAILED)
		{
			printf("Failed to set default brightness for %s", cameras[ii].serial);
			return {};
		}
		worked = k4a_device_set_color_control(cameras[ii].device, K4A_COLOR_CONTROL_CONTRAST, K4A_COLOR_CONTROL_MODE_MANUAL, 5);
		if (worked == K4A_RESULT_FAILED)
		{
			printf("Failed to set default contrast for %s", cameras[ii].serial);
			return {};
		}
		worked = k4a_device_set_color_control(cameras[ii].device, K4A_COLOR_CONTROL_SATURATION, K4A_COLOR_CONTROL_MODE_MANUAL, 32);
		if (worked == K4A_RESULT_FAILED)
		{
			printf("Failed to set default saturation for %s", cameras[ii].serial);
			return {};
		}
		worked = k4a_device_set_color_control(cameras[ii].device, K4A_COLOR_CONTROL_SHARPNESS, K4A_COLOR_CONTROL_MODE_MANUAL, 4);
		if (worked == K4A_RESULT_FAILED)
		{
			printf("Failed to set default sharpness for %s", cameras[ii].serial);
			return {};
		}
		worked = k4a_device_set_color_control(cameras[ii].device, K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION, K4A_COLOR_CONTROL_MODE_MANUAL, 0);
		if (worked == K4A_RESULT_FAILED)
		{
			printf("Failed to set default backlight compensation for %s", cameras[ii].serial);
			return {};
		}
		worked = k4a_device_set_color_control(cameras[ii].device, K4A_COLOR_CONTROL_GAIN, K4A_COLOR_CONTROL_MODE_MANUAL, 0);
		if (worked == K4A_RESULT_FAILED)
		{
			printf("Failed to set default gain for %s", cameras[ii].serial);
			return {};
		}
		worked = k4a_device_set_color_control(cameras[ii].device, K4A_COLOR_CONTROL_POWERLINE_FREQUENCY, K4A_COLOR_CONTROL_MODE_MANUAL, 2);
		if (worked == K4A_RESULT_FAILED)
		{
			printf("Failed to set default powerline frequency for %s", cameras[ii].serial);
			return {};
		}
	}
	return cameras;
}

std::vector<CAMERA> get_devices(k4a_depth_mode_t depth, k4a_image_format_t format, k4a_color_resolution_t resolution, k4a_fps_t fps)
{
	int num_devices = count_devices();
	if (!num_devices) { return {}; }
	std::vector<CAMERA> cameras;
	cameras.resize(num_devices);
	int sub_count = 0;
	int stdl_count = 0;
	int master_count = 0;
	//first, open each device, check it's role, and index it in the right position
	for (int ii = 0; ii < num_devices; ii += 1)
	{
		CAMERA this_camera;
		this_camera.device = open_single_device(ii);
		this_camera.role = assign_role(this_camera.device);
		this_camera.serial = get_serial(this_camera.device);
		this_camera.depth_mode = depth;
		this_camera.image_format = format;
		this_camera.resolution = resolution;
		this_camera.fps = fps;
		if (!this_camera.serial.length()) { exit_all(cameras); }
		if (this_camera.role == K4A_WIRED_SYNC_MODE_MASTER)
		{
			cameras[num_devices - 1] = this_camera;
			master_count += 1;
		}
		if (this_camera.role == K4A_WIRED_SYNC_MODE_SUBORDINATE)
		{
			cameras[sub_count + stdl_count] = this_camera;
			sub_count += 1;
		}
		if (this_camera.role == K4A_WIRED_SYNC_MODE_STANDALONE)
		{
			cameras[sub_count + stdl_count] = this_camera;
			stdl_count += 1;
		}
		if (!cameras[ii].device) { exit_all(cameras); return {}; }
	}
	//now that cameras are in the right order, start and calibrate them
	for (int ii = 0; ii < num_devices; ii += 1)
	{
		cameras[ii].delay = get_delay(num_devices, ii, cameras[ii].depth_mode); //exposure would also be set here (before starting)
		cameras[ii].config = configure_start(cameras[ii]);
		print_role(cameras[ii]);
		cameras[ii].calibration = calibrate_device(cameras[ii]);
		if (!cameras[ii].device) { exit_all(cameras); return {}; }
		cameras[ii].intr_transform = get_transformation(cameras[ii]);
		if (!cameras[ii].device) { exit_all(cameras); return {}; }
	}
	cameras = default_color_settings(cameras);
	reverse(cameras.begin(), cameras.end()); //flips the list so that master is first
	return cameras;
}

static cv::Matx33f calibration_to_color_camera_matrix(const k4a_calibration_t cal)
{
	const k4a_calibration_intrinsic_parameters_t::_param& i = cal.color_camera_calibration.intrinsics.parameters.param;
	cv::Matx33f camera_matrix = cv::Matx33f::eye();
	camera_matrix(0, 0) = i.fx;
	camera_matrix(1, 1) = i.fy;
	camera_matrix(0, 2) = i.cx;
	camera_matrix(1, 2) = i.cy;
	return camera_matrix;
}

static vector<float> calibration_to_color_camera_dist_coeffs(const k4a_calibration_t cal)
{
	const k4a_calibration_intrinsic_parameters_t::_param& i = cal.color_camera_calibration.intrinsics.parameters.param;
	return { i.k1, i.k2, i.p1, i.p2, i.k3, i.k4, i.k5, i.k6 };
}

void intrinsic_to_csv(cv::Matx33f intrinsic, string serial, string main_path)
{
	string full_path = main_path + add_slash("calibration") + serial + "_intrinsic" + ".csv";

	vector<vector<double>> transformation_matrix = { {1,0,0},{0,1,0},{0,0,1} };
	for (int ii = 0; ii < 3; ii += 1)
	{
		for (int jj = 0; jj < 3; jj += 1)
		{
			transformation_matrix[ii][jj] = intrinsic(ii, jj);
		}
	}
	std::ofstream outfile;
	outfile.open(full_path);
	string to_write;
	for (int ii = 0; ii < 3; ii += 1)
	{
		for (int jj = 0; jj < 3; jj += 1)
		{
			if (jj < 2) { to_write = to_string(transformation_matrix[ii][jj]) + "\t"; }
			else { to_write = to_string(transformation_matrix[ii][jj]) + "\n"; }
			outfile << to_write;
		}
	}
	outfile.close();
}

void distortion_to_csv(vector<float> dist, string serial, string main_path)
{
	string full_path = main_path + add_slash("calibration") + serial + "_distortion" + ".csv";

	int num_coeffs = dist.size();

	std::ofstream outfile;
	outfile.open(full_path);
	string to_write;
	for (int ii = 0; ii < num_coeffs; ii += 1)
	{
		if (ii < (num_coeffs - 1)) { to_write = to_string(dist[ii]) + "\t"; }
		else { to_write = to_string(dist[ii]) + "\n"; }
		outfile << to_write;
	}
	outfile.close();
}

void write_intrinsics_out(string main_dir, vector<CAMERA> cameras)
{
	int num_devices = cameras.size();
	cv::Matx33f this_intrinsic;
	for (int ii = 0; ii < num_devices; ii += 1)
	{
		this_intrinsic = calibration_to_color_camera_matrix(cameras[ii].calibration);
		intrinsic_to_csv(this_intrinsic, cameras[ii].serial, main_dir);
	}
	return;
}

void write_distortion_out(string main_dir, vector<CAMERA> cameras)
{
	int num_devices = cameras.size();
	vector<float> this_distortion;
	for (int ii = 0; ii < num_devices; ii += 1)
	{
		this_distortion = calibration_to_color_camera_dist_coeffs(cameras[ii].calibration);
		distortion_to_csv(this_distortion, cameras[ii].serial, main_dir);
	}
	return;
}

int create_subdirectories(string main_dir, vector<CAMERA> cameras)
{
	vector<string> lvl1 = { "images/", "calibration/" };
	int num_devices = cameras.size();

	int successfully_made = true;

	string current_dir = main_dir;
	if (successfully_made) { successfully_made = check_directory_exists(current_dir + lvl1[0]); }
	if (successfully_made) { successfully_made = check_directory_exists(current_dir + lvl1[1]); }
	current_dir += lvl1[0];
	for (int ii = 0; ii < num_devices; ii += 1)
	{
		string serial_str = string(cameras[ii].serial);
		string this_dir = current_dir + serial_str;
		successfully_made = check_directory_exists(this_dir);
		if (!successfully_made) { break; }
	}
	if (successfully_made)
	{
		write_intrinsics_out(main_dir, cameras);
		write_distortion_out(main_dir, cameras);
	}
	return successfully_made;
}

k4a_capture_t capture_frame(CAMERA camera, uint32_t TIMEOUT_IN_MS)
{
	k4a_capture_t capture = NULL;
	switch (k4a_device_get_capture(camera.device, &capture, TIMEOUT_IN_MS))
	{
	case K4A_WAIT_RESULT_SUCCEEDED:
		//printf("\nindividual capture succeeded\n");
		break;
	case K4A_WAIT_RESULT_TIMEOUT:
		printf("Timed out waiting for a capture from camera %s\n", camera.serial);
		exit(camera);
		break;
	case K4A_WAIT_RESULT_FAILED:
		printf("Failed to read a capture from camera %s\n", camera.serial);
		exit(camera);
	}
	return capture;
}

std::vector<CAMERA> multi_capture(std::vector<CAMERA> cameras, uint32_t TIMEOUT_IN_MS)
{
	int num_devices = cameras.size();
	for (int ii = 0; ii < num_devices; ii += 1)
	{
		cameras[ii].recent_capture = capture_frame(cameras[ii], TIMEOUT_IN_MS);
		//_sleep(100);
	}
	return cameras;
}

void close_capture(k4a_capture_t capture)
{
	k4a_capture_release(capture);
}

string get_date_time()
{
	struct tm* tm;
	time_t t;
	char date_time[100];
	t = time(NULL);
	tm = localtime(&t);
	strftime(date_time, sizeof(date_time), "%Y%m%d_%H%M%S", tm);
	return std::string(date_time);
}

void save_fig(cv::Mat frame, string type, string bit_depth, string extension, string directory, string date_time, string serial)
{
	string full_path = directory + add_slash("images") + add_slash(serial);
	string filename = serial + "_" + type + "_" + bit_depth + "_" + date_time + extension;
	full_path += filename;

	imwrite(full_path, frame);
	return;
}

void show_fig(cv::Mat frame, int wait)
{
	namedWindow("window", WINDOW_NORMAL);
	moveWindow("window", 0, 0);
	imshow("window", frame);
	waitKey(wait);
	return;
}

static cv::Mat color_to_opencv(const k4a_image_t& im)
{
	cv::Mat cv_image_with_alpha(k4a_image_get_height_pixels(im), k4a_image_get_width_pixels(im), CV_8UC4, (void*)k4a_image_get_buffer(im));
	cv::Mat cv_image_no_alpha;
	cv::cvtColor(cv_image_with_alpha, cv_image_no_alpha, cv::COLOR_BGRA2BGR);
	return cv_image_no_alpha;
}

template<typename T>
inline void ConvertToGrayScaleImage(const T* imgDat, const int size, const int vmin, const int vmax, uint8_t* img)
{
	for (int i = 0; i < size; i++)
	{
		T v = imgDat[i];
		float colorValue = 0.0f;
		if (v <= vmin)
		{
			colorValue = 0.0f;
		}
		else if (v >= vmax)
		{
			colorValue = 1.0f;
		}
		else
		{
			colorValue = (float)(v - vmin) / (float)(vmax - vmin);
		}
		img[i] = (uint8_t)(colorValue * 255);
	}
}

k4a_image_t depth_to_color(CAMERA camera, k4a_image_t depth_image, k4a_image_t color_image)
{
	//attributes of the color image will be required
	uint color_height = k4a_image_get_height_pixels(color_image);
	uint color_width = k4a_image_get_width_pixels(color_image);
	uint color_stride = k4a_image_get_stride_bytes(color_image);
	//create a transformed depth image to be written into (Must be same format as depth_image, and same dimensions as color_image)
	k4a_image_t transformed_depth_image = NULL;
	switch (k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, color_width, color_height, color_width * (int)sizeof(uint16_t), &transformed_depth_image))
	{
	case K4A_RESULT_SUCCEEDED:
		break;
	case K4A_RESULT_FAILED:
		printf("Blank depth image creation failed (depth_to_color) :(\n");
	}
	//apply the transformation
	switch (k4a_transformation_depth_image_to_color_camera(camera.intr_transform, depth_image, transformed_depth_image))
	{
	case K4A_RESULT_SUCCEEDED:
		break;
	case K4A_RESULT_FAILED:
		printf("Depth Transformation failed :(\n");
	}
	return transformed_depth_image;
}

k4a_image_t change_IR_format(k4a_image_t raw_IR)
{
	//get dimensions
	uint height = k4a_image_get_height_pixels(raw_IR);
	uint width = k4a_image_get_width_pixels(raw_IR);
	uint stride = k4a_image_get_stride_bytes(raw_IR);
	//get the raw IR's buffer
	uint8_t* data = (uint8_t*)(void*)k4a_image_get_buffer(raw_IR);
	size_t size = k4a_image_get_size(raw_IR);
	//create the new image
	k4a_image_t new_IR = NULL; //initialize the new image to be written into
	k4a_memory_destroy_cb_t* destroyer = NULL; //not sure about these arguments - I think if you don't need the buffer re-stored after
	void* context = NULL; //use, they can be void. Don't need buffer re-stored and seems to function for now.
	switch (k4a_image_create_from_buffer(K4A_IMAGE_FORMAT_CUSTOM16, width, height, stride, data, size, destroyer, context, &new_IR))
	{
	case K4A_RESULT_SUCCEEDED:
		break;
	case K4A_RESULT_FAILED:
		printf("New IR Format could not be written :(\n");
	}
	return new_IR;
}

k4a_image_t IR_to_color(CAMERA camera, k4a_image_t depth_image, k4a_image_t IR_image_raw, k4a_image_t color_image)
{
	//first we must transform the raw depth image into the same reference frame as the color image, so they may be overlayed
	//attributes of the color image will be required
	uint color_height = k4a_image_get_height_pixels(color_image);
	uint color_width = k4a_image_get_width_pixels(color_image);
	uint color_stride = k4a_image_get_stride_bytes(color_image);
	//create a transformed image to be written into (Must be same format as depth_image, and same dimensions as color_image)
	//first, the IR_image must be changed into CUSTOM16 format (captured in IR16 format)
	k4a_image_t IR_image = change_IR_format(IR_image_raw);
	//now create blank image containers
	k4a_image_t transformed_IR_image = NULL;
	k4a_image_t transformed_depth_image = NULL;
	switch (k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM16, color_width, color_height, color_width * (int)sizeof(uint16_t), &transformed_IR_image))
	{
	case K4A_RESULT_SUCCEEDED:
		break;
	case K4A_RESULT_FAILED:
		printf("Blank IR image creation failed (IR_to_color) :(\n");
	}
	switch (k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, color_width, color_height, color_width * (int)sizeof(uint16_t), &transformed_depth_image))
	{
	case K4A_RESULT_SUCCEEDED:
		break;
	case K4A_RESULT_FAILED:
		printf("Blank depth image creation failed (IR_to_color) :(\n");
	}
	//apply the transformation
	switch (k4a_transformation_depth_image_to_color_camera_custom(camera.intr_transform, depth_image, IR_image, transformed_depth_image,
		transformed_IR_image, K4A_TRANSFORMATION_INTERPOLATION_TYPE_NEAREST, 0))
	{
	case K4A_RESULT_SUCCEEDED:
		break;
	case K4A_RESULT_FAILED:
		printf("IR/depth Transformation failed :(\n");
	}
	return transformed_IR_image;
}

void get_color(k4a_image_t image, int show, string extension, string directory, string date_time, string serial)
{
	cv::Mat no_alpha = color_to_opencv(image);
	if (show) { show_fig(no_alpha, 0); }
	save_fig(no_alpha, "color", "24", extension, directory, date_time, serial);
	return;
}

void get_depth(k4a_image_t image, std::pair <int, int> bounds, int show, string type, string extension, string directory, string date_time, string serial)
{
	int height = k4a_image_get_height_pixels(image);
	int width = k4a_image_get_width_pixels(image);
	int strides = k4a_image_get_stride_bytes(image);
	uint8_t* data = k4a_image_get_buffer(image);
	uint16_t* data16 = reinterpret_cast<uint16_t*>(data);
	std::vector<uint8_t> grayScaleImg(width * height);
	int MinValue = bounds.first;
	int MaxValue = bounds.second;
	ConvertToGrayScaleImage(data16, width * height, MinValue, MaxValue, grayScaleImg.data());
	const cv::Mat frame(cv::Size(width, height), CV_8U, grayScaleImg.data(), cv::Mat::AUTO_STEP);
	cv::Mat frame16(cv::Size(width, height), CV_16U, (void*)data16, cv::Mat::AUTO_STEP);
	//if (show) { show_fig(frame); }
	save_fig(frame, type, "08", extension, directory, date_time, serial);
	save_fig(frame16, type, "16", extension, directory, date_time, serial);
	return;
}

int check_key_hit()
{
	if (kbhit())
	{
		printf("\nKeystroke detected - camera shutting down ASAP.\n");
		return 1;
	}
	else { return 0; }
}

void release_images(vector<k4a_image_t> images)
{
	int num_images = images.size();
	for (int ii = 0; ii < num_images; ii += 1)
	{
		if (images[ii] != NULL) { k4a_image_release(images[ii]); }
	}
	return;
}

void live_viewer(vector<CAMERA> cameras, int timeout)
//maybe add in multicam view later (horizontal concatenate)
{
	printf("how many seconds would you like to view for?");
	int view_sec;
	cin >> view_sec;
	std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();

	while (std::chrono::duration<double>(std::chrono::system_clock::now() - start_time).count() <
		view_sec)
	{
		cameras = multi_capture(cameras, timeout);
		k4a_capture_t main_capture = cameras[0].recent_capture;
		k4a_image_t main_color_image = k4a_capture_get_color_image(main_capture);
		cv::Mat cv_main_color_image = color_to_opencv(main_color_image);
		show_fig(cv_main_color_image, 1);
		release_images({ main_color_image });
		close_capture(cameras[0].recent_capture);
	}
	destroyAllWindows();
	return;
}

int process_capture(vector<CAMERA> cameras, int show, string extension, string directory, int exit_watch)
{
	k4a_image_t color_image; //right out of camera
	k4a_image_t depth_image; //right out of camera
	k4a_image_t IR_image; //right out of camera
	k4a_image_t transformed_depth_image; //intrinsic
	k4a_image_t transformed_IR_image; //intrinsic

	int key_stroke = 0;
	string date_time = get_date_time();

	int num_devices = cameras.size();
	for (int ii = 0; ii < num_devices; ii += 1)
	{
		if (!exit_watch || (exit_watch && !key_stroke))
		{
			color_image = NULL; //right out of camera
			depth_image = NULL; //right out of camera
			IR_image = NULL; //right out of camera
			transformed_depth_image = NULL; //intrinsic
			transformed_IR_image = NULL; //intrinsic

			std::pair <int, int> IR_bounds = GetIrLevels(cameras[ii].depth_mode);
			std::pair <int, int> depth_bounds = GetDepthModeRange(cameras[ii].depth_mode);

			//extract raw images
			color_image = k4a_capture_get_color_image(cameras[ii].recent_capture);
			depth_image = k4a_capture_get_depth_image(cameras[ii].recent_capture);
			IR_image = k4a_capture_get_ir_image(cameras[ii].recent_capture);
			//transform to color space
			if (exit_watch && !key_stroke) { key_stroke = check_key_hit(); }
			transformed_depth_image = depth_to_color(cameras[ii], depth_image, color_image);
			if (exit_watch && !key_stroke) { key_stroke = check_key_hit(); }
			transformed_IR_image = IR_to_color(cameras[ii], depth_image, IR_image, color_image);

			//show/save these images
			if (exit_watch && !key_stroke) { key_stroke = check_key_hit(); }
			get_color(color_image, show, extension, directory, date_time, cameras[ii].serial);
			if (exit_watch && !key_stroke) { key_stroke = check_key_hit(); }
			get_depth(transformed_depth_image, depth_bounds, show, "depth", extension, directory, date_time, cameras[ii].serial);
			if (exit_watch && !key_stroke) { key_stroke = check_key_hit(); }
			get_depth(transformed_IR_image, IR_bounds, show, "IR", extension, directory, date_time, cameras[ii].serial);
			if (exit_watch && !key_stroke) { key_stroke = check_key_hit(); }

			release_images({ color_image, depth_image, IR_image,
							transformed_depth_image, transformed_IR_image});
			close_capture(cameras[ii].recent_capture);
		}
	}
	return key_stroke;
}

void close_all_captures(std::vector<CAMERA> cameras)
{
	int num_devices = cameras.size();
	for (int ii = 0; ii < num_devices; ii += 1)
	{
		close_capture(cameras[ii].recent_capture);
	}
	return;
}

void single_capture(vector<CAMERA> cameras, string extension, string directory, int timeout)
{
	int show = want_to_show();
	if (show < 0) { return; }

	string action = single_prompt();
	while (action == "capture" || action == "show")
	{
		if (action == "capture")
		{
			printf("\nBeginning capture ...");
			cameras = multi_capture(cameras, timeout);
			int key_stroke = process_capture(cameras, show, extension, directory, false);
			printf("... Capture complete!\n");
		}

		if (action == "show")
		{
			live_viewer(cameras, timeout);
		}
		action = single_prompt();
	}
	return;
}

void loop_capture(std::vector<CAMERA> cameras, string extension, string directory, int timeout, std::vector<int> periods, int repeats, int initial)
/*if repeats == 0, loop indefinitely. if repeats>0, loop that number of times.*/
/*Initial still needs to be programmed in - useful if first desired capture @ 3s so that 0s capture does not interfere*/
{
	int num_devices = cameras.size();
	int show_quit_frequency = 15;
	clock_t start, end;
	unsigned long cpu_time_used;
	int count = 0;
	int key_stroke;
	key_stroke = 0;
	size_t num_periods = periods.size();
	int repeats_so_far = 0;
	printf("Beginning captures ...\n\n");
	while (!key_stroke)
	{
		for (int ii = 0; ii < num_periods; ii += 1)
		{
			start = clock();
			if (count % show_quit_frequency == 0) { printf("Press any key at any point to stop capturing.\n"); }
			cameras = multi_capture(cameras, timeout);
			if (!key_stroke) { key_stroke = check_key_hit(); }
			if (key_stroke) { break; }
			key_stroke = process_capture(cameras, false, extension, directory, true);
			printf("Captures so far: %i\n", count + 1);
			count = count + 1;
			end = clock();
			cpu_time_used = ((unsigned long)(end - start)) / CLOCKS_PER_SEC;
			unsigned long new_wait = 1000 * (periods[ii] - cpu_time_used); //conversion to ms for sleep function
			if (new_wait > 0) { _sleep(new_wait); }
			else { if (!key_stroke) { printf("Looping slower than requested time :(\n"); } }
			if (!key_stroke) { key_stroke = check_key_hit(); }
			if (key_stroke) { break; }
		}
		repeats_so_far += 1;
		if (repeats > 0 && repeats_so_far == repeats) { break; }
	}
	printf("... Captures complete!\n");
	return;
}

unsigned long convert_hhmmss_to_sec(string this_hhmmss)
{
	int hour = 3600*stoi(this_hhmmss.substr(0, 2));
	int minute = 60*stoi(this_hhmmss.substr(2, 2));
	int second = stoi(this_hhmmss.substr(4, 2));
	int combined = hour + minute + second;
	unsigned long converted = (unsigned long)combined;
	return converted;
}

void timed_capture(vector<CAMERA> cameras, string extension, string directory, int timeout, vector<string> capture_times, string time_type)
{
	size_t num_captures = capture_times.size();
	string time_needed;
	string time_actual;
	int key_stroke = 0;
	clock_t start, end;
	unsigned long cpu_time_used;
	unsigned long time_elapsed_needed;

	if (time_type == "elapsed")
	{
		string start_loop = begin_loop_prompt();
		if (start_loop == "exit") { return; }
	}
	printf("Beginning captures ...\n");
	printf("\nPress any key at any point to stop capturing.\n");
	if (time_type == "clock")
	{
		for (int ii = 0; ii < num_captures; ii += 1)
		{
			time_needed = capture_times[ii];
			time_actual = get_date_time();

			while (time_actual != time_needed)
			{
				time_actual = get_date_time();
				if (!key_stroke) { key_stroke = check_key_hit(); }
				if (key_stroke) { break; }
			}
			if (key_stroke) { break; }
			cameras = multi_capture(cameras, timeout);
			if (!key_stroke) { key_stroke = check_key_hit(); }
			if (key_stroke) { break; }
			key_stroke = process_capture(cameras, false, extension, directory, true);
			printf("Captures so far: %i\n", ii + 1);
		}
	}
	if (time_type == "elapsed")
	{
		start = clock();
		for (int ii = 0; ii < num_captures; ii += 1)
		{
			time_elapsed_needed = convert_hhmmss_to_sec(capture_times[ii]);
			//cout << time_elapsed_needed << endl;
			end = clock();
			cpu_time_used = ((unsigned long)(end - start)) / CLOCKS_PER_SEC;

			while (cpu_time_used != time_elapsed_needed)
			{
				end = clock();
				cpu_time_used = ((unsigned long)(end - start)) / CLOCKS_PER_SEC;
				if (!key_stroke) { key_stroke = check_key_hit(); }
				if (key_stroke) { break; }
			}
			if (key_stroke) { break; }
			cameras = multi_capture(cameras, timeout);
			if (!key_stroke) { key_stroke = check_key_hit(); }
			if (key_stroke) { break; }
			key_stroke = process_capture(cameras, false, extension, directory, true);
			printf("Captures so far: %i\n", ii + 1);
		}
	}
	printf("... Captures complete!\n");
	return;
}