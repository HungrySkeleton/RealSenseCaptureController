// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <iostream>
#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include "Header Files/InputEngine.h"
#include <map>
#include <chrono>
#include <mutex>
#include <thread>
#include <fstream>              // File IO
#include <iostream>             // Terminal IO
#include <sstream>              // Stringstreams
#include <iomanip>


// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "Header Files/stb_image_write.h"
//#include "Header Files/CaptureEngine.h"
#include "Header Files/CaptureEngineRealSense.h"
#include "Header Files/InputEngineRealSense.h"

// This example assumes camera with depth and color
// streams, and direction lets you define the target stream
enum class direction
{
    to_depth,
    to_color
}; 


int main() 
{
    try 
    {
        string version = "0.1.1"; // Version String 
        print_separator(); // Just Prints a Line Separator
        welcome_message(version); //Welcome Message

        rs2::pipeline pipe; //Create the pipeline
        rs2::config cfg; //Create a Configuration Scheme
        cfg.enable_all_streams(); //All Streams Enabled
        pipe.start(cfg); // Start

        
        //Obtain Pipeline Profile
        rs2::pipeline_profile pipeline_prof = pipe.get_active_profile();
        //Get a list of all the streams
        std::vector<rs2::stream_profile> stream_profile_list = pipeline_prof.get_streams();
        
        //Modify this to iterate for a multi cam setup to write the intrinsics.
        rs2::stream_profile color_stream = pipeline_prof.get_stream(RS2_STREAM_COLOR);
        rs2::stream_profile depth_stream = pipeline_prof.get_stream(RS2_STREAM_DEPTH);

        //Print ALL ACTIVE STREAMS
        for (auto x : stream_profile_list) {
            cout << x.stream_name() << endl;
        }

        //rs2::device device = get_a_realsense_device();
        //rs2::sensor sensor = get_a_sensor_from_a_device(device);
        //show_stream_intrinsics(device,sensor);
        print_stream_intrinsics(color_stream);
        print_stream_intrinsics(depth_stream);
        print_separator();
        rs2::device_list device_list = get_all_realsense_devices();
        print_separator();
        string main_directory = get_directory("save");
        int created_subdirectories = create_subdirectories(main_directory, device_list);
        print_separator();
        write_extrinsics(main_directory, color_stream, depth_stream);
        get_extrinsics(color_stream, depth_stream);
        int captureIntervalTime = get_sleep_time();
        int captureCount = 0;
        // Create a pipeline to easily configure and start the camera
        //string path = "C://Users//tranr//OneDrive//Desktop//Sample Real Sense Blank Project//Images//";
        string path = main_directory + "images" + "/";

        rs2::colorizer colorize_map; // Helper to colorize depth images
        // Define two align objects. One will be used to align
        // to depth viewport and the other to color.
        // Creating align object is an expensive operation
        // that should not be performed in the main loop
        rs2::align align_to_depth(RS2_STREAM_DEPTH);
        rs2::align align_to_color(RS2_STREAM_COLOR);

        direction   dir = direction::to_color;  // Alignment direction
        
        begin_loop_prompt();


        while (true) // Application still alive?
        {
            // Capture 30 frames to give autoexposure, etc. a chance to settle
            for (auto i = 0; i < 30; ++i) pipe.wait_for_frames();

            // Using the align object, we block the application until a frameset is available
            rs2::frameset frameset = pipe.wait_for_frames();
            string timestampString = get_date_time();
            frameset = align_to_color.process(frameset);
            for (auto&& frame : frameset)
            {
                auto depth = frameset.get_depth_frame();
                auto color = frameset.get_color_frame();
                auto colorized_depth = colorize_map.colorize(depth);

                // We can only save video frames as pngs, so we skip the rest
                // Get the serial number of the current frame's device
                auto serial = rs2::sensor_from_frame(frame)->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                if (auto vf = frame.as<rs2::video_frame>())
                {
                    //Write out Colorized Depth Images as well
                    if (vf.is<rs2::depth_frame>()) {

                        // Write images to disk
                        std::stringstream png_file;
                        auto vf1 = colorized_depth;
                        png_file << path << serial << "/" << timestampString << "_" <<"Colorized_" << vf1.get_profile().stream_name() << ".png";
                        stbi_write_png(png_file.str().c_str(), vf1.get_width(), vf1.get_height(),
                            vf1.get_bytes_per_pixel(), vf1.get_data(), vf1.get_stride_in_bytes());
                        std::cout << "Saved " << png_file.str() << std::endl;
                    } 
                    
                    // Write images to disk
                    std::stringstream png_file;
                    png_file << path << serial << "/" << timestampString << "_" << vf.get_profile().stream_name() << ".png";
                    stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(),
                        vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
                    std::cout << "Saved " << png_file.str() << std::endl;

                    // Record per-frame metadata for UVC streams
                    std::stringstream csv_file;
                    csv_file << path << serial << "/" << "_" << vf.get_profile().stream_name()
                        << "-metadata.csv";
                    metadata_to_csv(vf, csv_file.str());
                }
            }
            captureCount += 1;
            printf("Captures so far: %i, taken at %s\n", captureCount, timestampString);
            std::this_thread::sleep_for(std::chrono::seconds(captureIntervalTime));
        }

        return EXIT_SUCCESS;
    }
    catch (const rs2::error& e)
    {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }
} 







