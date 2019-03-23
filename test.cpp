// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> 
#include "example.hpp"          // Include short list of convenience functions for rendering

#include "arapaho.hpp"
#include <string>
#include <opencv2/opencv.hpp> 
#include "opencv2/core/core.hpp"
#include <sys/types.h>
#include <sys/stat.h>
#include <chrono>

// Use OpenCV for scaling the image (faster)
#define _ENABLE_OPENCV_SCALING

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
//
// Some configuration inputs
//
static char INPUT_DATA_FILE[]    = "gr.data"; 
static char INPUT_CFG_FILE[]     = "gr.cfg";
static char INPUT_WEIGHTS_FILE[] = "gr.weights";
#define MAX_OBJECTS_PER_FRAME (100)

#define TARGET_SHOW_FPS (10)

//
// Main test wrapper for arapaho
//
cv::Mat frame_to_mat(const rs2::frame& f)
{
    using namespace cv;
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
        cvtColor(r, r, COLOR_RGB2BGR);
        return r;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}
bool fileExists(const char *file) 
{
    struct stat st;
    if(!file) return false;
    int result = stat(file, &st);
    return (0 == result);
}

void arapaho(cv::Mat image, ArapahoV2* p)
{
    
    box* boxes = 0;
    
    std::string* labels;
    
    
    
    // Create arapaho`
    
    
    // Steps below this, can be performed in a loop
    
    // loop 
    // {
    //    setup arapahoImage;
    //    p->Detect(arapahoImage);
    //    p->GetBoxes;
    // }
    //
    
    // Setup image buffer here
    ArapahoV2ImageBuff arapahoImage;
    using namespace cv;
    // Detection step, only for image
    {
        int imageWidthPixels = 0, imageHeightPixels = 0;
        
        {
            imageWidthPixels = image.size().width;
            imageHeightPixels = image.size().height;
            DPRINTF("Image data = %p, w = %d, h = %d\n", image.data, imageWidthPixels, imageHeightPixels);
            
            // Remember the time
            auto detectionStartTime = std::chrono::system_clock::now();

            // Process the image
            arapahoImage.bgr = image.data;
            arapahoImage.w = imageWidthPixels;
            arapahoImage.h = imageHeightPixels;
            arapahoImage.channels = 3;
            // Using expectedW/H, can optimise scaling using HW in platforms where available
            
            int numObjects = 0;
            
            p->Detect(
                image,
                0.85,
                1,
                numObjects);
            std::chrono::duration<double> detectionTime = (std::chrono::system_clock::now() - detectionStartTime);
            
            printf("==> Detected [%d] objects in [%f] seconds\n", numObjects, detectionTime.count());
            
            if(numObjects > 0 && numObjects < MAX_OBJECTS_PER_FRAME) // Realistic maximum
            {    
                boxes = new box[numObjects];
                labels = new std::string[numObjects];
                if(!boxes)
                {
                    if(p) delete p;
                    p = 0;
                    return;
                }
                if(!labels)
                {
                    if(p) delete p;
                    p = 0;
                    if(boxes)
                    {
                        delete[] boxes;
                        boxes = NULL;                        
                    }
                    return;
                }
                
                // Get boxes and labels
                p->GetBoxes(
                    boxes,
                    labels,
                    numObjects
                    );
                
                int objId = 0;
                int leftTopX = 0, leftTopY = 0, rightBotX = 0,rightBotY = 0;
                for (objId = 0; objId < numObjects; objId++)
                {
                    leftTopX = 1 + imageWidthPixels*(boxes[objId].x - boxes[objId].w / 2);
                    leftTopY = 1 + imageHeightPixels*(boxes[objId].y - boxes[objId].h / 2);
                    rightBotX = 1 + imageWidthPixels*(boxes[objId].x + boxes[objId].w / 2);
                    rightBotY = 1 + imageHeightPixels*(boxes[objId].y + boxes[objId].h / 2);
                    DPRINTF("Box #%d: center {x,y}, box {w,h} = [%f, %f, %f, %f]\n", 
                            objId, boxes[objId].x, boxes[objId].y, boxes[objId].w, boxes[objId].h);
                    // Show image and overlay using OpenCV
                    rectangle(image,
                        cvPoint(leftTopX, leftTopY),
                        cvPoint(rightBotX, rightBotY),
                        CV_RGB(255, 0, 0), 1, 8, 0);
                    // Show labels
                    if (labels[objId].c_str())
                    {
                        DPRINTF("Label:%s\n\n", labels[objId].c_str());
                        putText(image, labels[objId].c_str(), cvPoint(leftTopX, leftTopY),
                            FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200, 200, 250), 1, CV_AA);
                    }
                }
                
                if (boxes)
                {
                    delete[] boxes;
                    boxes = NULL;
                }
                if (labels)
                {
                    delete[] labels;
                    labels = NULL;
                }   
                
            }// If objects were detected
            imshow("Arapaho", image);
            waitKey((1000 / TARGET_SHOW_FPS));
         
        } //If a frame was read
    }// Detection loop

    // Maybe clean up
    return;
}       

// Capture Example demonstrates how to
// capture depth and color video streams and render them to the screen
int main(int argc, char * argv[]) try
{
    // Early exits
    if(!fileExists(INPUT_DATA_FILE) || !fileExists(INPUT_CFG_FILE) || !fileExists(INPUT_WEIGHTS_FILE))
    {
        EPRINTF("Setup failed as input files do not exist or not readable!\n");
        return -1;       
    }
    
    ArapahoV2Params ap;
    ap.datacfg = INPUT_DATA_FILE;
    ap.cfgfile = INPUT_CFG_FILE;
    ap.weightfile = INPUT_WEIGHTS_FILE;
    ap.nms = 0.4;
    ap.maxClasses = 5;

    ArapahoV2* p = new ArapahoV2();
    if(!p)
    {
        return -1;
    }
    
    // TODO - read from arapaho.cfg    
    int expectedW = 0, expectedH = 0;
    bool ret = false;
    // Always setup before detect
    ret = p->Setup(ap, expectedW, expectedH);
    if(false == ret)
    {
        EPRINTF("Setup failed!\n");
        if(p) delete p;
        p = 0;
        return -1;
    }
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    // Create a simple OpenGL window for rendering:
    //window app(720, 480, "RealSense Capture Example");
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    // Declare rates printer for showing streaming rates of the enabled streams.
    rs2::rates_printer printer;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
     rs2::config cfg;
    //cfg.enable_stream(RS2_STREAM_COLOR); //this will cause seg fault in Mat(...)
    cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGR8);
    rs2::pipeline_profile pp = pipe.start(cfg);
    rs2::stream_profile sp = pp.get_stream(RS2_STREAM_COLOR);
    // Start streaming with default recommended configuration
    // The default video configuration contains Depth and Color streams
    // If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default
    //pipe.start();
    using namespace cv;
	//const auto window_name = "Display Image";
    //namedWindow(window_name, WINDOW_AUTOSIZE);
	while (waitKey(1) < 0) // Application still alive?
    {
        rs2::frameset data = pipe.wait_for_frames();  
        rs2::video_frame color = data.get_color_frame();
		//rs2_stream align_to = find_stream_to_align(profile.get_streams());
		//rs2::align align(align_to);
		//auto processed = align.process(data);
		//rs2::video_frame color_frame = processed.first(align_to);
		//rs2::depth_frame depth_frame = processed.get_depth_frame();
        const int w = color.get_width();
        const int h = color.get_height();

        // Create OpenCV matrix of size (w,h) from the colorized color data
        Mat image(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

            resize(image, image, cv::Size(), 0.75, 0.75);
		//const int w = data.as<rs2::video_frame>().get_width();
		//const int h = data.as<rs2::video_frame>().get_height();
		
        // Query frame size (width and height

        // Create OpenCV matrix of size (w,h) from the colorized depth data
       // Mat img(Size(w, h),0, (void*)color.get_data(), Mat::AUTO_STEP);
				// Create OpenCV matrix of size (w,h) from the colorized depth data
		//Mat img(cv::Size(720, 480), CV_8UC3, (void*)data.get_data(), cv::Mat::AUTO_STEP);
		
        // The show method, when applied on frameset, break it to frames and upload each frame into a gl textures
        // Each texture is displayed on different viewport according to it's stream unique id
       //	app.show(data);
        // imshow(window_name, frame_to_mat(color));
		// arapaho(frame_to_mat(color), p);
        //imshow(window_name, image);
		arapaho(image, p);
    }

    
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
	//Given a vector of streams, we try to find a depth stream and another stream to align depth with.
	//We prioritize color streams to make the view look better.
	//If color is not available, we take another stream that (other than depth)
	rs2_stream align_to = RS2_STREAM_ANY;
	bool depth_stream_found = false;
	bool color_stream_found = false;
	for (rs2::stream_profile sp : streams)
	{
		rs2_stream profile_stream = sp.stream_type();
		if (profile_stream != RS2_STREAM_DEPTH)
		{
			if (!color_stream_found)         //Prefer color
				align_to = profile_stream;

			if (profile_stream == RS2_STREAM_COLOR)
			{
				color_stream_found = true;
			}
		}
		else
		{
			depth_stream_found = true;
		}
	}

	if (!depth_stream_found)
		throw std::runtime_error("No Depth stream available");

	if (align_to == RS2_STREAM_ANY)
		throw std::runtime_error("No stream found to align with Depth");

	return align_to;
}
