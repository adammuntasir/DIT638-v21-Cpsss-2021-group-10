/*
 * Copyright (C) 2020  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Include the single-file, header-only middleware libcluon to create high-performance microservices
#include "cluon-complete.hpp"
#include "cluon-complete-v0.0.127.hpp"
// Include the OpenDLV Standard Message Set that contains messages that are usually exchanged for automotive or robotic applications
#include "opendlv-standard-message-set.hpp"

// Include the GUI and image processing header files from OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <fstream> //used for file handling
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#define PI 3.1415926535
#define X_POSITION_OF_CAR 320
#define Y_POSITION_OF_CAR 480

cv::Mat getMatrix()
{

    cv::Point2f src_vertices[4];
    src_vertices[0] = cv::Point(207, 285);
    src_vertices[1] = cv::Point(364, 285);
    src_vertices[2] = cv::Point(476, 350);
    src_vertices[3] = cv::Point(89, 353);

    // How the vertices should be mapped on the destination image
    //
    //    [0]   [1]
    //
    //    [2]   [3]
    //

    cv::Point2f dst_vertices[4];

    // CLOSE PERSPECTIVE
    dst_vertices[0] = cv::Point(125, 130);
    dst_vertices[1] = cv::Point(390, 130);
    dst_vertices[2] = cv::Point(390, 395);
    dst_vertices[3] = cv::Point(125, 395);

    cv::Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
    return M;
}

std::vector<cv::Point2f> convertPoints(std::vector<cv::Point2f> coordinates)
{

    cv::Mat M = getMatrix();
    std::vector<cv::Point2f> dst_points;

    // Changing perspective to birds-eye view
    cv::perspectiveTransform(coordinates, dst_points, M);
    return dst_points;
}

int32_t main(int32_t argc, char **argv)
{
    int32_t retCode{1};
    // Parse the command line parameters as we require the user to specify some mandatory information on startup.
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ((0 == commandlineArguments.count("cid")) ||
        (0 == commandlineArguments.count("name")) ||
        (0 == commandlineArguments.count("width")) ||
        (0 == commandlineArguments.count("height")))
    {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=253 --name=img --width=640 --height=480 --verbose" << std::endl;
    }
    else
    {
        // Extract the values from the command line parameters
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid())
        {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session where network messages are exchanged.
            // The instance od4 allows you to send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            opendlv::proxy::GroundSteeringRequest gsr;
            std::mutex gsrMutex;
            auto onGroundSteeringRequest = [&gsr, &gsrMutex](cluon::data::Envelope &&env) {
                // The envelope data structure provide further details, such as sampleTimePoint as shown in this test case:
                // https://github.com/chrberger/libcluon/blob/master/libcluon/testsuites/TestEnvelopeConverter.cpp#L31-L40
                std::lock_guard<std::mutex> lck(gsrMutex);
                gsr = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
                //std::cout << "lambda: groundSteering = " << gsr.groundSteering() << std::endl;
            };

            od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);

            IplImage *iplimage{nullptr};
            CvSize size;
            size.width = WIDTH;
            size.height = HEIGHT;

            iplimage = cvCreateImageHeader(size, IPL_DEPTH_8U, 4 /* four channels: ARGB */);
            sharedMemory->lock();
            {
                iplimage->imageData = sharedMemory->data();
                iplimage->imageDataOrigin = iplimage->imageData;
            }
            sharedMemory->unlock();

            float actual_steeringAngle;

            int minH_y = 6;
            int maxH_y = 30;

            int minS_y = 51;
            int maxS_y = 235;

            int minV_y = 75;
            int maxV_y = 255;

            int minH_b = 106;
            int maxH_b = 155;

            int minS_b = 59;
            int maxS_b = 255;

            int minV_b = 29;
            int maxV_b = 255;

            int minH_b_reflection = 121;
            int maxH_b_reflection = 179;

            int minS_b_reflection = 0;
            int maxS_b_reflection = 98;

            int minV_b_reflection = 0;
            int maxV_b_reflection = 255;



            cv::Mat masked_y;
            cv::Mat masked_b;
            cv::Mat colouredImg;
            cv::Mat blueImg;
            cv::Mat blueReflectionImg;
            cv::Mat masked_reflection;
            cv::Mat masked_blueAndReflection;
            cv::Mat masked_blueWithoutReflection;
            

            float turning_correct = 0.0;
            float turning_incorrect = 0.0;
            float turning_total = 0.0;
            float straight_total = 0.0;
            float straight_correct = 0.0;
            float straight_incorrect = 0.0;
            float straight_correct_p = 0.0;
            float turning_correct_p = 0.0;
            float straight_p = 0.0;
            float turning_p = 0.0;
            float total_p = 0.0;
            float allFrames = 0.0;

            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning())
            {

                cluon::data::TimeStamp before{cluon::time::now()};
                
                // Access the latest received pedal position and lock the mutex
                {
                    std::lock_guard<std::mutex> lck(gsrMutex);
                    actual_steeringAngle = gsr.groundSteering();
                    //std::cout << "main: groundSteering = " << gsr.groundSteering() << std::endl;
                }


                // OpenCV data structure to hold an image.
                cv::Mat img;
                cv::Mat cropped;
                // Wait for a notification of a new frame.
                sharedMemory->wait();

                // Lock the shared memory.
                sharedMemory->lock();

                // CROP THE IMAGE ---------------------------

                {
                    //--------------------------------------------------------------------------------------------------------------------
                    // Copy the pixels from the shared memory into our own data structure.
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();

                    img = cv::cvarrToMat(iplimage);

                    cropped = img(cv::Rect(0, img.rows / 2, img.cols, 160));
                }
                cv::cvtColor(cropped, colouredImg, CV_BGR2HSV);

                // Colour process to detect yellow cones
                cv::inRange(colouredImg, cv::Scalar(minH_y, minS_y, minV_y),
                            cv::Scalar(maxH_y, maxS_y, maxV_y),
                            masked_y);

                // Colour process to detect blue cones
                cv::inRange(colouredImg, cv::Scalar(minH_b, minS_b, minV_b),
                            cv::Scalar(maxH_b, maxS_b, maxV_b), masked_b);

                // Colour process to detect large reflections
                cv::inRange(
                    colouredImg,
                    cv::Scalar(minH_b_reflection, minS_b_reflection, minV_b_reflection),
                    cv::Scalar(maxH_b_reflection, maxS_b_reflection,
                               maxV_b_reflection),
                    masked_reflection);

                

                // Use bitwise AND operator to find reflections in blue cone image
                cv::bitwise_and(masked_b, masked_reflection, masked_blueAndReflection);


                // Use bitwise XOR operator to remove found reflections from blue cone image
                cv::bitwise_xor(masked_blueAndReflection, masked_b, masked_blueWithoutReflection);

                std::pair<cv::Mat, cv::Mat> masked_y_b =
                    std::make_pair(masked_y, masked_blueWithoutReflection);

                // TODO: Here, you can add some code to check the sampleTimePoint when the current frame was captured.
                sharedMemory->unlock();






                // Steering angle starts here --------------------------------------

                float calculated_steeringAngle;

                // after reverse engingeering the groundsteering requests we find out that the ground steering request that are less than 80 degrees means we can send ground steering request
                if ((angleOfRoad < 90) || (angleOfRoad > 90))
                {

                    // Getting a value between 0 and 0.6 based on the angle
                    calculated_steeringAngle = ((angleOfRoad)*0.003333) - 0.3;
                }
                else
                {
                    calculated_steeringAngle = 0;
                }


                if  (actual_steeringAngle == 0)
                {
                    straight_total += 1.0;
                    if ((calculated_steeringAngle >  (actual_steeringAngle * 1.05)) || (calculated_steeringAngle <  (actual_steeringAngle * 0.95))){
                        straight_incorrect += 1.0;
                    }else{
                        straight_correct += 1.0;
                    }
                }else{
                    turning_total += 1.0;
                    if ((calculated_steeringAngle >  (actual_steeringAngle * 1.5)) || (calculated_steeringAngle <  (actual_steeringAngle * 0.5))){
                        turning_incorrect += 1.0;
                    }else{
                        turning_correct += 1.0;
                    }
                }

                straight_correct_p = (straight_correct / (straight_correct + straight_incorrect)) *100;
                turning_correct_p = (turning_correct / (turning_correct + turning_incorrect)) *100;
                straight_p = (straight_total / (turning_total + straight_total)) *100;
                turning_p = (turning_total / (turning_total + straight_total)) *100;
                total_p = ((straight_correct_p * straight_p) + (turning_correct_p * turning_p)) *100;

                std::cout << "Correct 0 is " << straight_correct_p << std::endl;
                std::cout << "Correct turn is " << turning_correct_p << std::endl;
                std::cout << "straight total is " << straight_p << std::endl;
                std::cout << "turning total is " << turning_p << std::endl;
                std::cout << "Correct total is " << total_p << std::endl;
                std::cout << "nr of frames " << turning_total + straight_total << std::endl;


                cluon::data::TimeStamp after{cluon::time::now()};

                float time_diff = cluon::time::toMicroseconds(after) - cluon::time::toMicroseconds(before);

                allFrames += time_diff;
                

                std::cout << "time diff= " << time_diff<< std::endl;
                std::cout << "time diff ave= " << allFrames / 367 << std::endl;

                
                // If you want to access the latest received ground steering, don't forget to lock the mutex:
                {
                    std::lock_guard<std::mutex> lck(gsrMutex);
                    //std::cout << "main: groundSteering = " << gsr.groundSteering() << std::endl;
                }

                // Display image on your screen.
                if (VERBOSE)
                {
                    cv::imshow(sharedMemory->name().c_str(), img);
                    cv::waitKey(1);
                }
            }

            if (nullptr != iplimage)
            {
                cvReleaseImageHeader(&iplimage);
            }
        }
        retCode = 0;
    }
    return retCode;
}
