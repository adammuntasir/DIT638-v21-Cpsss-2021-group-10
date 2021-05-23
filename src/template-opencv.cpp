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


//below code is inspired by the website: https://janhalozan.com/2019/06/01/lane-detector/
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

    cv::Mat M = getPerspectiveTransform(src_vertices, dst_vertices); //formulates a ratio for transfomation of every pixel or point
    return M;
}


//below code is inspired by the website: https://janhalozan.com/2019/06/01/lane-detector/
std::vector<cv::Point2f> convertPoints(std::vector<cv::Point2f> coordinates)
{

    cv::Mat M = getMatrix();
    std::vector<cv::Point2f> dst_points;

    // Changing perspective to birds-eye view
    cv::perspectiveTransform(coordinates, dst_points, M); //transforms every point based on the ratio declared)
    return dst_points;
}

double calculateAngleOfRoad(std::vector<cv::Point2f> cones_y_b){

                    //to get the cone closes to the car
                    std::sort(cones_y_b.begin(), cones_y_b.end(),
                              [](const cv::Point2f &a, const cv::Point2f &b) {
                                  return a.y > b.y; // Sorting in order from highest to lowest of the y values
                              });

                    //uses pythagoras to get the straight distance of the car and the closest cone
                    double distance = std::sqrt(std::pow(cones_y_b[0].x - X_POSITION_OF_CAR, 2) +
                                               std::pow(cones_y_b[0].y - Y_POSITION_OF_CAR, 2) * 1.0);

                    double angle;
                    //when the car is close enough to the cone, then we make the turns
                    if (distance < 200)
                    {
                        // code below inspired from : https://www.cplusplus.com/reference/cmath/atan2/

                        // Getting the angle in radians(angle the cones make with the positive x axis)
                        double radians =
                            std::atan2(cones_y_b[0].y - cones_y_b[1].y, cones_y_b[0].x - cones_y_b[1].x);

                        // Converting to degrees
                        double degrees = radians * 180 / PI;

                        // Adjusting so the value is measured from right x axis(if < 90, we need a higher angle to go right; if > 90, we need a lower angle to go left(negative))
                        double adjusted = 180 - degrees;

                        if (adjusted > 60 && adjusted < 120)
                        {
                            angle = 90;
                        }
                        else
                        {
                            angle = adjusted;
                        }
                    }
                    else
                    {
                        angle = 90;
                    }

                    return angle;
}

double calculateSteeringAngle(std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> cones_y_b){
                double angleOfRoad = 0.0;
                double yellowAngle = 0.0;
                double blueAngle = 0.0;

                // If there are at least 2 blue and 2 yellow cones
                if ((cones_y_b.first.size() >= 2) && (cones_y_b.second.size() >= 2))
                {

                
                    //-------------------yellow  angle


                    yellowAngle = calculateAngleOfRoad(cones_y_b.first);

                    //-------------------blue angle

                    
                    blueAngle = calculateAngleOfRoad(cones_y_b.second);

                    //----------------find mean

                    // Taking the mean value of the found angles
                    double meanAngle = (yellowAngle + blueAngle) / 2;
                    angleOfRoad = meanAngle;
                }
                else if (cones_y_b.first.size() >= 2)
                {

                    //------------yellow is angle now

                    yellowAngle = calculateAngleOfRoad(cones_y_b.first);

                    angleOfRoad = yellowAngle;
                }
                else if (cones_y_b.second.size() >= 2)
                {

                    blueAngle = calculateAngleOfRoad(cones_y_b.second);

                    angleOfRoad = blueAngle;
                }
                else
                {

                    angleOfRoad = 90;
                }

                //--------------------------------------------------this is the end of calculating angles--------------------------------

                // Steering angle starts here --------------------------------------

                double calculated_steeringAngle;

                // after reverse engingeering the groundsteering requests we find out that the ground steering request that are less than 90 degrees means we can send ground steering request
                if ((angleOfRoad < 90) || (angleOfRoad > 90))
                {

                    // from angle 0 to angle 180 (which is the extreme road angles) corresponds to a steering range between  -0.3  and 0.3 which is a total of 0.6
                    // 0.6 / 180  = 0.0033333
                    // the acceptable range of steering angle is a maximum of 0.3 thats why we subtract 0.3 
                    // ( road angle * 0.003333 ) gives all the steering requests possible, and we subtract by 0.3 to keep it within the range
                    calculated_steeringAngle = ((angleOfRoad)*0.003333) - 0.3;
                }
                else
                {
                    calculated_steeringAngle = 0.0;
                }

                return calculated_steeringAngle;
}



//below code is inspired by the website: https://medium.com/@devanshvarshney/object-detection-and-tracking-using-opencv-4f68aa41dd3a
//and : https://learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
std::vector<cv::Point2f> drawContours(cv::Mat birdEyeView, cv::Mat reducedImg){
    
                std::vector<std::vector<cv::Point>> contours;
                std::vector<cv::Vec4i> hierarchy;
                cv::findContours(reducedImg, contours, hierarchy, CV_RETR_TREE,
                                 CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

                // Get the moments
                std::vector<cv::Moments> mu(contours.size());
                for (unsigned int i = 0; i < contours.size(); i++)
                {
                    mu[i] = moments(contours[i], false);
                }

                // Get the mass centers:
                std::vector<cv::Point2f> mc(contours.size());
                std::vector<cv::Point2f> mc_transformed(contours.size());
                for (unsigned int i = 0; i < contours.size(); i++)
                {
                    mc[i] = cv::Point2f(float(mu[i].m10 / mu[i].m00),
                                            float(mu[i].m01 / mu[i].m00));
                    mc[i].y = mc[i].y + 240;

                    mc_transformed = convertPoints(mc); //transforms every point based on the ratio declared)
                }


                // Draw contours
                for (unsigned int i = 0; i < contours.size(); i++)
                {
                    cv::circle(birdEyeView, mc_transformed[i], 10, cv::Scalar(0, 0, 255), 1,
                               CV_AA, 0);
                }

                return mc_transformed;
}



//below code is inspired by the website: https://docs.opencv.org/3.4/db/df6/tutorial_erosion_dilatation.html
std::pair<cv::Mat, cv::Mat> reduceImage(std::pair<cv::Mat, cv::Mat> masked_y_b){

    std::pair<cv::Mat, cv::Mat> reducedImg;

                cv::dilate(masked_y_b.first, reducedImg.first, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
                cv::erode(reducedImg.first, reducedImg.first, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6, 6)));

                cv::dilate(masked_y_b.second, reducedImg.second, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7)));


                //cv::Mat combined_masks;
                //cv::Mat resultAfterBlur;

                //cv::bitwise_or(reducedImg.second, reducedImg.first, combined_masks);

                cv::GaussianBlur(reducedImg.first, reducedImg.first, cv::Size(15, 15), 0);
                cv::GaussianBlur(reducedImg.second, reducedImg.second, cv::Size(15, 15), 0);

                return reducedImg;

}


std::pair<cv::Mat, cv::Mat> maskCones(cv::Mat cropped){
    

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

                std::pair<cv::Mat, cv::Mat> masked_y_b = std::make_pair(masked_y, masked_blueWithoutReflection);

                return masked_y_b;

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



            double actual_steeringAngle;

            double turning_correct = 0.0;
            double turning_incorrect = 0.0;
            double turning_total = 0.0;
            double straight_total = 0.0;
            double straight_correct = 0.0;
            double straight_incorrect = 0.0;
            double straight_correct_p = 0.0;
            double turning_correct_p = 0.0;
            double straight_p = 0.0;
            double turning_p = 0.0;
            double total_p = 0.0;
            double allFrames = 0.0;

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

                std::string stringTimeSample;
                stringTimeSample.append(std::to_string(cluon::time::toMicroseconds(sharedMemory->getTimeStamp().second)));

                // CROP THE IMAGE ---------------------------

                {
                    //--------------------------------------------------------------------------------------------------------------------
                    // Copy the pixels from the shared memory into our own data structure.
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();

                    cropped = img(cv::Rect(0, img.rows / 2, img.cols, 120));
                }
                sharedMemory->unlock();

                std::pair<cv::Mat, cv::Mat> masked_y_b = maskCones(cropped);
    

                // DETECT CONES ---------------------------

                std::pair<cv::Mat, cv::Mat> reducedImg = reduceImage(masked_y_b);

                cv::Mat dst(480, 640, CV_8UC3);

                cv::Mat original = img.clone();

                // Calculates a matrix of a perspective transform(transforms every pixel based on the ratio declared)

                cv::warpPerspective(original, dst, getMatrix(), dst.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);

                cv::Mat birdEyeView = dst;

                std::vector<cv::Point2f> mc_transformed_yel = drawContours(birdEyeView, reducedImg.first);
                std::vector<cv::Point2f> mc_transformed_blue = drawContours(birdEyeView, reducedImg.second);


                std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> contours_y_b =
                    std::make_pair(mc_transformed_yel, mc_transformed_blue);

                std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>>
                    cones_y_b = contours_y_b;

                //--------------------------------------------------this is the start of calculating angles --------------------------------

                double calculated_steeringAngle = calculateSteeringAngle(cones_y_b);

                if (actual_steeringAngle >= 0 && actual_steeringAngle <= 0)
                {
                    straight_total += 1.0;
                    if ((calculated_steeringAngle > (actual_steeringAngle * 1.05)) || (calculated_steeringAngle < (actual_steeringAngle * 0.95)))
                    {
                        straight_incorrect += 1.0;
                    }
                    else
                    {
                        straight_correct += 1.0;
                    }
                }
                else
                {
                    turning_total += 1.0;
                    if ((calculated_steeringAngle > (actual_steeringAngle * 1.5)) || (calculated_steeringAngle < (actual_steeringAngle * 0.5)))
                    {
                        turning_incorrect += 1.0;
                    }
                    else
                    {
                        turning_correct += 1.0;
                    }
                }

                straight_correct_p = (straight_correct / (straight_correct + straight_incorrect)) * 100;
                turning_correct_p = (turning_correct / (turning_correct + turning_incorrect)) * 100;
                straight_p = (straight_total / (turning_total + straight_total)) * 100;
                turning_p = (turning_total / (turning_total + straight_total)) * 100;
                total_p = ((straight_correct_p * straight_p) + (turning_correct_p * turning_p)) / 100;
                
                //std::cout << "Correct 0 is " << straight_correct_p << std::endl;
                //std::cout << "Correct turn is " << turning_correct_p << std::endl;
                //std::cout << "straight total is " << straight_p << std::endl;
                //std::cout << "turning total is " << turning_p << std::endl;
                //std::cout << "Correct total is " << total_p << std::endl;
                //std::cout << "nr of frames " << turning_total + straight_total << std::endl;
                
                cluon::data::TimeStamp after{cluon::time::now()};

                double time_diff = cluon::time::toMicroseconds(after) - cluon::time::toMicroseconds(before);

                allFrames += time_diff;

                //std::cout << "time diff= " << time_diff << std::endl;
                //std::cout << "time diff ave= " << allFrames / (turning_total + straight_total) << std::endl;

                std::cout << "group_10;" << stringTimeSample << ";" << calculated_steeringAngle << std::endl;

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

        }
        retCode = 0;
    }
    return retCode;
}
