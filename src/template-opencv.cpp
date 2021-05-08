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
                std::cout << "lambda: groundSteering = " << gsr.groundSteering() << std::endl;
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

            /*
            int BlueminH{110};
            int BluemaxH{135};
            int BlueminS{70};
            int BluemaxS{255};
            int BlueminV{47};
            int BluemaxV{255};
*/
            int BlueminH{111};
            int BluemaxH{127};
            int BlueminS{96};
            int BluemaxS{255};
            int BlueminV{40};
            int BluemaxV{255};
            /*
            int YellowminH{6};
            int YellowmaxH{30};
            int YellowminS{51};
            int YellowmaxS{255};
            int YellowminV{75};
            int YellowmaxV{255};
*/
            int YellowminH{10};
            int YellowmaxH{29};
            int YellowminS{58};
            int YellowmaxS{159};
            int YellowminV{137};
            int YellowmaxV{255};

            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning())
            {
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

                    cropped = img(cv::Rect(0, img.rows / 2, img.cols, img.rows / 2));
                }
                // TODO: Here, you can add some code to check the sampleTimePoint when the current frame was captured.
                sharedMemory->unlock();

                // TODO: Do something with the frame.
                // Example: Draw a red rectangle and display image.
                //cv::rectangle(img, cv::Point(50, 50), cv::Point(100, 100), cv::Scalar(0,0,255));

                cv::Mat imgHSV;
                cvtColor(cropped, imgHSV, cv::COLOR_BGR2HSV);

                // DETECT CONES ---------------------------

                // processing to detect blue cones
                cv::Mat BLueimgColorSpace;
                cv::inRange(imgHSV, cv::Scalar(BlueminH, BlueminS, BlueminV), cv::Scalar(BluemaxH, BluemaxS, BluemaxV), BLueimgColorSpace);

                // processing to detect yellow cones
                cv::Mat YellowimgColorSpace;
                cv::inRange(imgHSV, cv::Scalar(YellowminH, YellowminS, YellowminV), cv::Scalar(YellowmaxH, YellowmaxS, YellowmaxV), YellowimgColorSpace);

                // merging the two masks into the cropped video
                cv::Mat combined;
                cv::bitwise_or(BLueimgColorSpace, YellowimgColorSpace, combined);
                //std::pair<cv::Mat, cv::Mat> pairs = stid::make_pair(YellowimgColorSpace, BLueimgColorSpace);

                // ERODE AND DILATE FROM THIS POINT ---------------------------

                cv::Mat afterErosion;
                cv::Mat afterDilation;

                cv::dilate(combined, afterDilation, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
                cv::erode(afterDilation, afterErosion, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8, 8)));

                cv::imshow("Color-Space Image", afterErosion);

                // If you want to access the latest received ground steering, don't forget to lock the mutex:
                {
                    std::lock_guard<std::mutex> lck(gsrMutex);
                    std::cout << "main: groundSteering = " << gsr.groundSteering() << std::endl;
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
