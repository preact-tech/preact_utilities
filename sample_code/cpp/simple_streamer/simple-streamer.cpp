/**
 * @file simple-streamer.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that uses libtofcore to stream Distance Apmplitude data
 */
#include "tofcore/tof_sensor.hpp"
#include "tofcore/cartesian_transform.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <iomanip>
#include <thread>
#include <boost/program_options.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "open3d/Open3D.h"
#include <Eigen/Dense>

using namespace std::chrono_literals;
using namespace std::chrono;
using namespace tofcore;

static uint32_t baudRate{DEFAULT_BAUD_RATE};
static std::string devicePort{DEFAULT_PORT_NAME};
static volatile bool exitRequested{false};
static uint16_t protocolVersion{DEFAULT_PROTOCOL_VERSION};
static uint16_t minAmplitude{0};

tofcore::CartesianTransform cartesianTransform_;
open3d::visualization::Visualizer vis;


static void measurement_callback(std::shared_ptr<tofcore::Measurement_T> pData)
{

    using DataType = tofcore::Measurement_T::DataType;

    if (pData->type() == DataType::DISTANCE_AMPLITUDE)
    {
        std::vector<Eigen::Vector3d> points = std::vector<Eigen::Vector3d>();
        std::vector<Eigen::Vector3d> look_at = std::vector<Eigen::Vector3d>();

        cv::Mat dist_frame_resized;
        cv::Mat amp_frame_resized;
        // Do filtering on this opencv Mat
        cv::Mat dist_frame = cv::Mat(pData->height(), pData->width(), CV_16UC1, (void *)pData->distance().begin());
        cv::resize(dist_frame, dist_frame_resized, cv::Size(dist_frame.cols * 3, dist_frame.rows * 3));
        dist_frame_resized *= 5; // add some gain to make image look better
        cv::imshow("Distance Image", dist_frame_resized);
        cv::waitKey(1);
        cv::Mat amp_frame = cv::Mat(pData->height(), pData->width(), CV_16UC1, (void *)pData->amplitude().begin());
        cv::resize(amp_frame, amp_frame_resized, cv::Size(amp_frame.cols * 3, amp_frame.rows * 3));
        amp_frame_resized *= 50; // add some gain to make image look better
        cv::imshow("Amplitude Image", amp_frame_resized);
        cv::waitKey(1);
        //cv::medianBlur(dist_frame, dist_frame, 3);
        cv::Mat src = cv::Mat::zeros(dist_frame.size(), CV_32FC1);
        cv::Mat dst = cv::Mat::zeros(dist_frame.size(), CV_32FC1);
        dist_frame.convertTo(src, CV_32FC1);
        cv::bilateralFilter(src, dst, 9, 75,75);
        dst.convertTo(dist_frame, CV_16UC1);
        uint16_t *it_d = (unsigned short *)dist_frame.datastart;
        uint16_t *it_a = (unsigned short *)pData->amplitude().begin();
        int count = 0;
        double px, py, pz;
        uint16_t distance;
        uint16_t y;
        uint16_t x;
        int valid = 0;
        while (it_d != (const unsigned short *)dist_frame.dataend)
        {
            if (*it_a < minAmplitude)
            {
                ++count;
                it_d += 1;
                it_a += 1;
                continue;
            }
            distance = *it_d;
            y = count / pData->width();
            x = count % pData->width();
            valid = 0;
            px = py = pz = 0.1;
            cartesianTransform_.transformPixel(x, y, distance, px, py, pz);
            px /= 1000.0; // mm -> m
            py /= 1000.0; // mm -> m
            pz /= 1000.0; // mm -> m
            auto tmp = Eigen::Vector3d(pz, px, py);
            points.push_back(tmp);
            ++count;
            it_d += 1;
            it_a += 1;
        }
        open3d::geometry::PointCloud point_cloud = open3d::geometry::PointCloud(points);
        std::shared_ptr<open3d::geometry::PointCloud> cloud_ptr = std::make_shared<open3d::geometry::PointCloud>(point_cloud);
        // vis.RemoveGeometry();

        vis.ClearGeometries();
        vis.AddGeometry(cloud_ptr);
        //0.003 [radian/pixel] * (180/pi) [degrees/radian] = 0.1719 [degrees/pixel] OR 5.8178 [pixels/degree]
        //Taking 360 [degrees/rotation] * 5.8178 [pixels/degree] = 2094.3951 [pixels/rotation]
        vis.GetViewControl().Rotate(523.6 * 3, 523.6 * 3, 0, 0);
        vis.GetViewControl().SetZoom(0.25);
        vis.GetRenderOption().point_color_option_ = open3d::visualization::RenderOption::PointColorOption::YCoordinate;
        vis.GetRenderOption().point_size_=3;
        vis.PollEvents();
        vis.UpdateRender();
        vis.UpdateGeometry();
        dist_frame.release();
        dist_frame_resized.release();
        amp_frame.release();
        amp_frame_resized.release();
    }
}
namespace po = boost::program_options;

static void parseArgs(int argc, char *argv[])
{
    po::options_description desc("Simple Streaming Example");
    desc.add_options()("help,h", "produce help message")("device-uri,p", po::value<std::string>(&devicePort)->default_value(devicePort))("protocol-version,v", po::value<uint16_t>(&protocolVersion)->default_value(DEFAULT_PROTOCOL_VERSION))("min-amplitude,m", po::value<uint16_t>(&minAmplitude)->default_value(0));

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        exit(0);
    }
}

static void signalHandler(int signum)
{
    (void)signum;
    exitRequested = true;
}

int main(int argc, char *argv[])
{
    parseArgs(argc, argv);
    /*
     * Change default action of ^C, ^\ from abnormal termination in order to
     * perform a controlled shutdown.
     */
    signal(SIGINT, signalHandler);
#if defined(SIGQUIT)
    signal(SIGQUIT, signalHandler);
#endif
    {
        tofcore::Sensor sensor{protocolVersion, devicePort, baudRate};
        std::vector<double> rays_x, rays_y, rays_z;
        try
        {
            sensor.getLensInfo(rays_x, rays_y, rays_z);
            cartesianTransform_.initLensTransform(320, 240, rays_x, rays_y, rays_z);
        }
        catch (std::exception &e)
        {
            std::cout << "Error retreiving lens info..." << std::endl;
            return -1;
        }
        vis.CreateVisualizerWindow("Open3D", 1500, 1000, 75, 50, true);
        vis.PollEvents();
        vis.UpdateRender();

        sensor.subscribeMeasurement(&measurement_callback); // callback is called from background thread
        sensor.setIntegrationTime(3000);
        sensor.streamDistanceAmplitude();

        while (!exitRequested) // wait for ^\ or ^C
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::cout << "Shutting down..." << std::endl;
        sensor.stopStream();
    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
