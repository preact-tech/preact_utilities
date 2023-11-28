/**
 * @file simple-streamer.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that uses libtofcore to stream DCS or DCS+Ambient data
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
static bool captureAmxxx{false};
static bool captureDistance{false};
static std::string devicePort{DEFAULT_PORT_NAME};
static volatile bool exitRequested{false};
static uint16_t protocolVersion{DEFAULT_PROTOCOL_VERSION};
static size_t verbosity{0};

static std::atomic<uint32_t> amplitudeCount;
static std::atomic<uint32_t> dcsCount;
static std::atomic<uint32_t> distanceCount;
static std::atomic<uint32_t> grayscaleCount;
tofcore::CartesianTransform cartesianTransform_;
open3d::visualization::Visualizer vis;

bool first = true;

static void measurement_callback(std::shared_ptr<tofcore::Measurement_T> pData)
{
    cv::Mat dist_frame;
    cv::Mat dist_frame_resized;
    cv::Mat amp_frame;
    cv::Mat amp_frame_resized;

    double *x_out;
    double *y_out;
    double *z_out;
    uint16_t *it_d;
    uint16_t *it_a;
    int count = 0;
    uint16_t distance;
    uint16_t y;
    uint16_t x;
    int valid = 0;
    double px, py, pz;
    std::vector<Eigen::Vector3d> points = std::vector<Eigen::Vector3d>();
    std::vector<Eigen::Vector3d> look_at = std::vector<Eigen::Vector3d>();
    open3d::geometry::PointCloud point_cloud;
    std::shared_ptr<open3d::geometry::PointCloud> cloud_ptr;
    using DataType = tofcore::Measurement_T::DataType;
    switch (pData->type())
    {
    case DataType::DISTANCE_AMPLITUDE:
        ++amplitudeCount;
        ++distanceCount;
        if (verbosity > 0)
        {
            std::cout << "received DISTANCE-AMPLITUDE measurement data, packet size "
                      << (pData->pixel_buffer().size()) << std::endl;
        }
        // Do filtering on this opencv Mat
        dist_frame = cv::Mat(pData->height(), pData->width(), CV_16UC1, (void *)pData->distance().begin());
        cv::resize(dist_frame, dist_frame_resized, cv::Size(dist_frame.cols * 4, dist_frame.rows * 4));
        dist_frame_resized *= 5; // add some gain to make image look better
        cv::imshow("Distance Image", dist_frame_resized);
        cv::waitKey(1);
        amp_frame = cv::Mat(pData->height(), pData->width(), CV_16UC1, (void *)pData->amplitude().begin());
        cv::resize(amp_frame, amp_frame_resized, cv::Size(amp_frame.cols * 4, amp_frame.rows * 4));
        amp_frame_resized *= 50; // add some gain to make image look better
        cv::imshow("Amplitude Image", amp_frame_resized);
        cv::waitKey(1);
        it_d = (unsigned short *)dist_frame.datastart;
        it_a = (unsigned short *)pData->amplitude().begin();
        count = 0;
        points.clear();
        while (it_d != (const unsigned short *)dist_frame.dataend)
        {
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
        }
        point_cloud = open3d::geometry::PointCloud(points);
        cloud_ptr = std::make_shared<open3d::geometry::PointCloud>(point_cloud);
        // vis.RemoveGeometry();

        vis.ClearGeometries();
        vis.AddGeometry(cloud_ptr);
        vis.GetViewControl().Rotate(523.6*3,523.6*3,0,0);
        vis.GetViewControl().SetZoom(0.35);
        vis.GetRenderOption().point_color_option_=open3d::visualization::RenderOption::PointColorOption::YCoordinate;
        vis.PollEvents();
        vis.UpdateRender();
        vis.UpdateGeometry();
        dist_frame.release();
        dist_frame_resized.release();
        amp_frame.release();
        amp_frame_resized.release();
        break;
    case DataType::DCS:
        ++dcsCount;
        if (verbosity > 0)
        {
            std::cout << "received DCS measurement data, packet size " << pData->pixel_buffer().size() << std::endl;
        }
        break;
    case DataType::GRAYSCALE:
        ++grayscaleCount;
        if (verbosity > 0)
        {
            std::cout << "received GRAYSCALE measurement data, packet size " << pData->pixel_buffer().size() << std::endl;
        }
        break;
    case DataType::DISTANCE:
        ++distanceCount;
        if (verbosity > 0)
        {
            std::cout << "received DISTANCE measurement data, packet size " << pData->pixel_buffer().size() << std::endl;
        }
        break;
    case DataType::AMPLITUDE:
        ++amplitudeCount;
        if (verbosity > 0)
        {
            std::cout << "received AMPLITUDE measurement data, packet size "
                      << (pData->pixel_buffer().size()) << std::endl;
        }
        break;
    case DataType::AMBIENT:
        ++amplitudeCount;
        if (verbosity > 0)
        {
            std::cout << "received AMBIENT measurement data, packet size "
                      << (pData->pixel_buffer().size()) << std::endl;
        }
        break;

    default:
        std::cout << "UNRECOGNIZED data type: " << static_cast<int16_t>(pData->type()) << std::endl;
    }
    
}

namespace po = boost::program_options;

class CountValue : public po::typed_value<std::size_t>
{
public:
    CountValue() : CountValue(nullptr)
    {
    }

    CountValue(std::size_t *store) : po::typed_value<std::size_t>(store)
    {
        // Ensure that no tokens may be passed as a value.
        default_value(0);
        zero_tokens();
    }

    virtual ~CountValue()
    {
    }

    virtual void xparse(boost::any &store, const std::vector<std::string> & /*tokens*/) const
    {
        // Replace the stored value with the access count.
        store = boost::any(++count_);
    }

private:
    mutable std::size_t count_{0};
};

static void parseArgs(int argc, char *argv[])
{
    po::options_description desc("illuminator board test");
    desc.add_options()("help,h", "produce help message")("device-uri,p", po::value<std::string>(&devicePort)->default_value(devicePort))("protocol-version,v", po::value<uint16_t>(&protocolVersion)->default_value(DEFAULT_PROTOCOL_VERSION))("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))("amplitude,a", po::bool_switch(&captureAmxxx), "Capture DCS+Ambient or Distance Amplitude frames, (not just DCS or Distance)")("ambient", po::bool_switch(&captureAmxxx), "Capture DCS+Ambient or Distance Amplitude frames, (not just DCS or Distance)")("distance,d", po::bool_switch(&captureDistance), "Capture distance (or amplitude) frames instead of DCS frames")("verbose,V",
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           new CountValue(&verbosity),
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           "Increase verbosity of output");

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
        std::cout << devicePort << std::endl;
        std::vector<double> rays_x, rays_y, rays_z;
        sensor.getLensInfo(rays_x, rays_y, rays_z);
        try
        {
            cartesianTransform_.initLensTransform(320, 240, rays_x, rays_y, rays_z);
        }
        catch (std::exception &e)
        {
            std::cout << "Error retreiving lens info..." << std::endl;
            return -1;
        }
        vis.CreateVisualizerWindow("Open3D",1480,1080,50,50,true);
        vis.PollEvents();
        vis.UpdateRender();

        sensor.subscribeMeasurement(&measurement_callback); // callback is called from background thread
        sensor.setIntegrationTime(4000);
        sensor.streamDistanceAmplitude();

        auto lastTime = steady_clock::now();
        while (!exitRequested) // wait for ^\ or ^C
        {
            std::this_thread::sleep_until(lastTime + 1000ms);
            lastTime = steady_clock::now();
            const uint32_t amplitude{amplitudeCount};
            amplitudeCount = 0;
            const uint32_t dcs{4 * dcsCount};
            dcsCount = 0;
            const uint32_t distance{distanceCount};
            distanceCount = 0;
            const uint32_t grayscale{grayscaleCount};
            grayscaleCount = 0;
            std::cout << "RAW FPS: amplitude = " << amplitude << "; dcs = " << dcs
                      << "; distance = " << distance << "; grayscale = " << grayscale
                      << "; total = " << (amplitude + dcs + distance + grayscale) << std::endl;
        }
        std::cout << "Shutting down..." << std::endl;
        sensor.stopStream();
    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
