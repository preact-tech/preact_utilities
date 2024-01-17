/**
 * @file dual-streamer.cpp
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
#include <functional>
#include <mutex>
#include <queue>
#include <boost/program_options.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "open3d/Open3D.h"
#include <Eigen/Dense>

using namespace std::chrono_literals;
using namespace std::chrono;
using namespace tofcore;

// Stole this from here:
// https://www.bit-byter.com/blog/files/blocking-q-cpp.html
template <typename T>
class BlockingQueue
{
private:
    std::mutex mut;
    std::queue<T> private_std_queue;
    std::condition_variable condNotEmpty;
    std::condition_variable condNotFull;
    int count; // Guard with Mutex
    const int MAX{20};

public:
    void put(T new_value)
    {

        std::unique_lock<std::mutex> lk(mut);
        // Condition takes a unique_lock and waits given the false condition
        condNotFull.wait(lk, [this]
                         {
      if (count == MAX) {
        // this used to return false so we cannot push when queue is full,
        // but I would rather just throw out old data than hold up new data
        private_std_queue.pop();
        return true;
      } else {
        return true;
      } });
        private_std_queue.push(new_value);
        count++;
        condNotEmpty.notify_one();
    }
    void take(T &value)
    {
        std::unique_lock<std::mutex> lk(mut);
        // Condition takes a unique_lock and waits given the false condition
        condNotEmpty.wait(lk, [this]
                          { return !private_std_queue.empty(); });
        value = private_std_queue.front();
        private_std_queue.pop();
        count--;
        condNotFull.notify_one();
    }

};

std::thread cupidThread;
std::thread inputThread;
std::thread spawn_cupid();
BlockingQueue<open3d::geometry::PointCloud> sensor1_messages;
BlockingQueue<open3d::geometry::PointCloud> sensor2_messages;
BlockingQueue<std::array<open3d::geometry::PointCloud, 2>> matched_messages_;
static uint32_t baudRate{DEFAULT_BAUD_RATE};
static bool captureAmxxx{false};
static bool captureDistance{false};
static std::string devicePort1{DEFAULT_PORT_NAME};
static std::string devicePort2{"/dev/ttyACM2"};
static volatile bool exitRequested{false};
static  bool threadExitRequested{false};
static uint16_t protocolVersion{DEFAULT_PROTOCOL_VERSION};
static uint16_t minAmplitude{0};

tofcore::CartesianTransform cartesianTransform1_;
tofcore::CartesianTransform cartesianTransform2_;
open3d::visualization::Visualizer vis;
std::string transform_input;
std::mutex mut_trans;
std::mutex mut_thread;
float t_x,t_y,t_z,roll,pitch,yaw=0;
size_t split(const std::string &txt, std::vector<std::string> &strs, char ch)
{
    size_t pos = txt.find( ch );
    size_t initialPos = 0;
    strs.clear();

    // Decompose statement
    while( pos != std::string::npos ) {
        strs.push_back( txt.substr( initialPos, pos - initialPos ) );
        initialPos = pos + 1;

        pos = txt.find( ch, initialPos );
    }

    // Add the last one
    strs.push_back( txt.substr( initialPos, std::min( pos, txt.size() ) - initialPos + 1 ) );

    return strs.size();
}

void wait_input()
{

        std::this_thread::sleep_for(3s);

    while (!exitRequested)
    {
        std::this_thread::sleep_for(2s);
        std::cout << "Enter camera 2 transform"<<std::endl; // Type a number and press enter
        std::cout << "Format (angles in degrees): x y z roll pitch yaw"<<std::endl; // Type a number and press enter
        std::getline(std::cin, transform_input);
        std::cout << "Entered: " << transform_input <<std::endl; // Type a number and press enter
        std::vector<std::string> v;

        if(split(transform_input, v, ' ' )==6){
            std::unique_lock<std::mutex> lk(mut_trans);
            t_x=std::stof(v[0]);
            t_y=std::stof(v[1]);
            t_z=std::stof(v[2]);
            roll=std::stof(v[3]);
            pitch=std::stof(v[4]);
            yaw=std::stof(v[5]);
        }else{
        std::cout << "Transform does not have 6 values"<<std::endl; // Type a number and press enter

        }

    }
}
void find_match()
{
    open3d::geometry::PointCloud msg_1;
    open3d::geometry::PointCloud msg_2;

    while (!exitRequested)
    {

        sensor1_messages.take(msg_1);
        sensor2_messages.take(msg_2);
        {
        std::unique_lock<std::mutex> lk(mut_trans);
        Eigen::Matrix3d rot = open3d::geometry::Geometry3D::GetRotationMatrixFromAxisAngle(Eigen::Vector3d(roll,pitch,yaw));
        msg_2.Rotate(rot,Eigen::Vector3d(0,0,0));
        msg_2.Translate(Eigen::Vector3d(t_x,t_y,t_z));
        }
        std::shared_ptr<open3d::geometry::PointCloud> cloud_ptr1 = std::make_shared<open3d::geometry::PointCloud>(msg_1);
        std::shared_ptr<open3d::geometry::PointCloud> cloud_ptr2 = std::make_shared<open3d::geometry::PointCloud>(msg_2);
        // vis.RemoveGeometry();

        vis.ClearGeometries();
        vis.AddGeometry(cloud_ptr1);
        vis.AddGeometry(cloud_ptr2);
        vis.GetViewControl().Rotate(523.6 * 3, 523.6 * 3, 0, 0);
        vis.GetViewControl().SetZoom(0.5);
        vis.GetRenderOption().point_color_option_ = open3d::visualization::RenderOption::PointColorOption::YCoordinate;
        vis.PollEvents();
        vis.UpdateRender();
        vis.UpdateGeometry();
    }
}
std::thread spawn_cupid()
{
    return std::thread(&find_match);
}
static void measurement_callback1(std::shared_ptr<tofcore::Measurement_T> pData)
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
        cv::resize(dist_frame, dist_frame_resized, cv::Size(dist_frame.cols * 4, dist_frame.rows * 4));
        dist_frame_resized *= 5; // add some gain to make image look better
        cv::imshow("Distance Image1", dist_frame_resized);
        cv::waitKey(1);
        cv::Mat amp_frame = cv::Mat(pData->height(), pData->width(), CV_16UC1, (void *)pData->amplitude().begin());
        cv::resize(amp_frame, amp_frame_resized, cv::Size(amp_frame.cols * 4, amp_frame.rows * 4));
        amp_frame_resized *= 100; // add some gain to make image look better
        cv::imshow("Amplitude Image1", amp_frame_resized);
        cv::waitKey(1);
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
            cartesianTransform1_.transformPixel(x, y, distance, px, py, pz);
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
        sensor1_messages.put(point_cloud);
        dist_frame.release();
        dist_frame_resized.release();
        amp_frame.release();
        amp_frame_resized.release();
    }
}
static void measurement_callback2(std::shared_ptr<tofcore::Measurement_T> pData)
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
        cv::resize(dist_frame, dist_frame_resized, cv::Size(dist_frame.cols * 4, dist_frame.rows * 4));
        dist_frame_resized *= 5; // add some gain to make image look better
        cv::imshow("Distance Image2", dist_frame_resized);
        cv::waitKey(1);
        cv::Mat amp_frame = cv::Mat(pData->height(), pData->width(), CV_16UC1, (void *)pData->amplitude().begin());
        cv::resize(amp_frame, amp_frame_resized, cv::Size(amp_frame.cols * 4, amp_frame.rows * 4));
        amp_frame_resized *= 100; // add some gain to make image look better
        cv::imshow("Amplitude Image2", amp_frame_resized);
        cv::waitKey(1);
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
            cartesianTransform2_.transformPixel(x, y, distance, px, py, pz);
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
        sensor2_messages.put(point_cloud);

        dist_frame.release();
        dist_frame_resized.release();
        amp_frame.release();
        amp_frame_resized.release();
    }
}
namespace po = boost::program_options;

static void parseArgs(int argc, char *argv[])
{
    po::options_description desc("illuminator board test");
    desc.add_options()("help,h", "produce help message")("device1-uri,p1", po::value<std::string>(&devicePort1)->default_value(devicePort1))("device2-uri,p2", po::value<std::string>(&devicePort2)->default_value(devicePort2))("protocol-version,v", po::value<uint16_t>(&protocolVersion)->default_value(DEFAULT_PROTOCOL_VERSION))("min-amplitude,m", po::value<uint16_t>(&minAmplitude)->default_value(0));

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
        tofcore::Sensor sensor1{protocolVersion, devicePort1, baudRate};
        tofcore::Sensor sensor2{protocolVersion, devicePort2, baudRate};
        std::vector<double> rays1_x, rays1_y, rays1_z ;
        std::vector<double> rays2_x, rays2_y, rays2_z ;
        try
        {
            sensor1.getLensInfo(rays1_x, rays1_y, rays1_z);
            cartesianTransform1_.initLensTransform(320, 240, rays1_x, rays1_y, rays1_z);
        }
        catch (std::exception &e)
        {
            std::cout << "Error retreiving lens info from camera 1..." << std::endl;
            return -1;
        }
        try
        {
            sensor2.getLensInfo(rays2_x, rays2_y, rays2_z);
            cartesianTransform2_.initLensTransform(320, 240, rays2_x, rays2_y, rays2_z);
        }
        catch (std::exception &e)
        {
            std::cout << "Error retreiving lens info from camera 2..." << std::endl;
            return -1;
        }
        vis.CreateVisualizerWindow("Open3D", 1500, 1000, 75, 50, true);
        vis.PollEvents();
        vis.UpdateRender();

        sensor1.subscribeMeasurement(&measurement_callback1); // callback is called from background thread
        sensor1.setIntegrationTime(4000);
        sensor1.streamDistanceAmplitude();

        sensor2.subscribeMeasurement(&measurement_callback2); // callback is called from background thread
        sensor2.setIntegrationTime(4000);
        sensor2.streamDistanceAmplitude();
        cupidThread = spawn_cupid();
          inputThread=  std::thread(&wait_input);
        while (!exitRequested) // wait for ^\ or ^C
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::cout << "Shutting down..." << std::endl;
        sensor1.stopStream();
        sensor2.stopStream();
        cupidThread.join();
    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
