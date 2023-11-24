/***************************************************************************************
 ***                                                                                 ***
 ***  Copyright (c) 2022, Lucid Vision Labs, Inc.                                    ***
 ***                                                                                 ***
 ***  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR     ***
 ***  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,       ***
 ***  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE    ***
 ***  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER         ***
 ***  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,  ***
 ***  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE  ***
 ***  SOFTWARE.                                                                      ***
 ***                                                                                 ***
 ***************************************************************************************/


#include "GenTL.h"

#ifdef __linux__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#endif

#include "GenICam.h"

#ifdef __linux__
#pragma GCC diagnostic pop
#endif

#include "ArenaApi.h"

#define TAB1 "  "
#define TAB2 "    "
#define TAB3 "      "

#include <iostream>
#include <chrono>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>


//说明：使用的是HLT0035-001相机，属于Helios2型号
//只支持Coord3D_ABCY16数据类型
//将相机的最远深度设置为6000mm，可信范围为0.6～6m
//设置触发模式，相机一直启动，当信号isTrigered=1时才获取图像
//可以连续触发，但是注意触发的频率和相机本身频率可能存在冲突
//最后如果需要通过ros发布出来，再另外设置

// =-=-=-=-=-=-=-=-=-
// =-=- SETTINGS =-=-
// =-=-=-=-=-=-=-=-=-

#define PIXEL_FORMAT "Coord3D_ABCY16"
#define TIMEOUT 2000

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> myPointCloud;
ros::Publisher pcl_pub;

//验证设备信息
bool ValidateDevice(Arena::IDevice* pDevice)
{
	GenApi::INodeMap* pNodeMap = pDevice->GetNodeMap();

	// validate if Scan3dCoordinateSelector node exists. If not - probaly not
	// Helios camera used running the example
	GenApi::CEnumerationPtr checkpCoordSelector = pNodeMap->GetNode("Scan3dCoordinateSelector");
	// validate if Scan3dCoordinateOffset node exists. If not - probaly Helios
	// has an old firmware
	GenApi::CFloatPtr checkpCoord = pNodeMap->GetNode("Scan3dCoordinateOffset");

	if (!checkpCoordSelector)
	{
		std::cout << TAB1 << "Scan3dCoordinateSelector node is not found. Please make sure that Helios device is used for the example.\n\n";
		return false;
	}
	else if (!checkpCoord)

	{
		std::cout << TAB1 << "Scan3dCoordinateOffset node is not found. Please update Helios firmware.\n\n";
		return false;
	}
	else
	{
		return true;
	}
}

//将相机的图像数据Arena::IImage* pImage
//转换为pcl点云，
void convertPointCloud(Arena::IImage* pImage, const myPointCloud::Ptr pointcloud1)
{
	size_t width = pImage->GetWidth();
	size_t height = pImage->GetHeight();
	size_t size = width * height;
	size_t srcBpp = pImage->GetBitsPerPixel();
	size_t srcPixelSize = srcBpp / 8;
	const uint8_t* pInput = pImage->GetData();
	const uint8_t* pIn = pInput;

	pointcloud1->points.resize(0);
	pointcloud1->points.reserve(size);

	std::cout << TAB2 << "Convert points to pcl format\n";
	for (size_t i = 0; i < size; i++)
	{
		// Extract point data to signed 16 bit integer
		//    The first channel is the x coordinate, second channel is the y
		//    coordinate, the third channel is the z coordinate and the
		//    fourth channel is intensity. We offset pIn by 2 for each
		//    channel because pIn is an 8 bit integer and we want to read it
		//    as a 16 bit integer.
		uint16_t x = *reinterpret_cast<const uint16_t*>(pIn);
		uint16_t y = *reinterpret_cast<const uint16_t*>((pIn + 2));
		uint16_t z = *reinterpret_cast<const uint16_t*>((pIn + 4));
		uint16_t intensity = *reinterpret_cast<const uint16_t*>((pIn + 6));
		float x_coord = (float(x) * 0.25f + (-8192));//0.25是尺度因子，-8192是和数据类型相关的偏移量
		float y_coord = (float(y) * 0.25f + (-8192));//这两个都是确定的
		float z_coord = (float(z) * 0.25f);
		PointT p;
		if (z < 4000 * 6 && z > 4000 * 0.6)//只要0.6～6m范围内
		{
			p.x = x_coord * 0.001f;
			p.y = y_coord * 0.001f;
			p.z = z_coord * 0.001f;
			// std::cout << x << ", " << y << ", " << z << std::endl;
		}
		else//范围之外设置为Nan点
		{
			p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN ();
		}
		pointcloud1->points.push_back(p);
		//数据遍历到下一格
		pIn += srcPixelSize;
	}
	pointcloud1->is_dense = false;
	pointcloud1->height = size;
	pointcloud1->width = 1;
	pInput = NULL;
	delete[] pInput;
	pIn = NULL;
	delete[] pIn;

}



// demonstrates basic trigger configuration and use
// (1) sets trigger mode, source, and selector
// (2) starts stream
// (3) waits until trigger is armed
// (4) triggers image
// (5) gets image
// (6) requeues buffer
// (7) stops stream
void ConfigureTriggerAndAcquireImage(Arena::IDevice* pDevice)
{
	GenApi::INodeMap* pNodeMap = pDevice->GetNodeMap();

	//------------------设备信息检验-----------------------
    GenApi::CEnumerationPtr checkpCoordSelector = pNodeMap->GetNode("Scan3dCoordinateSelector");
	if (!checkpCoordSelector)
	{
		std::cout << TAB1 << "Scan3dCoordinateSelector node is not found. Please make sure that Helios device is used for the example.\n";
		return;
	}
    GenApi::CFloatPtr checkpCoord = pNodeMap->GetNode("Scan3dCoordinateOffset");
	if (!checkpCoord)
	{
		std::cout << TAB1 << "Scan3dCoordinateOffset node is not found. Please update Helios firmware.\n";
		return;
	}
    bool isHelios2 = false;
	GenICam::gcstring deviceModelName = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "DeviceModelName");
	std::string deviceModelName_tmp = deviceModelName.c_str();
	if (deviceModelName_tmp.rfind("HLT", 0) == 0 || deviceModelName_tmp.rfind("HTP", 0) == 0)
	{
		isHelios2 = true;
	}

	//-----------------保存基础设置，程序最后需要还原-----------------------
	// get node values that will be changed in order to return their values at
	// the end of the example
	GenICam::gcstring triggerSelectorInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TriggerSelector");
	GenICam::gcstring triggerModeInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TriggerMode");
	GenICam::gcstring triggerSourceInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "TriggerSource");
    GenICam::gcstring pixelFormatInitial = Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "PixelFormat");
	GenICam::gcstring operatingModeInitial = Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode");

	//-----------------基础设置：图像获取部分-----------------------
	std::cout << TAB1 << "Set " << PIXEL_FORMAT << " to pixel format\n";
	Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "PixelFormat", PIXEL_FORMAT);
	if (isHelios2)
	{
		std::cout << TAB1 << "Set 3D operating mode to Distance6000mm\n";
		Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode", "Distance6000mmSingleFreq");
	}
	else
	{
		std::cout << TAB1 << "Set 3D operating mode to Distance1500mm\n";
		Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode", "Distance1500mm");
	}
	// enable stream auto negotiate packet size
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
	// enable stream packet resend
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

	//-----------------基础设置：触发模式部分-----------------------
	// Set trigger selector
	//    Set the trigger selector to FrameStart. When triggered, the device will
	//    start acquiring a single frame. This can also be set to
	//    AcquisitionStart or FrameBurstStart.
	std::cout << TAB1 << "Set trigger selector to FrameStart\n";
	Arena::SetNodeValue<GenICam::gcstring>(
		pDevice->GetNodeMap(),
		"TriggerSelector",
		"FrameStart");
	// Set trigger mode
	//    Enable trigger mode before setting the source and selector and before
	//    starting the stream. Trigger mode cannot be turned on and off while the
	//    device is streaming.
	std::cout << TAB1 << "Enable trigger mode\n";
	Arena::SetNodeValue<GenICam::gcstring>(
		pDevice->GetNodeMap(),
		"TriggerMode",
		"On");

	// Set trigger source
	//    Set the trigger source to software in order to trigger images without
	//    the use of any additional hardware. Lines of the GPIO can also be used
	//    to trigger.
	std::cout << TAB1 << "Set trigger source to Software\n";
	Arena::SetNodeValue<GenICam::gcstring>(
		pDevice->GetNodeMap(),
		"TriggerSource",
		"Software");


	//----------------Start stream--------------------
	//    When trigger mode is off and the acquisition mode is set to stream
	//    continuously, starting the stream will have the camera begin acquiring
	//    a steady stream of images. However, with trigger mode enabled, the
	//    device will wait for the trigger before acquiring any.
	std::cout << TAB1 << "Start stream\n";
	pDevice->StartStream();

	// Trigger Armed
	//    Continually check until trigger is armed. Once the trigger is armed, it
	//    is ready to be executed.
	// 原程序里面，判断Device中是否有triggerArmed量判断是否触发。在我们的程序中使用另外的策略
	std::cout << TAB2 << "Wait until trigger is armed\n";
	bool triggerArmed = false;//
	myPointCloud::Ptr pointcloud1 = pcl::make_shared<myPointCloud>();
        
	//让i从0到10000，遇到1000就触发一次
	for(int i = 0; i < 10000; ++i)
	{
		if(i % 1000 != 0)
		{
			continue;
		}
		// Trigger an image
		//    Trigger an image manually, since trigger mode is enabled. This triggers
		//    the camera to acquire a single image. A buffer is then filled and moved
		//    to the output queue, where it will wait to be retrieved.
		std::cout << TAB2 << "Trigger image ------------------\n";
		auto time1 = std::chrono::high_resolution_clock::now();
		Arena::ExecuteNode(
			pDevice->GetNodeMap(),
			"TriggerSoftware");
		auto time2 = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::milli> time_duration = time2 - time1;
		// ROS_INFO("获取一张图像花费： %.2fms", (time2 - time1).toSec() * 1000);
		std::cout << TAB2 << "获取一张图像花费：" << time_duration.count() << " ms" << std::endl;
		// Get image
		//    Once an image has been triggered, it can be retrieved. If no image has
		//    been triggered, trying to retrieve an image will hang for the duration
		//    of the timeout and then throw an exception.
		std::cout << TAB2 << "Get image";

		Arena::IImage* pImage = pDevice->GetImage(TIMEOUT);
		std::cout << " (" << pImage->GetWidth() << "x" << pImage->GetHeight() << ")\n";

		time1 = std::chrono::high_resolution_clock::now();
		convertPointCloud(pImage, pointcloud1);
		time2 = std::chrono::high_resolution_clock::now();
		time_duration = time2 - time1;
		std::cout << TAB2 << "转换一张图像花费：" << time_duration.count() << " ms" << std::endl;
		//发布点云
		sensor_msgs::PointCloud2 outputPointCloud1;  
		pcl::toROSMsg(*pointcloud1, outputPointCloud1);
		outputPointCloud1.header.stamp = ros::Time::now();
		outputPointCloud1.header.frame_id = "map";
		pcl_pub.publish(outputPointCloud1);  
		// requeue buffer
		std::cout << TAB2 << "Requeue buffer\n";
		pDevice->RequeueBuffer(pImage);
	}
	
	// Stop the stream
	std::cout << TAB1 << "Stop stream\n";
	pDevice->StopStream();

	// return nodes to their initial values
	Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "TriggerSource", triggerSourceInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "TriggerMode", triggerModeInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "TriggerSelector", triggerSelectorInitial);
	Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode", operatingModeInitial);
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "PixelFormat", pixelFormatInitial);

}

// =-=-=-=-=-=-=-=-=-
// =- PREPARATION -=-
// =- & CLEAN UP =-=-
// =-=-=-=-=-=-=-=-=-

int main(int argc, char** argv)
{
	ros::init (argc, argv, "publish_pointcloud");  
	ros::NodeHandle nh; 
	pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pointcloud", 10); 
	// flag to track when an exception has been thrown
	bool exceptionThrown = false;
	std::cout << "Cpp_Trigger\n";
	try
	{
		// prepare example
		Arena::ISystem* pSystem = Arena::OpenSystem();
		pSystem->UpdateDevices(1000);
		std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
		if (deviceInfos.size() == 0)
		{
			std::cout << "\nNo camera connected\nPress enter to complete\n";
			std::getchar();
			return 0;
		}
		Arena::IDevice* pDevice = pSystem->CreateDevice(deviceInfos[0]);

		// run example
		std::cout << "Commence example\n\n";
		ConfigureTriggerAndAcquireImage(pDevice);
		std::cout << "\nExample complete\n";

		// clean up example
		pSystem->DestroyDevice(pDevice);
		Arena::CloseSystem(pSystem);
	}
	catch (GenICam::GenericException& ge)
	{
		std::cout << "\nGenICam exception thrown: " << ge.what() << "\n";
		exceptionThrown = true;
	}
	catch (std::exception& ex)
	{
		std::cout << "Standard exception thrown: " << ex.what() << "\n";
		exceptionThrown = true;
	}
	catch (...)
	{
		std::cout << "Unexpected exception thrown\n";
		exceptionThrown = true;
	}

	std::cout << "Press enter to complete\n";
	std::getchar();

	if (exceptionThrown)
		return -1;
	else
		return 0;
}
