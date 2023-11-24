#include <iostream>
#include <ArenaApi.h>
// #include <SaveApi.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#define TAB1 "  "
#define TAB2 "    "
#define TAB3 "      "

#define PIXEL_FORMAT "Coord3D_ABCY16"

#define IMAGE_TIMEOUT 2000


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> myPointCloud;

ros::Publisher pcl_pub;
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


void convertPointCloud(Arena::IDevice* pDevice)
{
    GenApi::INodeMap* pNodeMap = pDevice->GetNodeMap();
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
    GenICam::gcstring pixelFormatInitial = Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "PixelFormat");
	GenICam::gcstring operatingModeInitial = Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode");
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
	
	std::cout << TAB1 << "Get xyz coordinate scales and offsets\n\n";
	Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dCoordinateSelector", "CoordinateA");
	// getting scaleX as float by casting since SetPly() will expect it passed as
	// float
	float scaleX = static_cast<float>(Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateScale"));
	// getting offsetX as float by casting since SetPly() will expect it passed
	// as float
	float offsetX = static_cast<float>(Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateOffset"));
	Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dCoordinateSelector", "CoordinateB");
	double scaleY = Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateScale");
	// getting offsetY as float by casting since SetPly() will expect it passed
	// as float
	float offsetY = static_cast<float>(Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateOffset"));
	Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dCoordinateSelector", "CoordinateC");
	double scaleZ = Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateScale");
	std::cout << TAB1 << "(offsetX, offsetY) = ("<< offsetX << ", " << offsetY << ")" << std::endl;
	
	// enable stream auto negotiate packet size
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);

	// enable stream packet resend
	Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);
	std::cout << TAB2 << "Acquire image\n";

	pDevice->StartStream();
    int count = 0;
    
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer(new pcl::visualization::PCLVisualizer("Cloud"));

	while(1)
    {
        Arena::IImage* pImage = pDevice->GetImage(IMAGE_TIMEOUT);
        // prepare info from input buffer
        size_t width = pImage->GetWidth();
        size_t height = pImage->GetHeight();
        size_t size = width * height;
        size_t srcBpp = pImage->GetBitsPerPixel();
        size_t srcPixelSize = srcBpp / 8;
        const uint8_t* pInput = pImage->GetData();
	    const uint8_t* pIn = pInput;
		myPointCloud::Ptr pointcloud1 = pcl::make_shared<myPointCloud>();
        pointcloud1->points.resize(0);
        if(count == 0)
        {
            pointcloud1->points.reserve(size);
        }
        std::cout << TAB2 << "Convert points to pcl format\n";
        //我们的格式是 ABCY16
		int count2 = 0, count3 =0, count4 = 0;
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
			// uint32_t z_32 = *reinterpret_cast<const uint32_t*>((pIn + 4));
			uint16_t x_orig = x;
			uint16_t y_orig = y;
			uint16_t z_orig = z;
			uint16_t intensity = *reinterpret_cast<const uint16_t*>((pIn + 6));
			float x_coord = (float(x) * 0.25f + offsetX);
			float y_coord = (float(y) * 0.25f + offsetY);
			float z_coord = (float(z) * 0.25f);
			float z_coord_old = z_coord;
			// if(z_coord <= 1000)
			// {
			// 	z_coord += 65535.0f * 0.25f;
			// 	x_coord = x_coord * z_coord/z_coord_old;
			// 	y_coord = y_coord * z_coord/z_coord_old;
			// }
			// if(z_32 - z > 10 || z_32 - z < -10)
			// {
			// 	// std::cout << x << ", " << y << ", " << z << std::endl;
			// 	std::cout << z_32 << ", " << z << std::endl;
			// }
			// std::cout << z_32 << std::endl;
			// if z is less than max value, as invalid values get filtered to
			// 65535
			// z值在一定范围内的才保留，太远()和太近(0.25m)的不要	
			PointT p;
			if (z_orig < 4000 * 6 && z_orig > 4000 * 0.6)//只要0.6～6m范围内
			{
				// Convert x, y and z to millimeters
				//    Using each coordinates' appropriate scales, convert x, y
				//    and z values to mm. For the x and y coordinates in an
				//    unsigned pixel format, we must then add the offset to our
				//    converted values in order to get the correct position in
				//    millimeters.

				p.x = x_coord * 0.001f;
				p.y = y_coord * 0.001f;
				p.z = z_coord * 0.001f;
				// std::cout << x << ", " << y << ", " << z << std::endl;
				count2 ++;
			}
			else
			{
				
				p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN ();
				if(z_orig == 0)
				{
					// std::cout << z_orig << std::endl;
					count3 ++ ;
				}
				if(z_orig == 65535)
				{
					count4 ++;
				}
			}
			pointcloud1->points.push_back(p);
			pIn += srcPixelSize;
		}
		// pcl_viewer->removePointCloud("oneFrameCloudPoint");
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb0(pointcloud1, 0, 0, 0);
		// pcl_viewer->addPointCloud(pointcloud1, rgb0, "oneFrameCloudPoint");
		pointcloud1->is_dense = false;
		pointcloud1->height = size;
		pointcloud1->width = 1;
		std::cout << TAB2 << "当前帧：" << count << std::endl;
		std::cout << TAB2 << "处于范围内的点数："<< count2 << " 处于范围外的点数：" << count3 << ", "<< count4 << std::endl;
		std::cout << TAB2 << "当前点云内的点数："<< pointcloud1->points.size() << std::endl;
		count ++;
		pInput = NULL;
		delete[] pInput;
		pIn = NULL;
		delete[] pIn;
		pDevice->RequeueBuffer(pImage);
		sensor_msgs::PointCloud2 outputPointCloud1;  
		pcl::toROSMsg(*pointcloud1, outputPointCloud1);
		outputPointCloud1.header.stamp = ros::Time::now();
		outputPointCloud1.header.frame_id = "map";
		pcl_pub.publish(outputPointCloud1);  
    }
    pDevice->StopStream();

    // return nodes to their initial values
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode", operatingModeInitial);
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "PixelFormat", pixelFormatInitial);


}


int main(int argc, char** argv)
{
	ros::init (argc, argv, "publish_pointcloud");  
	ros::NodeHandle nh; 
	pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pointcloud", 10); 
	// flag to track when an exception has been thrown
	bool exceptionThrown = false;

	std::cout << "Cpp_Helios_MinMaxDepth\n";

	try
	{
		// prepare example
		Arena::ISystem* pSystem = Arena::OpenSystem();
		pSystem->UpdateDevices(100);
		std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
		if (deviceInfos.size() == 0)
		{
			std::cout << "\nNo camera connected\nPress enter to complete\n";
			std::getchar();
			return 0;
		}
		Arena::IDevice* pDevice = pSystem->CreateDevice(deviceInfos[0]);

		std::cout << "Commence example\n\n";

		// run example
		convertPointCloud(pDevice);

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
		std::cout << "\nStandard exception thrown: " << ex.what() << "\n";
		exceptionThrown = true;
	}
	catch (...)
	{
		std::cout << "\nUnexpected exception thrown\n";
		exceptionThrown = true;
	}

	std::cout << "Press enter to complete\n";
	std::getchar();

	if (exceptionThrown)
		return -1;
	else
		return 0;
}