// Standard Library
#include <iostream>
// OpenCV Header
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// Kinect for Windows SDK Header
#include <Kinect.h>
using namespace std;

class WorkTimer {
public:
	WorkTimer(){}
	~WorkTimer(){}
	double time;
	void start(){
		tickBegin = cv::getTickCount();
	}
	void stop(){
		tickEnd = cv::getTickCount();
		time = 1000.0 * (double)(tickEnd - tickBegin) / (double)(cv::getTickFrequency());
	}
private:
	int64 tickBegin;
	int64 tickEnd;
};

int main(int argc, char** argv)
{
	// 1a. Get default Sensor
	IKinectSensor* pSensor = nullptr;
	GetDefaultKinectSensor(&pSensor);
	// 1b. Open sensor
	pSensor->Open();
	// 2a. Get frame source
	IDepthFrameSource* pDepthFrameSource = nullptr;
	pSensor->get_DepthFrameSource(&pDepthFrameSource);

	/*IColorFrameSource* pColorSource = nullptr;
	pSensor->get_ColorFrameSource(&pColorSource);*/

	// 2b. Get frame description
	int        depthWidth = 0;
	int        depthHeight = 0;
	IFrameDescription* pDepthFrameDescription = nullptr;
	pDepthFrameSource->get_FrameDescription(&pDepthFrameDescription);
	pDepthFrameDescription->get_Width(&depthWidth);
	pDepthFrameDescription->get_Height(&depthHeight);
	pDepthFrameDescription->Release();
	pDepthFrameDescription = nullptr;

	// 2c. get some dpeth only meta
	UINT16 uDepthMin = 0, uDepthMax = 0;
	pDepthFrameSource->get_DepthMinReliableDistance(&uDepthMin);
	pDepthFrameSource->get_DepthMaxReliableDistance(&uDepthMax);
	cout << "Reliable Distance: "
		<< uDepthMin << " ¨C " << uDepthMax << endl;
	
	// perpare OpenCV
	cv::Mat mDepthImg(depthHeight, depthWidth, CV_16UC1);
	cv::Mat mImg8bit(depthHeight, depthWidth, CV_8UC1);

	// 3a. get frame reader
	IDepthFrameReader* pDepthFrameReader = nullptr;
	pDepthFrameSource->OpenReader(&pDepthFrameReader);
	// Enter main loop
	int id = 0;
	WorkTimer timer;
	while (true)
	{
		// 4a. Get last frame
		IDepthFrame* pDepthFrame = nullptr;
		if (pDepthFrameReader->AcquireLatestFrame(&pDepthFrame) == S_OK)
		{
			timer.start();
			// 4c. copy the depth map to image
			pDepthFrame->CopyFrameDataToArray(depthWidth * depthHeight,
				reinterpret_cast<UINT16*>(mDepthImg.data));
			// 4d. convert from 16bit to 8bit
			mDepthImg.convertTo(mImg8bit, CV_8U, 255.0f / uDepthMax);
			//cv::imshow("Depth Map", mImg8bit);

			std::string nameTail = ".bmp";
			string location= "D:\\depthimage\\";
			cv::imwrite(location + to_string(id) + nameTail, mImg8bit);
			id++;
			// 4e. release frame
			pDepthFrame->Release();
			timer.stop();
			cout << "Time to write an image to file: " << timer.time << " ms" << endl;
		}
		// 4f. check keyboard input
		/*if (cv::waitKey(30) == VK_ESCAPE){
			break;
		}*/
	}
	// 3b. release frame reader
	pDepthFrameReader->Release();
	pDepthFrameReader = nullptr;
	// 2d. release Frame source
	pDepthFrameSource->Release();
	pDepthFrameSource = nullptr;
	// 1c. Close Sensor
	pSensor->Close();
	// 1d. Release Sensor
	pSensor->Release();
	pSensor = nullptr;
	return 0;
}