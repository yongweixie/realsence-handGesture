#include"Hand3D.hpp"
#include"usart.h"
#include"RealSenseHelper.hpp"
#include<opencv2/opencv.hpp>
#include<Windows.h>
#include"ringbuffer.hpp"
using namespace std;
using namespace cv;
#include<time.h>
Comm usart;
#define devView(i) imshow(#i,i)
int setHand(unsigned char id, UINT16 param, unsigned char speed_level);
void RandHandset(int n);
Scalar colors[3] = { Scalar(255,0,0), Scalar(0,255,0), Scalar(0,0,255) };
int main()
{
	LARGE_INTEGER t0, t,freq;
	QueryPerformanceCounter(&t0);
	ringBuffer<PXCPoint3DF32> handTrack(5);
	PXCSenseManager* sr300_manager = PXCSenseManager::CreateInstance();
	sr300_manager->EnableStream(PXCCapture::STREAM_TYPE_COLOR,640,360,60);
	PXCImage *colorIm;
	usart.bInitPort("COM3");
	//KillDCM();
	PXCSmoother *smooth = NULL;
	sr300_manager->EnableHand();
	PXCHandModule *handModule = sr300_manager->QueryHand();
	PXCHandConfiguration* handConfig = handModule->CreateActiveConfiguration();
	handConfig->SetTrackingMode(PXCHandData::TrackingModeType::TRACKING_MODE_FULL_HAND);
	handConfig->EnableStabilizer(true);
	handConfig->EnableTrackedJoints(true);
	handConfig->EnableNormalizedJoints(true);
	handConfig->EnableSegmentationImage(true);
	handConfig->ApplyChanges();
	PXCHandData* handData = handModule->CreateOutput();
	sr300_manager->QuerySession()->CreateImpl<PXCSmoother>(&smooth);
	PXCSmoother::Smoother1D* smoother[5];
	for (size_t i = 0; i < 5; i++) 
	{
		smoother[i] = smooth->Create1DSpring();
		smoother[i]->SmoothValue(0.1f);
	}
	sr300_manager->Init();
	int hist_w = 50;
	Mat indicator = Mat::zeros(Size(5 * hist_w, 5 * hist_w), CV_8UC3);
	pxcStatus ret = PXC_STATUS_NO_ERROR;
	Mat mask;
	RNG rng;


	while (ret >= PXC_STATUS_NO_ERROR || ret == -303)
	{
		QueryPerformanceCounter(&t0);
		if (sr300_manager->AcquireFrame(true) < PXC_STATUS_NO_ERROR) break;
		indicator.setTo(0);
		handData->Update();
		handData->QueryNumberOfHands();
		pxcUID handId;
		handData->QueryHandId(PXCHandData::ACCESS_ORDER_BY_TIME, 0, handId);
		PXCHandData::IHand *ihand = 0;
		//cout << handData->QueryNumberOfHands();
		if (handData->QueryHandDataById(handId, ihand) == pxcStatus::PXC_STATUS_DATA_UNAVAILABLE) {
			cout << "请将手放置于摄像头前方" << endl;
			sr300_manager->ReleaseFrame();
			//destroyAllWindows();
			continue;
		}
		//if (!ihand->IsCalibrated()) {
		//	cout << "正在校准..." << endl;
		//	sr300_manager->ReleaseFrame();
		//	continue;
		//}
		PXCHandData::FingerData finger[5];
		for (size_t i = 0; i < 5; i++)
		{
			ihand->QueryFingerData((PXCHandData::FingerType)i, finger[i]);
			//cout << "Finger ID：" << i << " --- " << (int)(finger[i].foldedness / 100.0f * indicator.rows) << endl;
				auto percent = smoother[i]->SmoothValue(finger[i].foldedness) / 100.0f;
				auto h = (int)(percent * indicator.rows);
				rectangle(indicator, Rect(i*hist_w, indicator.rows - h, hist_w, h), Scalar(255, 255, 255), -1);
			/*if (!i) continue;
			setHand(i + 16, (1 - percent) * 90, 1);*/

		}
		int nfold = 0,state = 0;
		for (size_t i = 0; i < 5; i++) {
			nfold += finger[i].foldedness / 100.0f < 0.5 ? 1 : 0;
		}
		auto sideType = ihand->QueryBodySide();
		auto worldPos = ihand->QueryMassCenterWorld();

		handTrack.push_back(worldPos);
			if ((handTrack[handTrack.size() - 1].z - handTrack[0].z) <-0.03
				&& (handTrack[handTrack.size() - 1].y  - handTrack[0].y) <-0.03)
			{
				RandHandset(rand() % 3);
				//cout << "出拳" <<" " ;
				for (size_t i = 0; i < handTrack.size(); i++)
				{
					handTrack[i] = { 0,0, 0 };
				}
			}
		
		if (ihand->HasSegmentationImage())
		{
			PXCImage* handMask_meta = NULL;
			ihand->QuerySegmentationImage(handMask_meta);
			mask = getMat(handMask_meta, Size(640, 360), PXCImage::PIXEL_FORMAT_Y8, CV_8U);
			flip(mask, mask, 1);
			devView(mask);
		}

		PXCCapture::Sample *sample = sr300_manager->QuerySample();
		colorIm = sample->color;
		Mat handcolor = getMat(colorIm, Size(640, 360), PXCImage::PIXEL_FORMAT_RGB24, CV_8UC3);
		flip(handcolor, handcolor, 1);
		devView(handcolor);
	
		imshow("手指指示器", indicator);
		if (waitKey(1) == 27) break;
		sr300_manager->ReleaseFrame();
		QueryPerformanceCounter(&t);
		QueryPerformanceFrequency(&freq);
		double ms = (t.QuadPart - t0.QuadPart) * 1000 / freq.QuadPart;
		//cout << ms << endl;
		
	}
	for (size_t i = 0; i < 5; i++) 
	{
		smoother[i]->Release();
	}
	//cout << ret << endl;
	cvMoveWindow("handcolor", 650, 0);
	cvMoveWindow("手指指示器", 400, 400);
	cvMoveWindow("mask", 0, 0);
	smooth->Release();
	handData->Release();
	sr300_manager->Release();
	//system("pause");
	return 0;
}




int setHand(unsigned char id, UINT16 param, unsigned char speed_level)
{
	union param16 { unsigned char data[2];	UINT16 value; } u16;
	u16.value = param;
	union fingers
	{
		unsigned char data[4];
	};
	unsigned char pkg[8];
	pkg[7] = 0;
	pkg[7] += pkg[0] = 0xFF;
	pkg[7] += pkg[1] = 0xFF;
	pkg[7] += pkg[2] = id;
	pkg[7] += pkg[3] = 0x00;
	pkg[7] += pkg[4] = u16.data[0];
	pkg[7] += pkg[5] = u16.data[1];
	pkg[7] += pkg[6] = speed_level;
	WriteFile(usart.hCom, pkg, 8, NULL, NULL);
	Sleep(1);
}
void RandHandset(int n)
{
	cout << n << endl;
	switch (n)
	{
	case 0:
		setHand(0 + 16, 89, 0);
		setHand(1 + 16, 0, 0);
		setHand(2 + 16, 0, 0);
		setHand(3 + 16, 89, 0);
		setHand(4 + 16, 89, 0);
		break;
	case 1:
		setHand(0 + 16, 00, 0);
		setHand(1 + 16, 00, 0);
		setHand(2 + 16, 00, 0);
		setHand(3 + 16, 00, 0);
		setHand(4 + 16, 00, 0);
		break;

	case 2:
		setHand(0 + 16, 89, 0);
		setHand(1 + 16, 89, 0);
		setHand(2 + 16, 89, 0);
		setHand(3 + 16, 89, 0);
		setHand(4 + 16, 89, 0);
		break;
	default:
		break;
	}
}