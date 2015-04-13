#include "com_apm_tobias_automaticpackagemeasuring_core_PackageMeasurer.h"
#include "apm_core/lib/include/package_measurer.h"
#include <opencv2/opencv.hpp>
#include <vector>

using automatic_package_measuring::PackageMeasurer;

PackageMeasurer apm;

struct Point {
	int x;
	int y;
};

JNIEXPORT void JNICALL Java_com_apm_tobias_automaticpackagemeasuring_core_PackageMeasurer_analyzeVideoFrame
(JNIEnv *env, jobject obj, jbyteArray videoFrame, jint width, jint height) {

	int frameWidth = static_cast<int>(width);
	int frameHeight = static_cast<int>(height);

	cv::Mat inputBGR;
	jbyte *baseAddress = env->GetByteArrayElements(videoFrame, NULL);
	cv::Mat tmp(frameHeight + frameHeight / 2, frameWidth, CV_8UC1, baseAddress, frameWidth);
	cv::cvtColor(tmp, inputBGR, CV_YUV420sp2BGR);
	env->ReleaseByteArrayElements(videoFrame, baseAddress, 0);

	apm.AnalyzeImage(inputBGR);
}

JNIEXPORT jobject JNICALL Java_com_apm_tobias_automaticpackagemeasuring_core_PackageMeasurer_getReferenceObjectCoordinates(
		JNIEnv *env, jobject obj) {

	std::vector<cv::Point2f> objectCoordinates = apm.GetReferenceObject();
	Point* ref = new Point[objectCoordinates.size()];

	for (int i = 0; i < objectCoordinates.size(); ++i) {
		Point p;
		p.x = objectCoordinates[i].x;
		p.y = objectCoordinates[i].y;
		ref[i] = p;
	}

	jobject bb = (env)->NewDirectByteBuffer((void*) ref,
			sizeof(Point)*objectCoordinates.size());

	return bb;
}

JNIEXPORT jobject JNICALL Java_com_apm_tobias_automaticpackagemeasuring_core_PackageMeasurer_getPackageCoordinates(
		JNIEnv *env, jobject obj) {

	std::vector<cv::Point2f> objectCoordinates = apm.GetPackage();
	Point* ref = new Point[objectCoordinates.size()];

	for (int i = 0; i < objectCoordinates.size(); ++i) {
		Point p;
		p.x = objectCoordinates[i].x;
		p.y = objectCoordinates[i].y;
		ref[i] = p;
	}

	jobject bb = (env)->NewDirectByteBuffer((void*) ref,
			sizeof(Point)*objectCoordinates.size());

	return bb;
}
