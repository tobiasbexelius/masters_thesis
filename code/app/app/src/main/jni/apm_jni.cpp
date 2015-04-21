#include "apm_jni.h"
#include "apm_core/lib/include/package_measurer.h"
#include <opencv2/opencv.hpp>
#include <vector>

automatic_package_measuring::PackageMeasurer apm;

jobject CoordinatesToByteBuffer(JNIEnv *env, const std::vector<cv::Point2f>& vec){
    int array_size = vec.size()*2;
    int* array = new int[array_size];
    for(int i = 0, j = 0; i < array_size; i+=2, ++j) {
        array[i] = vec[j].x;
        array[i+1] = vec[j].y;
    }

	jobject bb = (env)->NewDirectByteBuffer((void*) array, array_size*sizeof(int));
    return bb;
}

JNIEXPORT void JNICALL Java_com_apm_tobias_automaticpackagemeasuring_PackageMeasurer_analyzeVideoFrame
(JNIEnv *env, jobject obj, jbyteArray videoFrame, jint width, jint height, jint rotation) {

	int frameWidth = static_cast<int>(width);
	int frameHeight = static_cast<int>(height);

	cv::Mat inputBGR;
	jbyte *baseAddress = env->GetByteArrayElements(videoFrame, NULL);
	cv::Mat tmp(frameHeight + frameHeight / 2, frameWidth, CV_8UC1, baseAddress, frameWidth);
	cv::cvtColor(tmp, inputBGR, CV_YUV420sp2BGR);
	env->ReleaseByteArrayElements(videoFrame, baseAddress, 0);

	apm.AnalyzeImage(inputBGR, rotation);
}

JNIEXPORT jobject JNICALL Java_com_apm_tobias_automaticpackagemeasuring_PackageMeasurer_getReferenceObjectCoordinates(
		JNIEnv *env, jobject obj) {

	std::vector<cv::Point2f> reference_object = apm.GetReferenceObject();
	return CoordinatesToByteBuffer(env, reference_object);
}

JNIEXPORT jobject JNICALL Java_com_apm_tobias_automaticpackagemeasuring_PackageMeasurer_getPackageCoordinates(
		JNIEnv *env, jobject obj) {

	std::vector<cv::Point2f> package = apm.GetPackage();
	return CoordinatesToByteBuffer(env, package);
}

JNIEXPORT jobject JNICALL Java_com_apm_tobias_automaticpackagemeasuring_PackageMeasurer_getPackageDimensions
  (JNIEnv * env, jobject obj) {
    cv::Vec3f dimensions = apm.GetMeasurements();
    float* dimensions_array = new float[3];
    dimensions_array[0] = dimensions[0];
    dimensions_array[1] = dimensions[1];
    dimensions_array[2] = dimensions[2];

    jobject bb = (env)->NewDirectByteBuffer((void*) dimensions_array, 3*sizeof(float));
    return bb;

}

JNIEXPORT jobject JNICALL Java_com_apm_tobias_automaticpackagemeasuring_PackageMeasurer_getPackageMeasuredEdges
  (JNIEnv * env, jobject obj){
cv::Vec3i edges = apm.GetMeasuredEdges();
    int* edges_array = new int[3];
    edges_array[0] = edges[0];
    edges_array[1] = edges[1];
    edges_array[2] = edges[2];

    jobject bb = (env)->NewDirectByteBuffer((void*) edges_array, 3*sizeof(int));
    return bb;
}

JNIEXPORT jobject JNICALL Java_com_apm_tobias_automaticpackagemeasuring_PackageMeasurer_setReferenceObjectSize
  (JNIEnv *, jobject, jfloat width, jfloat height) {
    apm.SetReferenceObjectSize(cv::Vec2f(width, height));
  }
