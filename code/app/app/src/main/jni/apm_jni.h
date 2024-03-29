/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class com_apm_tobias_automaticpackagemeasuring_PackageMeasurer */

#ifndef _Included_com_apm_tobias_automaticpackagemeasuring_PackageMeasurer
#define _Included_com_apm_tobias_automaticpackagemeasuring_PackageMeasurer
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     com_apm_tobias_automaticpackagemeasuring_PackageMeasurer
 * Method:    analyzeVideoFrame
 * Signature: -
 */
JNIEXPORT void JNICALL Java_com_apm_tobias_automaticpackagemeasuring_PackageMeasurer_analyzeVideoFrame
  (JNIEnv *, jobject, jbyteArray, jint, jint, jint);

/*
 * Class:     com_apm_tobias_automaticpackagemeasuring_PackageMeasurer
 * Method:    getReferenceObjectCoordinates
 * Signature: ()Ljava/nio/ByteBuffer;
 */
JNIEXPORT jobject JNICALL Java_com_apm_tobias_automaticpackagemeasuring_PackageMeasurer_getReferenceObjectCoordinates
  (JNIEnv *, jobject);

/*
 * Class:     com_apm_tobias_automaticpackagemeasuring_PackageMeasurer
 * Method:    getPackageCoordinates
 * Signature: ()Ljava/nio/ByteBuffer;
 */
JNIEXPORT jobject JNICALL Java_com_apm_tobias_automaticpackagemeasuring_PackageMeasurer_getPackageCoordinates
  (JNIEnv *, jobject);

/*
 * Class:     com_apm_tobias_automaticpackagemeasuring_PackageMeasurer
 * Method:    getPackageDimensions
 * Signature: ()Ljava/nio/ByteBuffer;
 */
JNIEXPORT jobject JNICALL Java_com_apm_tobias_automaticpackagemeasuring_PackageMeasurer_getPackageDimensions
  (JNIEnv *, jobject);

/*
 * Class:     com_apm_tobias_automaticpackagemeasuring_PackageMeasurer
 * Method:    getPackageMeasuredEdges
 * Signature: ()Ljava/nio/ByteBuffer;
 */
JNIEXPORT jobject JNICALL Java_com_apm_tobias_automaticpackagemeasuring_PackageMeasurer_getPackageMeasuredEdges
  (JNIEnv *, jobject);

/*
 * Class:     com_apm_tobias_automaticpackagemeasuring_PackageMeasurer
 * Method:    setReferenceObjectSize
 * Signature: -
 */
JNIEXPORT jobject JNICALL Java_com_apm_tobias_automaticpackagemeasuring_PackageMeasurer_setReferenceObjectSize
  (JNIEnv *, jobject, jfloat, jfloat);


#ifdef __cplusplus
}
#endif
#endif
