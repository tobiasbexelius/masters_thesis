/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class com_apm_tobias_automaticpackagemeasuring_core_PackageMeasurer */

#ifndef _Included_com_apm_tobias_automaticpackagemeasuring_core_PackageMeasurer
#define _Included_com_apm_tobias_automaticpackagemeasuring_core_PackageMeasurer
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     com_apm_tobias_automaticpackagemeasuring_core_PackageMeasurer
 * Method:    analyzeVideoFrame
 * Signature: ([BII)V
 */
JNIEXPORT void JNICALL Java_com_apm_tobias_automaticpackagemeasuring_core_PackageMeasurer_analyzeVideoFrame
  (JNIEnv *, jobject, jbyteArray, jint, jint);

/*
 * Class:     com_apm_tobias_automaticpackagemeasuring_core_PackageMeasurer
 * Method:    getReferenceObjectCoordinates
 * Signature: ()Ljava/nio/ByteBuffer;
 */
JNIEXPORT jobject JNICALL Java_com_apm_tobias_automaticpackagemeasuring_core_PackageMeasurer_getReferenceObjectCoordinates
  (JNIEnv *, jobject);

/*
 * Class:     com_apm_tobias_automaticpackagemeasuring_core_PackageMeasurer
 * Method:    getPackageCoordinates
 * Signature: ()Ljava/nio/ByteBuffer;
 */
JNIEXPORT jobject JNICALL Java_com_apm_tobias_automaticpackagemeasuring_core_PackageMeasurer_getPackageCoordinates
  (JNIEnv *, jobject);

#ifdef __cplusplus
}
#endif
#endif
