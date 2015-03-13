package com.apm.tobias.automaticpackagemeasuring.core;

import android.util.Log;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class PackageMeasurer {

    public native void analyzeVideoFrame(byte[] data, int width, int height);

    private native ByteBuffer getReferenceObjectCoordinates();

    private native ByteBuffer getPackageCoordinates();

    public ReferenceObject getReferenceObject() {
        ByteBuffer referenceObject = getReferenceObjectCoordinates();
        referenceObject.order(ByteOrder.LITTLE_ENDIAN);

        ReferenceObject object = new ReferenceObject(referenceObject);
        return object;
    }

    public ReferenceObject getPackage() {
        ByteBuffer referenceObject = getPackageCoordinates();
        referenceObject.order(ByteOrder.LITTLE_ENDIAN);

        ReferenceObject object = new ReferenceObject(referenceObject);
        return object;
    }

    static {
        System.loadLibrary("gnustl_shared");
        System.loadLibrary("opencv_java");
        System.loadLibrary("com_apm_tobias_automaticpackagemeasuring_core_PackageMeasurer");
    }
}
