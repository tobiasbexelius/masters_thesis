package com.apm.tobias.automaticpackagemeasuring;

import android.graphics.Point;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

public class PackageMeasurer {

    public PackageMeasurer() {
        setReferenceObjectSize(297/2f, 210);
    }

    public native void setReferenceObjectSize(float width, float height);

    public native void analyzeVideoFrame(byte[] data, int width, int height, int rotation);

    private native ByteBuffer getReferenceObjectCoordinates();

    private native ByteBuffer getPackageCoordinates();

    private native ByteBuffer getPackageDimensions();

    private native ByteBuffer getPackageMeasuredEdges();

    public List<Point> getReferenceObject() {
        ByteBuffer buffer = getReferenceObjectCoordinates();
        return extractVertices(buffer);
    }

    public List<Point> getPackage() {
        ByteBuffer buffer = getPackageCoordinates();
        return extractVertices(buffer);
    }

    public List<Float> getDimensions() {
        ByteBuffer buffer = getPackageDimensions();
        buffer.order(ByteOrder.LITTLE_ENDIAN);

        List<Float> dimensions = new ArrayList<>();
        while (buffer.remaining() >= 4) { // while one float remains
            dimensions.add(buffer.getFloat());
        }
        return dimensions;
    }

    public List<Integer> getMeasuredEdges() {
        ByteBuffer buffer = getPackageMeasuredEdges();
        buffer.order(ByteOrder.LITTLE_ENDIAN);
        List<Integer> measuredEdges = new ArrayList<>();
        while (buffer.remaining() >= 4) { // while one int remains
            measuredEdges.add(buffer.getInt());
        }
        return measuredEdges;
    }

    private List<Point> extractVertices(ByteBuffer buffer) {
        buffer.order(ByteOrder.LITTLE_ENDIAN);
        List<Point> vertices = new ArrayList<>();
        while (buffer.remaining() >= 8) { // while two ints remain
            vertices.add(new Point(buffer.getInt(), buffer.getInt()));
        }

        return vertices;
    }

    static {
        System.loadLibrary("gnustl_shared");
        System.loadLibrary("opencv_java");
        System.loadLibrary("com_apm_tobias_automaticpackagemeasuring_core_PackageMeasurer");
    }
}
