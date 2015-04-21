package com.apm.tobias.automaticpackagemeasuring;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Point;
import android.util.AttributeSet;
import android.view.View;

import java.util.Iterator;
import java.util.List;

public class OverlayView extends View {

    private int mPreviewHeight;
    private int mPreviewWidth;
    private int mRotation;
    private Paint mReferenceObjectPaint;
    private Paint mPackagePaint;
    private Paint mTextPaint;
    private List<Point> mReferenceObject;
    private List<Point> mPackage;
    private List<Float> mDimensions;
    private List<Integer> mMeasuredEdges;
    int measurementTextOffset;

    public OverlayView(Context context, AttributeSet attrs) {
        super(context, attrs);
        mReferenceObjectPaint = new Paint();
        mReferenceObjectPaint.setARGB(255, 255, 0, 0);
        mReferenceObjectPaint.setStrokeWidth(10.0f); // TODO screen size independence

        mPackagePaint = new Paint();
        mPackagePaint.setARGB(255, 0, 0, 255);
        mPackagePaint.setStrokeWidth(10.0f); // TODO screen size independence

        mTextPaint = new Paint();
        mTextPaint.setARGB(255, 0, 0, 0);
        int textSize = context.getResources().getDimensionPixelSize(R.dimen.measurement_text_size);
        measurementTextOffset = context.getResources().getDimensionPixelSize(R.dimen.measurement_text_offset);

        mTextPaint.setTextSize(textSize);

    }

    public void setRotation(int degrees) {
        mRotation = degrees;
    }

    public void setPreviewSize(int height, int width) {
        this.mPreviewHeight = height;
        this.mPreviewWidth = width;
    }

    public void setReferenceObject(List<Point> vertices) {
        this.mReferenceObject = vertices;
    }

    public void setPackage(List<Point> vertices) {
        this.mPackage = vertices;
    }

    public void setDimensions(List<Float> dimensions) {
        this.mDimensions = dimensions;
    }

    public void setMeasuredEdges(List<Integer> edges) {
        this.mMeasuredEdges = edges;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        if (mPreviewHeight == 0 || mPreviewWidth == 0) {
            return;
        }
        drawPolygon(canvas, mReferenceObject, mReferenceObjectPaint);
        drawPolygon(canvas, mPackage, mPackagePaint);
        if (mPackage != null && mReferenceObject != null && !mReferenceObject.isEmpty() && mPackage.size() == 6) {
            drawMeasurement(canvas, mPackage.get(mMeasuredEdges.get(0)), mPackage.get((mMeasuredEdges.get(0) + 1) % 6), mDimensions.get(0), Paint.Align.RIGHT);
            drawMeasurement(canvas, mPackage.get(mMeasuredEdges.get(1)), mPackage.get((mMeasuredEdges.get(1) + 1) % 6), mDimensions.get(1), Paint.Align.LEFT);
            drawMeasurement(canvas, mPackage.get(mMeasuredEdges.get(2)), mPackage.get((mMeasuredEdges.get(2) + 1) % 6), mDimensions.get(2), Paint.Align.LEFT);
        }
    }

    private void drawMeasurement(Canvas canvas, Point p1, Point p2, float measurement, Paint.Align align) {
        p1 = scaleCoordinates(p1);
        p2 = scaleCoordinates(p2);

        float middleX = (p1.x + p2.x) / 2;
        float middleY = (p1.y + p2.y) / 2;

        float dx = p2.x - p1.x;
        float dy = p2.y - p1.y;
        double angle = Math.atan2(dy, dx);
        double offsetAngle = angle > Math.PI / 2 ? angle - Math.PI / 2 : angle + Math.PI / 2;
        float offsetX = (float) (Math.cos(offsetAngle) * measurementTextOffset);
        float offsetY = (float) (Math.sin(offsetAngle) * measurementTextOffset);
        float textX = middleX + offsetX;
        float textY = middleY + offsetY;

        mTextPaint.setTextAlign(align);
        canvas.drawText(String.format("%d mm", (int) Math.round(measurement)), textX, textY, mTextPaint);
    }

    private void drawPolygon(Canvas canvas, List<Point> vertices, Paint paint) {
        if (vertices == null || vertices.isEmpty())
            return;

        Iterator<Point> it = vertices.iterator();
        Point first = scaleCoordinates(it.next());
        Point previous = first;
        while (it.hasNext()) {
            Point current = scaleCoordinates(it.next());
            canvas.drawLine(previous.x, previous.y, current.x, current.y, paint);
            previous = current;
        }
        canvas.drawLine(previous.x, previous.y, first.x, first.y, paint);
    }

    public Point scaleCoordinates(Point p) {
        int width = getWidth();
        int height = getHeight();

        int rotatedWidth;
        int rotatedHeight;
        switch (mRotation) {
            case 90:
                rotatedWidth = mPreviewHeight;
                rotatedHeight = mPreviewWidth;
                break;

            case 180:
                rotatedWidth = mPreviewWidth;
                rotatedHeight = mPreviewHeight;
                break;

            case 270:
                rotatedWidth = mPreviewHeight;
                rotatedHeight = mPreviewWidth;
                break;

            case 0:
            default:
                rotatedWidth = mPreviewWidth;
                rotatedHeight = mPreviewHeight;
                break;
        }

        float scaleX = 1f * width / rotatedWidth;
        float scaleY = 1f * height / rotatedHeight;
        float x = p.x * scaleX;
        float y = p.y * scaleY;
        return new Point(Math.round(x), Math.round(y));
    }

}
