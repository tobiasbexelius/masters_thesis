package com.apm.tobias.automaticpackagemeasuring;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.View;

import com.apm.tobias.automaticpackagemeasuring.core.Edge;
import com.apm.tobias.automaticpackagemeasuring.core.Vertex;

import java.util.List;

public class OverlayView extends View {

    private Paint mReferenceObjectPaint;
    private Paint mPackagePaint;
    private List<Edge> referenceObjectEdges;
    private List<Edge> packageEdges;

    public OverlayView(Context context, AttributeSet attrs) {
        super(context, attrs);
        mReferenceObjectPaint = new Paint();
        mReferenceObjectPaint.setARGB(255, 255, 0, 0);
        mReferenceObjectPaint.setStrokeWidth(10.0f); // TODO screen size independence

        mPackagePaint = new Paint();
        mPackagePaint.setARGB(255, 0, 0, 255);
        mPackagePaint.setStrokeWidth(10.0f); // TODO screen size independence
    }

    public void clearReferenceObject() {
        this.referenceObjectEdges = null;
    }

    public void clearPackage() {
        this.packageEdges = null;
    }

    public void updateReferenceObject(List<Edge> edges) {
        this.referenceObjectEdges = edges;

    }

    public void updatePackage(List<Edge> edges) {
        this.packageEdges = edges;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        if (referenceObjectEdges != null)
            drawObject(canvas, referenceObjectEdges, mReferenceObjectPaint);

        if (packageEdges != null)
            drawObject(canvas, packageEdges, mPackagePaint);
    }

    private void drawObject(Canvas canvas, List<Edge> edges, Paint paint) {
        for (Edge edge : edges) {
            Vertex v1 = transformCoordinates(canvas, edge.getV1());
            Vertex v2 = transformCoordinates(canvas, edge.getV2());
            canvas.drawLine(v1.getX(), v1.getY(), v2.getX(), v2.getY(), paint);
        }
    }

    public Vertex transformCoordinates(Canvas c, Vertex v) {
        return new Vertex(c.getWidth() - v.getY(), v.getX());
    }

}
