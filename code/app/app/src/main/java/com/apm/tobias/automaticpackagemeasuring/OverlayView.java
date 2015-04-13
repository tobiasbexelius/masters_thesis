package com.apm.tobias.automaticpackagemeasuring;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.util.Log;
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
         double yScale = 1;//1920.0/1280;
         double xScale = 1;//1080.0/720;
        for (Edge edge : edges) {
            Vertex v1 = transformCoordinates(canvas, edge.getV1());
            Vertex v2 = transformCoordinates(canvas, edge.getV2());
            canvas.drawLine((int) (v1.getX()*xScale), (int) (v1.getY()*yScale), (int) (v2.getX()*xScale), (int) (v2.getY()*yScale), paint);
        }
    }

    public Vertex transformCoordinates(Canvas c, Vertex v) {
        return new Vertex(c.getWidth() - v.getY(), v.getX());
    }

}
