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

    private Paint mPaint;
    private List<Edge> edges;

    public OverlayView(Context context, AttributeSet attrs) {
        super(context, attrs);
        mPaint = new Paint();
        mPaint.setARGB(255, 255, 0, 0);
        mPaint.setStrokeWidth(10.0f); // TODO screen size independence
    }

    public void clear() {
        this.edges = null;
    }

    public void updateEdges(List<Edge> edges) {
        this.edges = edges;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        if (edges != null) {
            for (Edge edge : edges) {
                Vertex v1 = transformCoordinates(canvas, edge.getV1());
                Vertex v2 = transformCoordinates(canvas, edge.getV2());

                canvas.drawLine(v1.getX(), v1.getY(), v2.getX(), v2.getY(), mPaint);
            }
        }
    }

    public Vertex transformCoordinates(Canvas c, Vertex v) {
        return new Vertex(c.getWidth() - v.getY(), v.getX());
    }

}
