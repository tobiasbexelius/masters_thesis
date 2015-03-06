package com.apm.tobias.automaticpackagemeasuring.core;

import android.util.Log;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

public class ReferenceObject {

    private ByteBuffer data;

    public ReferenceObject(ByteBuffer data) {
        this.data = data;
    }

    public List<Edge> getEdges() {
        List<Edge> edges = new ArrayList<>();
        data.rewind();
        if (!data.hasRemaining())
            return edges;
        Vertex first = new Vertex(data.getInt(), data.getInt());
        Vertex previous = first;

        for (int i = 0; i < 3; i++) {
            Vertex current = new Vertex(data.getInt(), data.getInt());
            edges.add(new Edge(previous, current));
            previous = current;
        }
        edges.add(new Edge(previous, first));

        return edges;
    }
}
