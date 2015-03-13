package com.apm.tobias.automaticpackagemeasuring.core;

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

        Vertex first = null;
        if (data.remaining() > 4)
            first = new Vertex(data.getInt(), data.getInt());
        else
            return edges;
        Vertex previous = first;

        while (data.remaining() > 4) { // while we can read an int
            Vertex current = new Vertex(data.getInt(), data.getInt());
            edges.add(new Edge(previous, current));
            previous = current;
        }
        edges.add(new Edge(previous, first));

        return edges;
    }
}
