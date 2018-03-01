using System;
using System.Collections.Generic;
using TriangleNet.Geometry;
using TriangleNet.Topology;

/*
 Fast-enough triangle finder for voronoi terrain.
 Works by binning each vertex into a grid and noting down which triangles each vertex
 participates in. When a find operation is executed on an arbitrary point, it finds nearby
 vertices and checks if the point is contained within the triangles those vertices
 participate in.
 */
public class TriangleBin
{
    float binsize;
    private List<Vertex>[,] grid;
    private List<List<Triangle>> vertToTriangle = new List<List<TriangleNet.Topology.Triangle>>();

    /*
     @brief Initializer.
     @param mesh The mesh which we are going to find triangles within.
     @param xsize The maximum x value in the mesh. TODO: Use the mesh to figure this out.
     @param ysize The maximum y value in the mesh. TODO: Use the mesh to figure this out.
     @param binsize How large each grid tile is. This param is pretty important; larger means
        the find operation gets slower, but smaller means the find operation might not find
        the right triangle at all.
     */
    public TriangleBin(TriangleNet.Mesh mesh, int xsize, int ysize, float binsize) {
        this.binsize = binsize;
        this.grid = new List<Vertex>[(int)Math.Ceiling(xsize / binsize), (int)Math.Ceiling(ysize / binsize)];

        foreach (Vertex vertex in mesh.Vertices) {
            vertToTriangle.Add(new List<Triangle>());
            addVertex(vertex);
        }

        foreach (Triangle triangle in mesh.Triangles) {
            addTriangle(triangle);
        }
    }

    /* Notes each vertex in the triangle as participating in that triangle. */
    private void addTriangle(Triangle triangle) {
        vertToTriangle[triangle.vertices[0].id].Add(triangle);
        vertToTriangle[triangle.vertices[1].id].Add(triangle);
        vertToTriangle[triangle.vertices[2].id].Add(triangle);
    }

    /* Adds a vertex to the correct grid tile. */
    private void addVertex(Vertex vertex) {
        int binX = (int)Math.Floor(vertex.x / binsize);
        int binY = (int)Math.Floor(vertex.y / binsize);

        if (grid[binX, binY] == null) {
            grid[binX, binY] = new List<Vertex>();
        }

        grid[binX, binY].Add(vertex);
    }

    /* Gets the triangle which a point belongs to. Returns null if no triangle was found. */
    public Triangle getTriangleForPoint(Point point) {
        int binX = (int)Math.Floor(point.x / binsize);
        int binY = (int)Math.Floor(point.y / binsize);

        for (int x = -1; x < 1; x++) {
            for (int y = -1; y < 1; y++) {
                if (binX + x <= 0 || binX + x > grid.GetLength(0) ||
                    binY + y <= 0 || binY + y > grid.GetLength(1)) {
                    continue;
                }

                List<Vertex> bin = grid[binX + x, binY + y];
                if (bin == null) {
                    continue;
                }

                foreach (Vertex vertex in bin) {
                    List<Triangle> candidateTriangles = vertToTriangle[vertex.id];

                    foreach (Triangle triangle in candidateTriangles) {
                        if (triangle.Contains(point)) {
                            return triangle;
                        }
                    }
                }
            }
        }

        return null;
    }
}