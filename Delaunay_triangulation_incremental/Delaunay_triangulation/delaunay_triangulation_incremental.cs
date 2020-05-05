using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Delaunay_triangulation
{
    // References
    // Bowyer Watson Algorithm 
    // https://en.wikipedia.org/wiki/Bowyer%E2%80%93Watson_algorithm

    // An implementation of Watson's algorithm for computing 
    // 2-dimensional Delaunay triangulations 
    // https://www.newcastle.edu.au/__data/assets/pdf_file/0018/22482/07_An-implementation-of-Watsons-algorithm-for-computing-two-dimensional-Delaunay-triangulations.pdf

    // Lecture Notes on Delaunay Mesh Generation
    // Jonathan Richard Shewchuk
    // http://web.mit.edu/ehliu/Public/ProjectX/Summer2005/delnotes.pdf
    // Triangle: Engineering a 2D Quality Mesh Generator and Delaunay Triangulator
    // https://people.eecs.berkeley.edu/~jrs/papers/triangle.pdf

    public static class delaunay_triangulation_incremental
    {
        const double eps = 0.00000001; // 10^-8 floating point error
        private static mesh_store main_mesh = new mesh_store();

        public static void delaunay_start(List<Form1.planar_object_store.point2d> input_points,
               ref List<Form1.planar_object_store.edge2d> output_edges,
               ref List<Form1.planar_object_store.face2d> output_triangles,
               ref List<Form1.planar_object_store.instance_tracker> output_edge_tracker)
        {

            // vertex strore intialization
            List<mesh_store.point_store> point_list = new List<mesh_store.point_store>(); // initialize the vertex store delaunay pts

            // Sort by x - axis
            List<Form1.planar_object_store.point2d> sorted_pts = input_points.OrderBy(obj => obj.x).ToList();

            // Transfer the input points to the delaunay pts (very in-efficient but I want the program to be versatille and easy to understand and implement)
            // revise this part to suit your requirements
            int id = 0; // id starts at zero 0
            foreach (Form1.planar_object_store.point2d inpt_pt in sorted_pts)
            {
                point_list.Add(new mesh_store.point_store(id, inpt_pt.x, inpt_pt.y, inpt_pt)); //vert_type = true since its an user input delaunay points
                id++;
            }

            main_mesh = new mesh_store(point_list, ref sorted_pts); // Main call

            // Re-initialize outputs
            output_edges = new List<Form1.planar_object_store.edge2d>();
            output_triangles = new List<Form1.planar_object_store.face2d>();
            output_edge_tracker = new List<Form1.planar_object_store.instance_tracker>();

            output_edges = main_mesh.local_output_edges;
            output_triangles = main_mesh.local_output_triangle;
            output_edge_tracker = main_mesh.local_history_tracker;
        }

        public static void incremental_add_point(Form1.planar_object_store.point2d add_pt,
                                                 ref List<Form1.planar_object_store.point2d> output_points,
                                                 ref List<Form1.planar_object_store.edge2d> output_edges,
                                                 ref List<Form1.planar_object_store.face2d> output_triangles)
        {
            // check the mesh availability
            if (main_mesh.is_meshed == true)
            {
                // Routine to add point by point for constrained delaunay triangulation


            }
        }

        public class mesh_store
        {
            //#### Output Variables #######
            private List<Form1.planar_object_store.point2d> local_input_points = new List<Form1.planar_object_store.point2d>();
            public List<Form1.planar_object_store.edge2d> local_output_edges = new List<Form1.planar_object_store.edge2d>();
            public List<Form1.planar_object_store.face2d> local_output_triangle = new List<Form1.planar_object_store.face2d>();
            public List<Form1.planar_object_store.instance_tracker> local_history_tracker = new List<Form1.planar_object_store.instance_tracker>();
            //#####################################

            // Local Variables
            private List<point_store> _all_points = new List<point_store>();
            private List<edge_store> _all_edges = new List<edge_store>();
            private List<triangle_store> _all_triangles = new List<triangle_store>();
            public bool is_meshed = false;

            private List<int> unique_edgeid_list = new List<int>();
            private List<int> unique_triangleid_list = new List<int>();

            // super triangle points
            private point_store s_p1, s_p2, s_p3;

            public List<triangle_store> all_triangles
            {
                get { return this._all_triangles; }
            }

            public List<edge_store> all_edges
            {
                get { return this._all_edges; }
            }

            public mesh_store()
            {
                // Empty constructor
            }

            public mesh_store(List<point_store> input_vertices, ref List<Form1.planar_object_store.point2d> parent_inpt_points)
            {
                local_input_points = parent_inpt_points;
                // intialize the edges and triangles
                this._all_edges = new List<edge_store>();
                this._all_triangles = new List<triangle_store>();

                // Create an imaginary triangle that encloses all the point set
                set_bounding_triangle(input_vertices);

                foreach (point_store pt in input_vertices)
                {
                    // Find the index of triangle containing this point
                    int containing_triangle_index = this.all_triangles.FindIndex(obj => obj.is_point_inside(pt) == true);


                    point_store tri_a = this.all_triangles[containing_triangle_index].pt1;
                    point_store tri_b = this.all_triangles[containing_triangle_index].pt2;
                    point_store tri_c = this.all_triangles[containing_triangle_index].pt3;

                    remove_triangle(containing_triangle_index);

                    // add the triangle
                    int first_triangle_id, second_triangle_id, third_triangle_id;

                    first_triangle_id = add_triangle(pt, tri_a, tri_b);
                    second_triangle_id = add_triangle(pt, tri_b, tri_c);
                    third_triangle_id = add_triangle(pt, tri_c, tri_a);

                    // Flip the bad triangles recursively
                    flip_bad_edges(first_triangle_id, pt);
                    flip_bad_edges(second_triangle_id, pt);
                    flip_bad_edges(third_triangle_id, pt);

                }

                // Revise output edges and triangles
                local_output_edges = new List<Form1.planar_object_store.edge2d>();
                local_output_triangle = new List<Form1.planar_object_store.face2d>();

                List<edge_store> revised_edges = new List<edge_store>();
                revised_edges = this.all_edges.FindAll(obj => obj.contains_point(this.s_p1) == false && obj.contains_point(this.s_p2) == false && obj.contains_point(this.s_p3) == false).ToList();

                int i = 0;
                foreach (edge_store the_edge in revised_edges)
                {
                    local_output_edges.Add(new Form1.planar_object_store.edge2d(i, the_edge.start_pt.get_parent_data_type, the_edge.end_pt.get_parent_data_type));
                    i++;
                }

                List<triangle_store> revised_tris = new List<triangle_store>();
                revised_tris = this.all_triangles.FindAll(obj => obj.contains_point(this.s_p1) == false && obj.contains_point(this.s_p2) == false && obj.contains_point(this.s_p3) == false).ToList();

                i = 0;
                foreach (triangle_store the_edge in revised_tris)
                {
                    local_output_triangle.Add(new Form1.planar_object_store.face2d(i, the_edge.pt1.get_parent_data_type, the_edge.pt2.get_parent_data_type, the_edge.pt3.get_parent_data_type));
                    i++;
                }

            }

            private void remove_triangle(int tri_index)
            {
                int edge_index1 = this.all_edges.FindLastIndex(obj => obj.edge_id == this.all_triangles[tri_index].e1.edge_id);
                int edge_index2 = this.all_edges.FindLastIndex(obj => obj.edge_id == this.all_triangles[tri_index].e2.edge_id);
                int edge_index3 = this.all_edges.FindLastIndex(obj => obj.edge_id == this.all_triangles[tri_index].e3.edge_id);

                // Edge 1
                if (edge_index1 != -1)
                {
                    this._all_edges[edge_index1].remove_triangle(this.all_triangles[tri_index]);
                }
                // Edge 2
                if (edge_index2 != -1)
                {
                    this._all_edges[edge_index2].remove_triangle(this.all_triangles[tri_index]);
                }
                // Edge 3
                if (edge_index3 != -1)
                {
                    this._all_edges[edge_index3].remove_triangle(this.all_triangles[tri_index]);
                }

                unique_triangleid_list.Add(this.all_triangles[tri_index].tri_id);

                // Remove from the output variables !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                remove_from_local_trianlge_list(this._all_triangles[tri_index]);

                this._all_triangles.RemoveAt(tri_index);
            }

            private int add_triangle(point_store p1, point_store p2, point_store p3)
            {
                int edge_index1 = -1;
                int edge_index2 = -1;
                int edge_index3 = -1;


                // Edge 1
                edge_index1 = this.all_edges.FindLastIndex(obj => obj.Equals_without_orientation(new edge_store(-1, p1, p2)));
                if (edge_index1 == -1)
                {
                    edge_index1 = this.all_edges.Count;
                    this._all_edges.Add(new edge_store(get_unique_edge_id(), p1, p2));

                    // Add to the output variables !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    add_to_local_edge_list(this.all_edges[this.all_edges.Count - 1]);

                }

                // Edge 2
                edge_index2 = this.all_edges.FindLastIndex(obj => obj.Equals_without_orientation(new edge_store(-1, p2, p3)));
                if (edge_index2 == -1)
                {
                    edge_index2 = this.all_edges.Count;
                    this._all_edges.Add(new edge_store(get_unique_edge_id(), p2, p3));

                    // Add to the output variables !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    add_to_local_edge_list(this.all_edges[this.all_edges.Count - 1]);

                }

                // Edge 3
                edge_index3 = this.all_edges.FindLastIndex(obj => obj.Equals_without_orientation(new edge_store(-1, p3, p1)));
                if (edge_index3 == -1)
                {
                    edge_index3 = this.all_edges.Count;
                    this._all_edges.Add(new edge_store(get_unique_edge_id(), p3, p1));

                    // Add to the output variables !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    add_to_local_edge_list(this.all_edges[this.all_edges.Count - 1]);

                }

                // Triangle
                this._all_triangles.Add(new triangle_store(get_unique_triangle_id(), p1, p2, p3, this.all_edges[edge_index1], this.all_edges[edge_index2], this.all_edges[edge_index3]));

                // Update the triangle details to the edge
                this._all_edges[edge_index1].add_triangle(this.all_triangles[this.all_triangles.Count - 1]);
                this._all_edges[edge_index2].add_triangle(this.all_triangles[this.all_triangles.Count - 1]);
                this._all_edges[edge_index3].add_triangle(this.all_triangles[this.all_triangles.Count - 1]);

                // Add to the output variables !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                add_to_local_trianlge_list(this.all_triangles[this.all_triangles.Count - 1]);

                return this.all_triangles[this.all_triangles.Count - 1].tri_id;
            }

            private void add_to_local_trianlge_list(triangle_store t)
            {
                //____________________________________________________________________________________________________________________________________________________________________
                // !! Addition on main list here
                Form1.planar_object_store.face2d temp_face = new Form1.planar_object_store.face2d(t.tri_id, t.pt1.get_parent_data_type, t.pt2.get_parent_data_type, t.pt3.get_parent_data_type);
                local_output_triangle.Add(temp_face);


                // #################################################################
                Form1.planar_object_store.instance_tracker temp_edge_tracker = new Form1.planar_object_store.instance_tracker();
                temp_edge_tracker.edge_list = new List<Form1.planar_object_store.edge2d>();
                temp_edge_tracker.edge_list.AddRange(local_output_edges);
                temp_edge_tracker.face_list = new List<Form1.planar_object_store.face2d>();
                temp_edge_tracker.face_list.AddRange(local_output_triangle);
                local_history_tracker.Add(temp_edge_tracker);
                // #################################################################
            }

            private void add_to_local_edge_list(edge_store e)
            {

                //____________________________________________________________________________________________________________________________________________________________________
                // !! Addition on main list here
                Form1.planar_object_store.edge2d temp_edge = new Form1.planar_object_store.edge2d(e.edge_id, e.start_pt.get_parent_data_type, e.end_pt.get_parent_data_type);
                local_output_edges.Add(temp_edge);
            }

            private void remove_from_local_trianlge_list(triangle_store t)
            {
                //____________________________________________________________________________________________________________________________________________________________________
                // !! Addition on main list here
                Form1.planar_object_store.face2d temp_face = new Form1.planar_object_store.face2d(t.tri_id, t.pt1.get_parent_data_type, t.pt2.get_parent_data_type, t.pt3.get_parent_data_type);
                int temp_id = local_output_triangle.FindLastIndex(obj => obj.face_id == t.tri_id);
                local_output_triangle.RemoveAt(temp_id);
                //local_output_triangle.Remove(temp_face);


                // #################################################################
                Form1.planar_object_store.instance_tracker temp_edge_tracker = new Form1.planar_object_store.instance_tracker();
                temp_edge_tracker.edge_list = new List<Form1.planar_object_store.edge2d>();
                temp_edge_tracker.edge_list.AddRange(local_output_edges);
                temp_edge_tracker.face_list = new List<Form1.planar_object_store.face2d>();
                temp_edge_tracker.face_list.AddRange(local_output_triangle);
                local_history_tracker.Add(temp_edge_tracker);
                // #################################################################
            }

            private void remove_from_local_edge_list(edge_store e)
            {

                //____________________________________________________________________________________________________________________________________________________________________
                // !! Addition on main list here
                Form1.planar_object_store.edge2d temp_edge = new Form1.planar_object_store.edge2d(e.edge_id, e.start_pt.get_parent_data_type, e.end_pt.get_parent_data_type);
                int temp_id = local_output_edges.FindLastIndex(obj => obj.edge_id == e.edge_id);
                local_output_edges.RemoveAt(temp_id);
                //local_output_edges.Remove(temp_edge);
            }

            private void flip_bad_edges(int tri_id, point_store pt)
            {
                // find the triangle with input id
                int tri_index = this.all_triangles.FindLastIndex(obj => obj.tri_id == tri_id);
                // find the edge of this triangle whihc does not contain pt
                int common_edge_index = this.all_edges.FindLastIndex(obj => obj.edge_id == this.all_triangles[tri_index].get_other_edge_id(pt));

                // main method to legalize edges by recursively flipping all the illegal edges
                int neighbour_tri_index = this.all_triangles.FindIndex(obj => obj.tri_id == this.all_edges[common_edge_index].other_triangle_id(this.all_triangles[tri_index]));

                // legalize only if the triangle has a neighbour
                if (neighbour_tri_index != -1)
                {
                    // check whether the newly added pt is inside the neighbour triangle
                    if (this.all_triangles[neighbour_tri_index].is_point_in_circumcircle(pt) == true)
                    {
                        // find the other vertex of the closest triangle
                        point_store neighbour_tri_other_vertex = this.all_triangles[neighbour_tri_index].get_other_pt(this.all_edges[common_edge_index]);
                        point_store edge_a_pt = this.all_edges[common_edge_index].start_pt;
                        point_store edge_b_pt = this.all_edges[common_edge_index].end_pt;

                        // Remove the common edge
                        // remove_edge(the_edge);
                        // int edge_index = this._all_edges.FindLastIndex(obj => obj.edge_id == the_edge.edge_id);
                        unique_edgeid_list.Add(this.all_edges[common_edge_index].edge_id);

                        // Remove from the output variables !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        remove_from_local_edge_list(this.all_edges[common_edge_index]);

                        this._all_edges.RemoveAt(common_edge_index);

                        // Remove the two triangles
                        remove_triangle(tri_index);
                        remove_triangle(neighbour_tri_index);

                        // Add two triangles
                        int first_triangle_id, second_triangle_id;

                        first_triangle_id = add_triangle(pt, neighbour_tri_other_vertex, edge_a_pt);
                        second_triangle_id = add_triangle(pt, neighbour_tri_other_vertex, edge_b_pt);

                        // recursion below
                        flip_bad_edges(first_triangle_id, pt);
                        flip_bad_edges(second_triangle_id, pt);
                    }

                }
            }

            private int get_unique_edge_id()
            {
                int edge_id;
                // get an unique edge id
                if (unique_edgeid_list.Count != 0)
                {
                    edge_id = unique_edgeid_list[0]; // retrive the edge id from the list which stores the id of deleted edges
                    unique_edgeid_list.RemoveAt(0); // remove that id from the unique edge id list
                }
                else
                {
                    edge_id = this.all_edges.Count;
                }
                return edge_id;

            }

            private int get_unique_triangle_id()
            {
                int tri_id;
                // get an unique triangle id
                if (unique_triangleid_list.Count != 0)
                {
                    tri_id = unique_triangleid_list[0]; // retrive the triangle id from the list which stores the id of deleted edges
                    unique_triangleid_list.RemoveAt(0); // remove that id from the unique triangle id list
                }
                else
                {
                    tri_id = this.all_triangles.Count;
                }
                return tri_id;
            }

            private void set_bounding_triangle(List<point_store> all_input_vertices)
            {
                point_store[] x_sorted = all_input_vertices.OrderBy(obj => obj.x).ToArray();
                point_store[] y_sorted = all_input_vertices.OrderBy(obj => obj.y).ToArray();

                // Define bounding triangle
                double max_x, max_y, k;
                max_x = (x_sorted[x_sorted.Length - 1].x - x_sorted[0].x);
                max_y = (y_sorted[y_sorted.Length - 1].y - y_sorted[0].y);
                k = 1000 * Math.Max(max_x, max_y);

                // zeoth _point
                double x_zero, y_zero;
                x_zero = (x_sorted[x_sorted.Length - 1].x + x_sorted[0].x) * 0.5;
                y_zero = (y_sorted[y_sorted.Length - 1].y + y_sorted[0].y) * 0.5;

                // id for the super triangle points
                int pt_count = all_input_vertices.Count;


                // set the vertex
                this.s_p1 = new point_store(pt_count + 1, 0, Math.Round(k / 2.0f), new Form1.planar_object_store.point2d(pt_count + 1, x_zero, k));
                this.s_p2 = new point_store(pt_count + 2, Math.Round(k / 2.0f), 0.0, new Form1.planar_object_store.point2d(pt_count + 2, k, y_zero));
                this.s_p3 = new point_store(pt_count + 3, -1 * Math.Round(k / 2.0f), -1 * Math.Round(k / 2.0f), new Form1.planar_object_store.point2d(pt_count + 3, -k, -k));

                local_input_points.Add(new Form1.planar_object_store.point2d(this.s_p1.pt_id, this.s_p1.x, this.s_p1.y));
                local_input_points.Add(new Form1.planar_object_store.point2d(this.s_p2.pt_id, this.s_p2.x, this.s_p2.y));
                local_input_points.Add(new Form1.planar_object_store.point2d(this.s_p3.pt_id, this.s_p3.x, this.s_p3.y));

                // set the edges
                add_triangle(this.s_p1, this.s_p2, this.s_p3);
            }

            public class point_store
            {
                int _pt_id;
                double _x;
                double _y;
                Form1.planar_object_store.point2d parent_pt;

                public int pt_id
                {
                    get { return this._pt_id; }
                }

                public double x
                {
                    get { return this._x; }
                }

                public double y
                {
                    get { return this._y; }
                }

                public Form1.planar_object_store.point2d get_parent_data_type
                {
                    get { return parent_pt; }
                }

                public point_store(int i_pt_id, double i_x, double i_y, Form1.planar_object_store.point2d pt)
                {
                    // Constructor
                    // set id
                    this._pt_id = i_pt_id;
                    // co-ordinate
                    this._x = i_x;
                    this._y = i_y;
                    // store the output point variable
                    this.parent_pt = pt;
                }

                public bool Equals(point_store other)
                {
                    if (Math.Abs(this._x - other.x) <= eps && Math.Abs(this._y - other.y) <= eps)
                    {
                        return true;
                    }
                    return false;
                }

                // Operators
                public point_store sub(point_store other_pt)
                {
                    double ab_x = this.x - other_pt.x;
                    double ab_y = this.y - other_pt.y;

                    return new point_store(-1, ab_x, ab_y, null);
                }

                public point_store add(point_store other_pt)
                {
                    double ab_x = this.x + other_pt.x;
                    double ab_y = this.y + other_pt.y;

                    return new point_store(-1, ab_x, ab_y, null);
                }

                public double dot(point_store other_pt)
                {
                    return ((this.x * other_pt.x) + (this.y * other_pt.y));
                }

                public point_store mult(double v)
                {
                    return (new point_store(this.pt_id, this.x * v, this.y * v, null));
                }

            }

            public class edge_store
            {
                int _edge_id;
                point_store _start_pt;
                point_store _end_pt;
                triangle_store _left_triangle;
                triangle_store _right_triangle;

                public int edge_id
                {
                    get { return this._edge_id; }
                }

                public point_store start_pt
                {
                    get { return this._start_pt; }
                }

                public point_store end_pt
                {
                    get { return this._end_pt; }
                }

                public edge_store sym
                {
                    get { return new edge_store(this._edge_id, this._end_pt, this._start_pt); }
                }

                public edge_store(int i_edge_id, point_store i_start_pt, point_store i_end_pt)
                {
                    // Constructor
                    // set id
                    this._edge_id = i_edge_id;
                    // set start and end pt
                    this._start_pt = i_start_pt;
                    this._end_pt = i_end_pt;

                    // set triangles to null
                    this._left_triangle = null;
                    this._right_triangle = null;
                }

                public bool contains_point(point_store the_point)
                {
                    // find whether the point belongs to the triangle
                    if (the_point.Equals(this._start_pt) == true ||
                        the_point.Equals(this._end_pt) == true)
                    {
                        return true;
                    }
                    return false;
                }

                public int other_triangle_id(triangle_store the_triangle)
                {
                    // Function to return the other triangle associated with this edge
                    if (this._left_triangle != null)
                    {
                        if (the_triangle.Equals(this._left_triangle) == true)
                        {
                            if (this._right_triangle == null)
                            {
                                return -1;
                            }
                            return this._right_triangle.tri_id;
                        }
                    }

                    if (this._right_triangle != null)
                    {
                        if (the_triangle.Equals(this._right_triangle) == true)
                        {
                            if (this._left_triangle == null)
                            {
                                return -1;
                            }
                            return this._left_triangle.tri_id;
                        }
                    }

                    return -1;

                }

                public void add_triangle(triangle_store the_triangle)
                {
                    // check whether the input triangle has this edge
                    if (the_triangle.contains_edge(this) == true)
                    {
                        if (rightof(the_triangle.mid_pt, this) == true)
                        {
                            // Add the right triangle
                            this._right_triangle = the_triangle;
                        }
                        else
                        {
                            // Add the left triangle
                            this._left_triangle = the_triangle;
                        }
                    }
                }

                public void remove_triangle(triangle_store the_triangle)
                {
                    // check whether the input triangle has this edge
                    if (the_triangle.contains_edge(this) == true)
                    {
                        if (rightof(the_triangle.mid_pt, this) == true)
                        {
                            // Remove the right triangle
                            this._right_triangle = null;
                        }
                        else
                        {
                            // Remove the left triangle
                            this._left_triangle = null;
                        }
                    }

                }

                private bool ccw(point_store a, point_store b, point_store c)
                {
                    // Computes | a.x a.y  1 |
                    //          | b.x b.y  1 | > 0
                    //          | c.x c.y  1 |
                    return (((b.x - a.x) * (c.y - a.y)) - ((b.y - a.y) * (c.x - a.x))) > 0;
                }

                private bool rightof(point_store x, edge_store e)
                {
                    return ccw(x, e.end_pt, e.start_pt);
                }

                public bool Equals(edge_store other)
                {
                    if (other.start_pt.Equals(this._start_pt) == true && other.end_pt.Equals(this._end_pt) == true)
                    {
                        return true;
                    }
                    return false;
                }

                public bool Equals_without_orientation(edge_store other)
                {
                    if ((other.Equals(this) == true) || (other.Equals(this.sym) == true))
                    {
                        return true;
                    }
                    return false;
                }

                public double find_closest_distance(point_store pt)
                {
                    point_store ab = end_pt.sub(start_pt); // ab_x = x2 - x1,  ab_y = y2 - y1  (end_pt - start_pt)
                    double ab_dot = ab.dot(ab); // ab_x*ab_x + ab_y*ab_y

                    // double u=((x3 - x1) * px + (y3 - y1) * py) / ((px*px)+(py*py));
                    double u = (pt.sub(start_pt)).dot(ab) / ab_dot;

                    if (u < 0.0)
                    {
                        u = 0.0;
                    }
                    else if (u > 1.0)
                    {
                        u = 1.0;
                    }

                    // closest point on the edge from the input pt
                    point_store c = start_pt.add(ab.mult(u));
                    // vector connecting the input pt and the pt on line
                    point_store pt_to_c = c.sub(pt);

                    return (pt_to_c.dot(pt_to_c));
                }

                public point_store find_common_pt(edge_store other_edge)
                {
                    // Returns the common point of this edge and the other edges
                    if (this.start_pt.Equals(other_edge.start_pt))
                    {
                        return this.start_pt;
                    }
                    else if (this.start_pt.Equals(other_edge.end_pt))
                    {
                        return this.start_pt;
                    }
                    else if (this.end_pt.Equals(other_edge.start_pt))
                    {
                        return this.end_pt;
                    }
                    else if (this.end_pt.Equals(other_edge.end_pt))
                    {
                        return this.end_pt;
                    }
                    return null;
                }

                public point_store find_other_pt(point_store pt)
                {
                    if (this.start_pt.Equals(pt) == true)
                    {
                        return this.end_pt;
                    }
                    else if (this.end_pt.Equals(pt) == true)
                    {
                        return this.start_pt;
                    }

                    return null;
                }


            }

            public class triangle_store
            {
                int _tri_id;
                point_store _pt1;
                point_store _pt2;
                point_store _pt3;
                point_store _mid_pt;
                edge_store _e1; // from pt1 to pt2
                edge_store _e2; // from pt2 to pt3
                edge_store _e3; // from pt3 to pt1

                public int tri_id
                {
                    get { return this._tri_id; }
                }

                public point_store pt1
                {
                    get { return this._pt1; }
                }

                public point_store pt2
                {
                    get { return this._pt2; }
                }

                public point_store pt3
                {
                    get { return this._pt3; }
                }

                public edge_store e1
                {
                    get { return this._e1; }
                }

                public edge_store e2
                {
                    get { return this._e2; }
                }

                public edge_store e3
                {
                    get { return this._e3; }
                }

                public point_store mid_pt
                {
                    // return the mid point
                    get { return this._mid_pt; }
                }

                public triangle_store(int i_tri_id, point_store i_p1, point_store i_p2, point_store i_p3, edge_store i_e1, edge_store i_e2, edge_store i_e3)
                {
                    // Constructor
                    // set id
                    this._tri_id = i_tri_id;
                    // set points
                    this._pt1 = i_p1;
                    this._pt2 = i_p2;
                    this._pt3 = i_p3;
                    // set edges
                    this._e1 = i_e1;
                    this._e2 = i_e2;
                    this._e3 = i_e3;
                    // set the mid point
                    this._mid_pt = new point_store(-1, (this._pt1.x + this._pt2.x + this._pt3.x) / 3.0f, (this._pt1.y + this._pt2.y + this._pt3.y) / 3.0f, null);

                }

                public bool contains_point(point_store the_point)
                {
                    // find whether the point belongs to the triangle
                    if (the_point.Equals(this._pt1) == true ||
                        the_point.Equals(this._pt2) == true ||
                        the_point.Equals(this._pt3) == true)
                    {
                        return true;
                    }
                    return false;
                }

                public point_store get_other_pt(edge_store the_edge)
                {
                    // Returns the third vertex of this triangle which is not part of the given edge
                    if (the_edge.Equals_without_orientation(this.e1) == true)
                    {
                        return (this.e2.find_common_pt(this.e3));
                    }
                    else if (the_edge.Equals_without_orientation(this.e2) == true)
                    {
                        return (this.e3.find_common_pt(this.e1));
                    }
                    else if (the_edge.Equals_without_orientation(this.e3) == true)
                    {
                        return (this.e1.find_common_pt(this.e2));
                    }
                    return null;
                }

                public int get_other_edge_id(point_store pt)
                {
                    if (this.e1.start_pt.Equals(pt) == false && this.e1.end_pt.Equals(pt) == false)
                    {
                        return this.e1.edge_id;
                    }
                    else if (this.e2.start_pt.Equals(pt) == false && this.e2.end_pt.Equals(pt) == false)
                    {
                        return this.e2.edge_id;
                    }
                    else if (this.e3.start_pt.Equals(pt) == false && this.e3.end_pt.Equals(pt) == false)
                    {
                        return this.e3.edge_id;
                    }

                    return -1;
                }


                public bool contains_edge(edge_store the_edge)
                {
                    // find whether the edge belongs to the triangle
                    if (the_edge.Equals_without_orientation(this._e1) == true ||
                        the_edge.Equals_without_orientation(this._e2) == true ||
                        the_edge.Equals_without_orientation(this._e3) == true)
                    {
                        return true;
                    }
                    return false;
                }

                // Find the area of a triangle.This function uses the 1/2 determinant
                // method. Given three points (x1, y1), (x2, y2), (x3, y3):
                //              | x1 y1 1 |
                // Area = 0.5 * | x2 y2 1 |
                //              | x3 y3 1 |
                public double calc_det(point_store p1, point_store p2, point_store p3)
                {
                    return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
                }

                public bool is_point_inside(point_store the_pt)
                {
                    double d1, d2, d3;
                    bool has_neg, has_pos;

                    d1 = calc_det(the_pt, this._pt1, this._pt2);
                    d2 = calc_det(the_pt, this._pt2, this._pt3);
                    d3 = calc_det(the_pt, this._pt3, this._pt1);

                    has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
                    has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

                    return !(has_neg && has_pos);
                }

                // Tests if a given point lies in the circumcircle of this triangle.Let the
                // triangle ABC appear in counterclockwise (CCW) order. Then when det &gt;
                // 0, the point lies inside the circumcircle through the three points a, b
                // and c.If instead det &lt; 0, the point lies outside the circumcircle.
                // When det = 0, the four points are cocircular.If the triangle is oriented
                // clockwise (CW) the result is reversed.See Real-Time Collision Detection,
                // chap. 3, p. 34.

                public bool is_point_in_circumcircle(point_store the_pt)
                {
                    double a11 = pt1.x - the_pt.x;
                    double a21 = pt2.x - the_pt.x;
                    double a31 = pt3.x - the_pt.x;

                    double a12 = pt1.y - the_pt.y;
                    double a22 = pt2.y - the_pt.y;
                    double a32 = pt3.y - the_pt.y;

                    double a13 = (pt1.x - the_pt.x) * (pt1.x - the_pt.x) + (pt1.y - the_pt.y) * (pt1.y - the_pt.y);
                    double a23 = (pt2.x - the_pt.x) * (pt2.x - the_pt.x) + (pt2.y - the_pt.y) * (pt2.y - the_pt.y);
                    double a33 = (pt3.x - the_pt.x) * (pt3.x - the_pt.x) + (pt3.y - the_pt.y) * (pt3.y - the_pt.y);

                    double det = (a11 * a22 * a33 + a12 * a23 * a31 + a13 * a21 * a32 - a13 * a22 * a31 - a12 * a21 * a33 - a11 * a23 * a32);

                    return ((is_oriented_ccw() == true) ? det > 0.0 : det < 0.0);
                }


                private bool is_oriented_ccw()
                {
                    double a11 = pt1.x - pt3.x;
                    double a21 = pt2.x - pt3.x;

                    double a12 = pt1.y - pt3.y;
                    double a22 = pt2.y - pt3.y;

                    double det = a11 * a22 - a12 * a21;

                    return det > 0.0;
                }


                public bool Equals(triangle_store the_triangle)
                {
                    // find the triangle equals
                    if (this.contains_edge(the_triangle.e1) == true &&
                       this.contains_edge(the_triangle.e2) == true &&
                       this.contains_edge(the_triangle.e3) == true)
                    {
                        return true;
                    }
                    return false;
                }

            }

        }
    }
}
