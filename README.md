# Delaunay triangulation using Incremental Algorithm
This is a C# implementation of Bowyer-Watson Delaunay triangulation. The implementation follows incremental algorithm with each points added, the edges & triangles are recursively flipped to staisfy the circum circle condition.<br /><br />
![](/Delaunay_triangulation_incremental/Images/Bowyer_Watson_Incremental_gif_1.gif)<br /><br />
![](/Delaunay_triangulation_incremental/Images/incremental_delaunay_mesh.png)
![](/Delaunay_triangulation_incremental/Images/incremental_delaunay_mesh2.png)

# References
•	Bowyer Watson Algorithm<br />
 https://en.wikipedia.org/wiki/Bowyer%E2%80%93Watson_algorithm

•	An implementation of Watson's algorithm for computing 2-dimensional Delaunay triangulations <br />
https://www.newcastle.edu.au/data/assets/pdf_file/0018/22482/07_An-implementation-of-Watsons-algorithm-for-computing-two-dimensional-Delaunay-triangulations.pdf

•	Lecture Notes on Delaunay Mesh Generation<br />
Jonathan Richard Shewchuk<br />
http://web.mit.edu/ehliu/Public/ProjectX/Summer2005/delnotes.pdf

•	Triangle: Engineering a 2D Quality Mesh Generator and Delaunay Triangulator<br />
https://people.eecs.berkeley.edu/~jrs/papers/triangle.pdf
