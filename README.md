This is my homework for the CG class, so it's not that well optimized due to the time limitation. But you might still take it as reference.

The code is mainly based on the c++ STL and implemented all the 3D math calculations with it. As for the data structure and some basic calculations, I refered to Kevin Beason's "smallpt: Global Illumination in 99 lines of C++".
http://www.kevinbeason.com/smallpt/#mods

Additionally, I used OpenCV to write image files, and used "tinyobjloader" for loading ".obj" and ".mtl" files.
https://github.com/syoyo/tinyobjloader

I used Bounding Boxes recursively divided similar to an Octree as a accelerate structure. It substantially increased the rendering speed in most cases. For the "cbox" scene, the algorithm runs in about 1 SPP per minute (1 thread) with 512 * 512 resolution.

Some other features:
	2 * 2 ramdom sampled subpixels for Anti-Aliasing
	two possible methods for implementing glossy reflection, changing reflect direction or changing reflect color
	ideal fresnel refraction