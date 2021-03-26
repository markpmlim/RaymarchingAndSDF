This set of Swift Playgrounds are developed by porting the pixel shader code developed by Jamie Wong and listed on www.shadertoy.com. 

Shadertoy.com has an online shader creation tool based on WebGL2 and GLSL 330; no polygons are used.


The pixel shader codes have been converted to macOS' Metal Shading Language. The required uniforms
iTime, iMouse are passed to be kernel function as u_time and u_mouse respectively. The uniform u_resolution can be calculate from the size of the drawable texture.


Metal's 2D pixel coordinate system has its origin (0.0, 0.0) at the top left corner of the display area. 


Note: if the MSL shader code is compiled and run in an XCode Project, animation is likely to be too fast.
The MSL code can be run with a vertex-fragment function pair without modification.


Runtime requirements: Xcode 9.1 or later

System requirements: macOS 10.13 or later

Links:

http://jamie-wong.com/2016/07/15/ray-marching-signed-distance-functions/


https://iquilezles.org/www/articles/raymarchingdf/raymarchingdf.htm