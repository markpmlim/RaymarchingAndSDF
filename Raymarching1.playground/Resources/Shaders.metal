// https://www.shadertoy.com/view/llt3R4

#include <metal_stdlib>

using namespace metal;

/**
 * Ray Marching: Part 1
 * Challenges
 * - Make the circle yellow
 * - Make the circle smaller by decreasing its radius
 * - Make the circle smaller by moving the camera back
 * - Make the size of the circle oscillate using the sin() function and the iTime
 *   uniform provided by shadertoy
 */

constant const int MAX_MARCHING_STEPS = 255;
constant const float MIN_DIST = 0.0;
constant const float MAX_DIST = 100.0;
constant const float EPSILON = 0.0001;

/**
 * Signed distance function for a sphere centered at the origin with radius 1.0;
 */
float sphereSDF(float3 samplePoint) {
    return length(samplePoint) - 1.0;
}

/**
 * Signed distance function describing the scene.
 * 
 * Absolute value of the return value indicates the distance to the surface.
 * Sign indicates whether the point is inside or outside the surface,
 * negative indicating inside.
 */
float sceneSDF(float3 samplePoint) {
    return sphereSDF(samplePoint);
}

/**
 * Return the shortest distance from the eyepoint to the scene surface along
 * the marching direction. If no part of the surface is found between start and end,
 * return end.
 * 
 * eye: the eye point, acting as the origin of the ray
 * marchingDirection: the normalized direction to march in
 * start: the starting distance away from the eye
 * end: the max distance away from the ey to march before giving up
 */
float shortestDistanceToSurface(float3 eye, float3 marchingDirection,
                                float start, float end) {
    float depth = start;
    for (int i = 0; i < MAX_MARCHING_STEPS; i++) {
        float dist = sceneSDF(eye + depth * marchingDirection);
        if (dist < EPSILON) {
            // Our ray hit something.
			return depth;
        }
        // Use the returned value of "dist" to step up the distance to the surface.
        depth += dist;
        if (depth >= end) {
            return end;
        }
    } // for
    return end;
}

float radians(float angle) {
    float A = angle/180 * M_PI_F;
    return A;
}
/**
 * Return the normalized direction to march in from the eye point for a single pixel.
 * 
 * fieldOfView: vertical field of view in degrees
 * size: resolution of the output image
 * fragCoord: the x,y coordinate of the pixel in the output image
 * [0, w) --> [-w/2, w/2)
 * [0, h) --> [-h/2, w/2)
 */
float3 rayDirection(float fieldOfView, float2 size, float2 fragCoord) {
    float2 xy = fragCoord - size / 2.0;
    // Apply trigonometry to calculate the depth z.
    // length of opposite side/length of adjacent side = tan(ø/2)
    float z = size.y / tan(radians(fieldOfView) / 2.0);
    // The ray is pointing in the direction of -ve z-axis
    return normalize(float3(xy, -z));
}


kernel void
compute(texture2d<float, access::write>     output   [[texture(0)]],
        constant float                      &u_time  [[buffer(0)]],
        constant float2                     &u_mouse [[buffer(1)]],
        uint2                               gid      [[thread_position_in_grid]])
{
    uint width = output.get_width();
    uint height = output.get_height();
    uint col = gid.x;
    uint row = gid.y;
    if ((col >= width) || (row >= height)) {
        // In case the size of the texture does not match the size of the grid.
        // Return early if the pixel is out of bounds
        return;
    }
    float2 u_resolution = float2(width, height);
    // Metal's 2D pixel coord system has (0.0, 0.0) at the top left.
    float2 fragCoord = float2(gid.x, height-gid.y);

    // Get the normalized ray direction.
	float3 dir = rayDirection(45.0, u_resolution.xy, fragCoord);
    // Camera/eye position.
    float3 eye = float3(0.0, 0.0, 5.0);
    float dist = shortestDistanceToSurface(eye, dir, MIN_DIST, MAX_DIST);

    float4 fragColor;
    
    if (dist > MAX_DIST - EPSILON) {
        // Didn't hit anything
        fragColor = float4(0.0);
    }
    else {
        fragColor = float4(1.0, 0.0, 0.0, 0.0);
    }
    output.write(fragColor, gid);
}
