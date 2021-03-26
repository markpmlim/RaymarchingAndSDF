// https://www.shadertoy.com/view/lt33z7

#include <metal_stdlib>

using namespace metal;

/**
 * Ray Marching: Part 2
 * Challenges
 * - Change the diffuse color of the sphere to be blue
 * - Change the specual color of the sphere to be green
 * - Make one of the lights pulse by having its intensity vary over time
 * - Add a third light to the scene
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
 * end: the max distance away from the eye to march before giving up
 */
float shortestDistanceToSurface(float3 eye, float3 marchingDirection, float start, float end) {
    float depth = start;
    for (int i = 0; i < MAX_MARCHING_STEPS; i++) {
        float dist = sceneSDF(eye + depth * marchingDirection);
        if (dist < EPSILON) {
            // We're inside the scene surface!
			return depth;
        }
        // Move along the view ray
        depth += dist;
        if (depth >= end) {
            // Gone too far; give up
            return end;
        }
    }
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
 */
float3 rayDirection(float fieldOfView, float2 size, float2 fragCoord) {
    float2 xy = fragCoord - size / 2.0;
    float z = size.y / tan(radians(fieldOfView) / 2.0);
    return normalize(float3(xy, -z));
}

/**
 * Using the gradient of the SDF, estimate the normal on the surface at point p.
 */
float3 estimateNormal(float3 p) {
    return normalize(float3(
        sceneSDF(float3(p.x + EPSILON, p.y, p.z)) - sceneSDF(float3(p.x - EPSILON, p.y, p.z)),
        sceneSDF(float3(p.x, p.y + EPSILON, p.z)) - sceneSDF(float3(p.x, p.y - EPSILON, p.z)),
        sceneSDF(float3(p.x, p.y, p.z  + EPSILON)) - sceneSDF(float3(p.x, p.y, p.z - EPSILON))
    ));
}

/**
 * Lighting contribution of a single point light source via Phong illumination.
 * 
 * The float3 returned is the RGB color of the light's contribution.
 *
 * k_a: Ambient color
 * k_d: Diffuse color
 * k_s: Specular color
 * alpha: Shininess coefficient
 * p: position of point being lit
 * eye: the position of the camera
 * lightPos: the position of the light
 * lightIntensity: color/intensity of the light
 *
 * See https://en.wikipedia.org/wiki/Phong_reflection_model#Description
 */
float3 phongContribForLight(float3 k_d, float3 k_s, float alpha,
                            float3 p, float3 eye,
                            float3 lightPos, float3 lightIntensity) {

    float3 N = estimateNormal(p);           // normal vector
    float3 L = normalize(lightPos - p);     // vector from p towards light source
    float3 V = normalize(eye - p);          // vector from p towards camera
    float3 R = normalize(reflect(-L, N));   // reflected ray

    float dotLN = dot(L, N);
    float dotRV = dot(R, V);

    if (dotLN < 0.0) {
        // Light not visible from this point on the surface
        return float3(0.0, 0.0, 0.0);
    } 

    if (dotRV < 0.0) {
        // Light reflection in opposite direction as viewer,
        // apply diffuse contribution only
        return lightIntensity * (k_d * dotLN);
    }
    return lightIntensity * (k_d * dotLN + k_s * pow(dotRV, alpha));
}

/**
 * Lighting via Phong illumination.
 * 
 * The float3 returned is the RGB color of that point after lighting is applied.
 * k_a: Ambient color
 * k_d: Diffuse color
 * k_s: Specular color
 * alpha: Shininess coefficient
 * p: position of point being lit
 * eye: the position of the camera
 *
 * See https://en.wikipedia.org/wiki/Phong_reflection_model#Description
 */
float3 phongIllumination(float3 k_a, float3 k_d, float3 k_s, float alpha,
                         float3 p, float3 eye, float iTime) {
    // Compute ambient light as half of white light
    const float3 ambientLight = 0.5 * float3(1.0, 1.0, 1.0);
    float3 color = ambientLight * k_a;

    // The light source is rotating on a xz-plane at a distance 2.0 above origin.
    float3 light1Pos = float3(4.0 * sin(iTime),
                              2.0,
                              4.0 * cos(iTime));
    float3 light1Intensity = float3(0.4, 0.4, 0.4);

    color += phongContribForLight(k_d, k_s, alpha, p, eye,
                                  light1Pos,
                                  light1Intensity);

    // The light source is rotating on a xy-plane at a distance 2.0 from origin.
    float3 light2Pos = float3(2.0 * sin(0.37 * iTime),
                              2.0 * cos(0.37 * iTime),
                              2.0);
    float3 light2Intensity = float3(0.4, 0.4, 0.4);

    color += phongContribForLight(k_d, k_s, alpha, p, eye,
                                  light2Pos,
                                  light2Intensity);    
    return color;
}


kernel void
compute(texture2d<float, access::write>     output   [[texture(0)]],
        texture2d<float, access::sample>    input    [[texture(1)]],
        constant float                      &u_time  [[buffer(0)]],
        constant float2                     &u_mouse [[buffer(1)]],
        uint2                               gid      [[thread_position_in_grid]])
{
    uint width = output.get_width();
    uint height = output.get_height();
    uint col = gid.x;
    uint row = gid.y;
    if ((col >= width) || (row >= height))
    {
        // In case the size of the texture does not match the size of the grid.
        // Return early if the pixel is out of bounds
        return;
    }
    float2 u_resolution = float2(width, height);
    // Metal's 2D pixel coord system has (0,0) at the upper left corner.
    float2 fragCoord = float2(gid.x, height-gid.y);

    float3 dir = rayDirection(45.0, u_resolution.xy, fragCoord);
    float3 eye = float3(0.0, 0.0, 5.0);
    float dist = shortestDistanceToSurface(eye, dir, MIN_DIST, MAX_DIST);

    float4 fragColor;
    if (dist > MAX_DIST - EPSILON) {
        // Didn't hit anything
        fragColor = float4(0);
    }
    else {
        // The closest point on the surface to the eyepoint along the view ray
        float3 p = eye + dist * dir;

        float3 K_a = float3(0.2, 0.2, 0.2); // coefficents of the
        float3 K_d = float3(0.7, 0.2, 0.2); //  lighting contributions
        float3 K_s = float3(1.0, 1.0, 1.0);
        float shininess = 10.0;

        float3 color = phongIllumination(K_a, K_d, K_s, shininess,
                                         p, eye, u_time);
        fragColor = float4(color, 1.0);
    }

    output.write(fragColor, gid);
}
