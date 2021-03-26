// https://www.shadertoy.com/view/MttGz7

#include <metal_stdlib>

using namespace metal;

/**
 * Ray Marching: Part 4
 * Challenges:
 * - Show the union instead of the intersection
 * - Show cube - sphere
 * - Show sphere - cube
 * - Subtract a new sphere from the cube/sphere intersection to make the top face into a "bowl"
 */

constant const int MAX_MARCHING_STEPS = 255;
constant const float MIN_DIST = 0.0;
constant const float MAX_DIST = 100.0;
constant const float EPSILON = 0.0001;

/**
 * Constructive solid geometry intersection operation on SDF-calculated distances.
 */
float intersectSDF(float distA, float distB) {
    return max(distA, distB);
}

/**
* Constructive solid geometry union operation on SDF-calculated distances.
*/
float unionSDF(float distA, float distB) {
    return min(distA, distB);
}

/**
 * Constructive solid geometry difference operation on SDF-calculated distances.
 */
float differenceSDF(float distA, float distB) {
    return max(distA, -distB);
}

/**
 * Signed distance function for a cube centered at the origin
 * with width = height = length = 2.0
 */
float cubeSDF(float3 p) {
    // If d.x < 0, then -1 < p.x < 1, and same logic applies to p.y, p.z.
    // So if all components of d are negative, then p is inside the 2x2x2 cube
    float3 d = abs(p) - float3(1.0, 1.0, 1.0);

    // Assuming p is inside the cube, how far is it from the surface?
    // Result will be negative or zero.
    float insideDistance = min(max(d.x, max(d.y, d.z)), 0.0);

    // Assuming p is outside the cube, how far is it from the surface?
    // Result will be positive or zero.
    float outsideDistance = length(max(d, 0.0));

    return insideDistance + outsideDistance;
}

/**
 * Signed distance function for a sphere centered at the origin with radius 1.0.
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
    float sphereDist = sphereSDF(samplePoint / 1.2) * 1.2;
    float cubeDist = cubeSDF(samplePoint);
    return intersectSDF(cubeDist, sphereDist);
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
			return depth;
        }
        depth += dist;
        if (depth >= end) {
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
    // Direction of z is into the screen.
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
    float3 N = estimateNormal(p);
    float3 L = normalize(lightPos - p);
    float3 V = normalize(eye - p);
    float3 R = normalize(reflect(-L, N));

    float dotLN = dot(L, N);
    float dotRV = dot(R, V);

    if (dotLN < 0.0) {
        // Light not visible from this point on the surface
        return float3(0.0, 0.0, 0.0);
    } 

    if (dotRV < 0.0) {
        // Light reflection in opposite direction as viewer, apply only diffuse
        // component
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
float3 phongIllumination(float3 k_a, float3 k_d, float3 k_s,
                         float alpha, float3 p, float3 eye,
                         float iTime) {
    const float3 ambientLight = 0.5 * float3(1.0, 1.0, 1.0);
    float3 color = ambientLight * k_a;

    float3 light1Pos = float3(4.0 * sin(iTime),
                              2.0,
                              4.0 * cos(iTime));
    float3 light1Intensity = float3(0.4, 0.4, 0.4);

    color += phongContribForLight(k_d, k_s, alpha, p, eye,
                                  light1Pos,
                                  light1Intensity);

    float3 light2Pos = float3(2.0 * sin(0.37 * iTime),
                              2.0 * cos(0.37 * iTime),
                              2.0);
    float3 light2Intensity = float3(0.4, 0.4, 0.4);

    color += phongContribForLight(k_d, k_s, alpha,
                                  p, eye,
                                  light2Pos, light2Intensity);    
    return color;
}

/**
 * Return a transformation matrix that will transform a ray from view space
 * to world coordinates, given the eye point, the camera target, and an up vector.
 *
 * This assumes that the center of the camera is aligned with the negative z axis in
 * view space when calculating the ray marching direction. See rayDirection.
 */
float4x4 viewMatrix(float3 eye, float3 center, float3 up) {
    // Based on gluLookAt man page
    float3 f = normalize(center - eye);
    float3 s = normalize(cross(f, up));
    float3 u = cross(s, f);
    return float4x4(
                float4(s, 0.0),
                float4(u, 0.0),
                float4(-f, 0.0),
                float4(0.0, 0.0, 0.0, 1)
                );
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
    float2 fragCoord = float2(gid.x, height-gid.y);

    float3 viewDir = rayDirection(45.0, u_resolution.xy, fragCoord);
    float3 eye = float3(8.0, 5.0, 7.0);
    float4x4 viewToWorld = viewMatrix(eye,
                                      float3(0.0, 0.0, 0.0),
                                      float3(0.0, 1.0, 0.0));

    float3 worldDir = (viewToWorld * float4(viewDir, 0.0)).xyz;
    float dist = shortestDistanceToSurface(eye, worldDir, MIN_DIST, MAX_DIST);

    float4 fragColor = float4(0);
    if (dist > MAX_DIST - EPSILON) {
        // Didn't hit anything, show background color.
        fragColor = float4(0.0);
    }
    else {
        // The closest point on the surface to the eyepoint along the view ray
        float3 p = eye + dist * worldDir;
        
        float3 K_a = float3(0.2, 0.2, 0.2);
        float3 K_d = float3(0.7, 0.2, 0.2);
        float3 K_s = float3(1.0, 1.0, 1.0);
        float shininess = 10.0;
        
        float3 color = phongIllumination(K_a, K_d, K_s, shininess,
                                         p, eye, u_time);
        fragColor = float4(color, 1.0);
    }

    output.write(fragColor, gid);
}
