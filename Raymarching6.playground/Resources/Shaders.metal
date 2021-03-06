// https://www.shadertoy.com/view/4tcGDr

#include <metal_stdlib>

using namespace metal;

/**
 * Ray Marching Part 6
 * Challenges:
 * - Make a scene of your own! Try to use the rotation transforms, the CSG primitives,
 *   and the geometric primitives. Remember you can use vector subtraction for translation,
 *   and component-wise vector multiplication for scaling.
 */

constant const int MAX_MARCHING_STEPS = 255;
constant const float MIN_DIST = 0.0;
constant const float MAX_DIST = 100.0;
constant const float EPSILON = 0.0001;

/**
 * Rotation matrix around the X axis.
 */
float3x3 rotateX(float theta) {
    float c = cos(theta);
    float s = sin(theta);
    return float3x3(
                float3(1, 0,  0),
                float3(0, c, -s),
                float3(0, s,  c)
                );
}

/**
 * Rotation matrix around the Y axis.
 */
float3x3 rotateY(float theta) {
    float c = cos(theta);
    float s = sin(theta);
    return float3x3(
                float3( c, 0, s),
                float3( 0, 1, 0),
                float3(-s, 0, c)
                );
}

/**
 * Rotation matrix around the Z axis.
 */
float3x3 rotateZ(float theta) {
    float c = cos(theta);
    float s = sin(theta);
    return float3x3(
                float3(c, -s, 0),
                float3(s,  c, 0),
                float3(0,  0, 1)
                );
}

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
 * with dimensions specified by size.
 */
float boxSDF(float3 p, float3 size) {
    float3 d = abs(p) - (size / 2.0);

    // Assuming p is inside the cube, how far is it from the surface?
    // Result will be negative or zero.
    float insideDistance = min(max(d.x, max(d.y, d.z)), 0.0);

    // Assuming p is outside the cube, how far is it from the surface?
    // Result will be positive or zero.
    float outsideDistance = length(max(d, 0.0));

    return insideDistance + outsideDistance;
}

/**
 * Signed distance function for a sphere centered at the origin with radius r.
 */
float sphereSDF(float3 p, float r) {
    return length(p) - r;
}

/**
 * Signed distance function for an XY-aligned cylinder centered
 * at the origin with height h and radius r.
 */
float cylinderSDF(float3 p, float h, float r) {
    // How far inside or outside the cylinder the point is, radially?
    float inOutRadius = length(p.xy) - r;

    // How far inside or outside the cylinder is, axially aligned with the cylinder?
    float inOutHeight = abs(p.z) - h/2.0;

    // Assuming p is inside the cylinder, how far is it from the surface?
    // Result will be negative or zero.
    float insideDistance = min(max(inOutRadius, inOutHeight), 0.0);

    // Assuming p is outside the cylinder, how far is it from the surface?
    // Result will be positive or zero.
    float outsideDistance = length(max(float2(inOutRadius, inOutHeight), 0.0));

    return insideDistance + outsideDistance;
}


float radians(float angle) {
    float A = angle/180 * M_PI_F;
    return A;
}


/**
 * Signed distance function describing the scene.
 * 
 * Absolute value of the return value indicates the distance to the surface.
 * Sign indicates whether the point is inside or outside the surface,
 * negative indicating inside.
 */
float sceneSDF(float3 samplePoint, float iTime) {
    // Slowly spin the whole scene
    samplePoint = rotateY(iTime / 2.0) * samplePoint;

    float cylinderRadius = 0.4 + (1.0 - 0.4) * (1.0 + sin(1.7 * iTime)) / 2.0;
    float cylinder1 = cylinderSDF(samplePoint, 2.0, cylinderRadius);
    float cylinder2 = cylinderSDF(rotateX(radians(90.0)) * samplePoint, 2.0, cylinderRadius);
    float cylinder3 = cylinderSDF(rotateY(radians(90.0)) * samplePoint, 2.0, cylinderRadius);

    float cube = boxSDF(samplePoint, float3(1.8, 1.8, 1.8));

    float sphere = sphereSDF(samplePoint, 1.2);

    // 6 balls (small spheres)
    float ballOffset = 0.4 + 1.0 + sin(1.7 * iTime);
    float ballRadius = 0.3;
    float balls = sphereSDF(samplePoint - float3(ballOffset, 0.0, 0.0), ballRadius);
    balls = unionSDF(balls, sphereSDF(samplePoint + float3(ballOffset, 0.0, 0.0), ballRadius));
    balls = unionSDF(balls, sphereSDF(samplePoint - float3(0.0, ballOffset, 0.0), ballRadius));
    balls = unionSDF(balls, sphereSDF(samplePoint + float3(0.0, ballOffset, 0.0), ballRadius));
    balls = unionSDF(balls, sphereSDF(samplePoint - float3(0.0, 0.0, ballOffset), ballRadius));
    balls = unionSDF(balls, sphereSDF(samplePoint + float3(0.0, 0.0, ballOffset), ballRadius));

    float csgNut = differenceSDF(intersectSDF(cube, sphere),
                                 unionSDF(cylinder1, unionSDF(cylinder2, cylinder3)));

    return unionSDF(balls, csgNut);
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
float shortestDistanceToSurface(float3 eye, float3 marchingDirection, float start, float end, float iTime) {
    float depth = start;
    for (int i = 0; i < MAX_MARCHING_STEPS; i++) {
        float dist = sceneSDF(eye + depth * marchingDirection, iTime);
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
float3 estimateNormal(float3 p, float iTime) {
    return normalize(float3(
        sceneSDF(float3(p.x + EPSILON, p.y, p.z), iTime) - sceneSDF(float3(p.x - EPSILON, p.y, p.z), iTime),
        sceneSDF(float3(p.x, p.y + EPSILON, p.z), iTime) - sceneSDF(float3(p.x, p.y - EPSILON, p.z), iTime),
        sceneSDF(float3(p.x, p.y, p.z  + EPSILON), iTime) - sceneSDF(float3(p.x, p.y, p.z - EPSILON), iTime)
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
float3 phongContribForLight(float3 k_d, float3 k_s, float alpha, float3 p, float3 eye,
                            float3 lightPos, float3 lightIntensity, float iTime) {
    float3 N = estimateNormal(p, iTime);
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
                                  light1Intensity,
                                  iTime);

    float3 light2Pos = float3(2.0 * sin(0.37 * iTime),
                              2.0 * cos(0.37 * iTime),
                              2.0);
    float3 light2Intensity = float3(0.4, 0.4, 0.4);
    
    color += phongContribForLight(k_d, k_s, alpha, p, eye,
                                  light2Pos,
                                  light2Intensity,
                                  iTime);    
    return color;
}

/**
 * Return a transformation matrix that will transform a ray from view space
 * to world coordinates, given the eye point, the camera target, and an up vector.
 *
 * This assumes that the center of the camera is aligned with the negative z axis in
 * view space when calculating the ray marching direction. See rayDirection.
 */
float3x3 viewMatrix(float3 eye, float3 center, float3 up) {
    // Based on gluLookAt man page
    float3 f = normalize(center - eye);
    float3 s = normalize(cross(f, up));
    float3 u = cross(s, f);
    return float3x3(s, u, -f);
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
    float2 fragCoord = float2(gid.x, height-gid.y);

    float3 viewDir = rayDirection(45.0, u_resolution.xy, fragCoord);
    // Moving camera rotating about y-axis.
    float3 eye = float3(8.0, 5.0 * sin(0.2 * u_time), 7.0);
    float3x3 viewToWorld = viewMatrix(eye,
                                      float3(0.0, 0.0, 0.0),
                                      float3(0.0, 1.0, 0.0));

    float3 worldDir = viewToWorld * viewDir;
    float dist = shortestDistanceToSurface(eye, worldDir, MIN_DIST, MAX_DIST, u_time);

    float4 fragColor;
    if (dist > MAX_DIST - EPSILON) {
        // Didn't hit anything
        fragColor = float4(0.0);
    }
    else {
        // The closest point on the surface to the eyepoint along the view ray
        float3 p = eye + dist * worldDir;

        float3 K_a = (estimateNormal(p, u_time) + float3(1.0)) / 2.0;
        float3 K_d = K_a;
        float3 K_s = float3(1.0, 1.0, 1.0);
        float shininess = 10.0;

        float3 color = phongIllumination(K_a, K_d, K_s, shininess,
                                         p, eye,
                                         u_time);
        fragColor = float4(color, 1.0);
    }
    output.write(fragColor, gid);
}
