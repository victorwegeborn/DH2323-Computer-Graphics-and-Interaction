//
//  Clipping.cpp
//  VFC
//
//  Created by Victor Wegeborn on 2017-05-17.
//  Copyright © 2017 Victor Wegeborn. All rights reserved.
//


// TODO FELSÖK VAR FAN PROBLEMET ÄR. FELSÖK GENERATE frustum.

#include "Clipping.hpp"
#include <cmath>
#include <iostream>


/*  Clipping(const int &, const int &, const int &)
 *
 *  Constructor for clipping class. Calculates FOV and aspectRatio as well as
 *  width/heigth for the near/far clipplane.
 *
 *  param: screen height, screen width, focal length.
 *
 */
Clipping::Clipping(const int &SCREEN_WIDTH, const int FOCAL_LENGTH)
: SCREEN_WIDTH(SCREEN_WIDTH), FOCAL_LENGTH(FOCAL_LENGTH) {
    
    /* The geometrical relationship simplified.
     
    verticalFOV = 2.f*atanf((float)SCREEN_HEIGHT/(2.f*(float)FOCAL_LENGTH));
    widthNear = 2.f*tanf(0.5f*verticalFOV)*nearDistance;
    */
    
    widthNear = (SCREEN_WIDTH/FOCAL_LENGTH)*nearDistance;
};




/*  generatefrustum(glm::vec3 &, glm::vec3 &, glm::vec3 &, glm::vec3 &)
 *
 *  Calculates the frustum.
 *
 *  Optimization list: A new frustum is only needed when the camera has moved.
 *                     Since a plane can be calculated from tree points, there is
 *                     no need to calculate all 8 points that make up the frustum.
 *                     This is NOT implementet.
 *
 *  param:  cameraPosition, forward, right, up. (FORWARD AND UP MUST BE NORMALIZED!)
 */
void Clipping::generateFrustum(glm::vec3 &cameraPosition, glm::vec3 &forward, glm::vec3 &right,glm::vec3 &up) {
    
    glm::vec3 farPlaneCenter = cameraPosition + forward * farDistance;
    glm::vec3 nearPlaneCenter = cameraPosition + forward * nearDistance;
    
    // calculate planes for the frustum:
    // Near plane
    planes[0].normal = forward;
    planes[0].point = nearPlaneCenter;
    
    // far plane
    planes[1].normal = -forward;
    planes[1].point = farPlaneCenter;
    
    // right plane
    planes[2].normal = glm::cross(up, glm::normalize(nearPlaneCenter + right * widthNear *0.5f - cameraPosition));
    planes[2].point = cameraPosition;
    
    // left plane
    planes[3].normal = glm::cross(-up, glm::normalize(nearPlaneCenter - right * widthNear *0.5f - cameraPosition));
    planes[3].point = cameraPosition;
    
    // top plane
    planes[4].normal = glm::cross(glm::normalize(nearPlaneCenter + up * widthNear *0.5f - cameraPosition), right);
    planes[4].point = cameraPosition;
    
    // bottom plane
    planes[5].normal = glm::cross(glm::normalize(nearPlaneCenter - up * widthNear *0.5f - cameraPosition), -right);
    planes[5].point = cameraPosition;

     
    // Compute d for all planes in equation ax+by+cz+d = 0
    for(int i = 0; i < planes.size(); ++i)
        planes[i].d = (-glm::dot(planes[i].normal,planes[i].point));
}




/*  clipTofrustum(std::vector<Vertex> &)
 *
 *  The extended sutherland-hodgeman algorithm for tree diemnsions
 *
 *  params: vertecies of each triangle
 */
void Clipping::clipToFrustum(std::vector<Vertex> &vertices) {
    
    // keep a unmodified copy of the vertecies in the outputlist
    std::vector<Vertex> output = vertices;
    
    for(Plane P : planes) {
        std::vector<Vertex> input = output;
        output.clear();
        Vertex S = input.back();
        
        for(Vertex E : input) {
            float S2PDistance = signedDistanceFromPointToPlane(S.position, P);
            float E2PDistance = signedDistanceFromPointToPlane(E.position, P);
            if( E2PDistance >= 0.f) {
                if(S2PDistance < 0.f) {
                    Vertex v{computeIntersection(E.position,S.position,P)};
                    output.push_back(v);
                }
                output.push_back(E);
            }
            else if(S2PDistance >= 0.f) {
                Vertex v{computeIntersection(S.position,E.position,P)};
                output.push_back(v);
            }
            S = E;
        }
        if(output.size() == 0)
            break;
    }
    vertices = output;
}




/*  signedDistanceFromPointToPlane(const clm::vec3 &, const Clipping::Plane &)
 *
 *  Returnes the signed distance between the plane and a point.
 *  If the distance i < 0, then the point is on the opposite side of the
 *  surface normal. 
 *
 *  params: point, plane
 *
 *  return: sigend distance
 */
float Clipping::signedDistanceFromPointToPlane(const glm::vec3 &point, const Clipping::Plane &plane) {
    return glm::dot(plane.normal, point) + plane.d;
}




/*  computeIntersection(const Vertex &, const Vertex &, const Clipping::Plane &)
 *
 *  Algebraic approach to find intersections between a line (with points start and end)
 *  and a plane.
 *
 *  params: starting point, ending point, frustum plane.
 *
 *  return: the point in the plane where the intersection occurred.
 */
glm::vec3 Clipping::computeIntersection(const glm::vec3 &start, const glm::vec3 &end, const Clipping::Plane &plane) {
    glm::vec3 lineDirection = glm::normalize(end-start);
    float denominator = glm::dot(lineDirection, plane.normal);
    
    if(denominator == 0) // parallell line. return end vector.
        return end;
    
    float scalar = (float)glm::dot(plane.point - start, plane.normal) / denominator;
    return (scalar*lineDirection)+start;
}


