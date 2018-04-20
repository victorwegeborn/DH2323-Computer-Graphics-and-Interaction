//
//  Clipping.hpp
//  VFC
//
//  Created by Victor Wegeborn on 2017-05-17.
//  Copyright © 2017 Victor Wegeborn. All rights reserved.
//

#ifndef Clipping_hpp
#define Clipping_hpp

#include "glm/glm.hpp"
#include <vector>
#include "Vertex.h"

struct Vertex;

class Clipping {
    const int SCREEN_WIDTH;
    const int FOCAL_LENGTH;
    
    const float nearDistance = 3.f;
    const float farDistance = 6.f;
    float widthNear;
    
    struct Plane {
        glm::vec3 point;
        glm::vec3 normal;
        float d; // for plane equation ax+by+cz+d = 0
    };
    
    std::vector<Plane> planes{6};
    
    
public :
    Clipping(const int &SCREEN_WIDTH, const int FOCAL_LENGTH);
    void generateFrustum(glm::vec3 &cameraPosition, glm::vec3 &forward, glm::vec3 &right,glm::vec3 &up);
    void clipToFrustum(std::vector<Vertex> &vertices);
    
private:
    float signedDistanceFromPointToPlane(const glm::vec3 &point, const Clipping::Plane &plane);
    glm::vec3 computeIntersection(const glm::vec3 &start, const glm::vec3 &end, const Clipping::Plane &plane);
};

#endif /* Clipping_hpp */

