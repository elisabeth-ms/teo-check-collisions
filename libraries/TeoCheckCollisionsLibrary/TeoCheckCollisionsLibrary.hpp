// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TEO_CHECK_COLLISIONS_LIBRARY_HPP__
#define __TEO_CHECK_COLLISIONS_LIBRARY_HPP__

#include "TeoCheckSelfCollisionsLibrary.hpp"
#define DEFAULT_E_LIMIT1 0.5
#define DEFAULT_E_LIMIT2 1.2
#define DEFAULT_E_LIMIT3 2.0

using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometryf>;

namespace roboticslab
{

struct SuperQuadric{
    int label_idx;
    std::array<float,11> params;
};

/**
 * @ingroup teoCheckCollisionsLibrary
 *
 * @brief teoCheckCollisionsLibrary
 */
class TeoCheckCollisionsLibrary: public TeoCheckSelfCollisionsLibrary
{
public:
    TeoCheckCollisionsLibrary(const std::string & t_fixedObjectsFileFullPath):
                            m_fixedObjectsFileFullPath(t_fixedObjectsFileFullPath){}
    void configureEnvironmentFixedObjects();

    void getBoxShapesFixedObjects(std::vector<std::array<float,3>> & boxShapesFixedObjects){
       boxShapesFixedObjects = m_boxShapesFixedObjects;
    }
    void getFixedObjectTransformations(std::vector<std::array<double,7>> & transformations);

    bool collision();

    void setSuperquadrics(const std::vector<int> label_idx, const std::vector<std::array<float,11>> params);
    void getSuperquadrics(std::vector<int>& label_idx, std::vector<std::array<float,11>> &params);

    void updateEnvironmentCollisionObjects();

protected:
    std::vector<fcl::CollisionObjectf> m_environmentCollisionObjects;
    std::vector<fcl::CollisionObjectf> m_fixedCollisionObjects;

    std::string m_fixedObjectsFileFullPath;
    unsigned int m_numFixedObjects;
    std::vector<std::array<float,3>> m_positionFixedObjects;
    std::vector<std::array<float,4>> m_orientationFixedObjects;
    std::vector<std::array<float,3>> m_boxShapesFixedObjects;
    std::vector<SuperQuadric> m_superquadrics;

};

} // namespace roboticslab

#endif // __TEO_CHECK_COLLISIONS_LIBRARY_HPP__
