// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TEO_CHECK_COLLISIONS_LIBRARY_HPP__
#define __TEO_CHECK_COLLISIONS_LIBRARY_HPP__

#include "TeoCheckSelfCollisionsLibrary.hpp"

using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometryf>;

namespace roboticslab
{

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


protected:
    std::vector<fcl::CollisionObjectf> m_environmentCollisionObjects;
    std::vector<fcl::CollisionObjectf> m_fixedCollisionObjects;

    std::string m_fixedObjectsFileFullPath;
    unsigned int m_numFixedObjects;
    std::vector<std::array<float,3>> m_positionFixedObjects;
    std::vector<std::array<float,4>> m_orientationFixedObjects;
    std::vector<std::array<float,3>> m_boxShapesFixedObjects;


};

} // namespace roboticslab

#endif // __TEO_CHECK_COLLISIONS_LIBRARY_HPP__
