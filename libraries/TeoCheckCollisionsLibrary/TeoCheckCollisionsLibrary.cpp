// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TeoCheckCollisionsLibrary.hpp"

constexpr auto DEFAULT_NUM_COLLISION_OBJETCS = 1;
constexpr auto DEFAULT_NUM_LINKS = 1;

namespace roboticslab
{
    void TeoCheckCollisionsLibrary::getFixedObjectTransformations(std::vector<std::array<double,7>> & transformations){
        for(int i=0; i<m_fixedCollisionObjects.size(); i++){
            fcl::Quaternionf quat = m_fixedCollisionObjects[i].getQuatRotation();
            fcl::Vector3f translation = m_fixedCollisionObjects[i].getTranslation();
            std::array<double,7> transformation = {quat.x(), quat.y(), quat.z(), quat.w(), translation[0], translation[1], translation[2]};
            transformations.push_back(transformation);
        }
    }

    void TeoCheckCollisionsLibrary::configureEnvironmentFixedObjects(){
        yarp::os::Property fullConfig;
        fullConfig.fromConfigFile(m_fixedObjectsFileFullPath.c_str());
        m_numFixedObjects = fullConfig.check("numFixedObjects", yarp::os::Value(DEFAULT_NUM_COLLISION_OBJETCS)).asInt32();
        printf("numFixedObjects: %d\n", m_numFixedObjects);

        for (int fixedObjectIndex = 0; fixedObjectIndex < m_numFixedObjects; fixedObjectIndex++)
        {
            std::string fixedObjectStr = "fixedObject_" + std::to_string(fixedObjectIndex);
            yarp::os::Bottle & bFixedObject = fullConfig.findGroup(fixedObjectStr);
            if (!bFixedObject.isNull())
            {
                printf("We have the fixedObject: %d\n", fixedObjectIndex);
                yarp::os::Bottle fixedObjectPosition = bFixedObject.findGroup("pos", "position (SI units)").tail();
                m_positionFixedObjects.push_back(std::array<float, 3>{fixedObjectPosition.get(0).asFloat32(),
                                                                        fixedObjectPosition.get(1).asFloat32(),
                                                                        fixedObjectPosition.get(2).asFloat32()});
                yarp::os::Bottle fixedObjectOrientation = bFixedObject.findGroup("rot", "orientaion (quaternion qx qy qz qw)").tail();
                m_orientationFixedObjects.push_back(std::array<float, 4>{fixedObjectOrientation.get(0).asFloat32(),
                                                                        fixedObjectOrientation.get(1).asFloat32(),
                                                                        fixedObjectOrientation.get(2).asFloat32(),
                                                                        fixedObjectOrientation.get(3).asFloat32()});
                
                printf("position: %f %f %f\n", m_positionFixedObjects.back()[0], m_positionFixedObjects.back()[1], m_positionFixedObjects.back()[2]);
                printf("orientation: %f %f %f %f\n", m_orientationFixedObjects.back()[0], m_orientationFixedObjects.back()[1], m_orientationFixedObjects.back()[2], m_orientationFixedObjects.back()[3]);

                yarp::os::Bottle bCollisionObjectBoxShape = bFixedObject.findGroup("boxShape", "collision box shape (SI units)").tail();                                                        

                if(!bCollisionObjectBoxShape.isNull()){
                    printf("Fixed object is defined as a box of size: %f %f %f", bCollisionObjectBoxShape.get(0).asFloat32(), bCollisionObjectBoxShape.get(1).asFloat32(), bCollisionObjectBoxShape.get(2).asFloat32());
                    fcl::Transform3f tfTest;
                    CollisionGeometryPtr_t collisionGeometryAux{new fcl::Boxf{bCollisionObjectBoxShape.get(0).asFloat32(), bCollisionObjectBoxShape.get(1).asFloat32(), bCollisionObjectBoxShape.get(2).asFloat32()}};
                    m_fixedCollisionObjects.push_back(fcl::CollisionObjectf{collisionGeometryAux, tfTest});
                    fcl::Quaternionf rotation(m_orientationFixedObjects.back()[3], m_orientationFixedObjects.back()[0], m_orientationFixedObjects.back()[1], m_orientationFixedObjects.back()[2]);
                    fcl::Vector3f translation(m_positionFixedObjects.back()[0], m_positionFixedObjects.back()[1], m_positionFixedObjects.back()[2]);
                    m_fixedCollisionObjects.back().setTransform(rotation, translation);
                    m_boxShapesFixedObjects.push_back(std::array<float, 3>{bCollisionObjectBoxShape.get(0).asFloat32(), 
                                                               bCollisionObjectBoxShape.get(1).asFloat32(), 
                                                               bCollisionObjectBoxShape.get(2).asFloat32()}); 
                }
                else{
                    printf("Error: The shape of fixedObject %d is defined wrong in file %s\n", fixedObjectIndex, m_fixedObjectsFileFullPath.c_str());
                }
            }
            else{
                printf("Error: fixedObject %d not found in file %s\n", fixedObjectIndex, m_fixedObjectsFileFullPath.c_str());
            }
        }
    }

    bool TeoCheckCollisionsLibrary::collision()
    {
        if (!selfCollision()){
            fcl::CollisionRequestf requestType;
            fcl::CollisionResultf collisionResult;
            for (int link1 = 0; link1<m_collisionObjects.size(); link1++)
            {
                for(int fixedObject = 0; fixedObject<m_fixedCollisionObjects.size(); fixedObject++){
                    fcl::collide(&m_collisionObjects[link1], &m_fixedCollisionObjects[fixedObject], requestType, collisionResult);
                    if (collisionResult.isCollision())
                    {
                        printf("collision betwenn robot segment %d and fixed object %d\n", link1, fixedObject);
                        return true;
                    }
                }
            }
            printf("collision() not collide\n");
            return false;
        }
        else{
            printf("collision() not collide\n");
            return false;
        }
    }

/************************************************************************/

/************************************************************************/

/************************************************************************/

} // namespace roboticslab
