// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TeoCheckCollisionsLibrary.hpp"

constexpr auto DEFAULT_NUM_COLLISION_OBJETCS = 1;
constexpr auto DEFAULT_NUM_LINKS = 1;

namespace roboticslab
{
    ShapeCollisionObject TeoCheckCollisionsLibrary::setObjectShape(const fcl::CollisionObjectf object,const SHAPE_TYPE shape_type,const int &label_idx, const std::vector<float> &length){
        ShapeCollisionObject aux;
        aux.label_idx = label_idx;
        aux.shape=shape_type;
        fcl::Quaternionf quat = object.getQuatRotation();
        fcl::Vector3f translation = object.getTranslation();
        aux.transform = {quat.x(), quat.y(), quat.z(), quat.w(), translation[0], translation[1], translation[2]};
        aux.size = length;

        return aux;
    }

    void TeoCheckCollisionsLibrary::getFixedObjectTransformations(std::vector<std::array<double, 7>> &transformations)
    {
        for (int i = 0; i < m_fixedCollisionObjects.size(); i++)
        {
            fcl::Quaternionf quat = m_fixedCollisionObjects[i].getQuatRotation();
            fcl::Vector3f translation = m_fixedCollisionObjects[i].getTranslation();
            std::array<double, 7> transformation = {quat.x(), quat.y(), quat.z(), quat.w(), translation[0], translation[1], translation[2]};
            transformations.push_back(transformation);
        }
    }

    void TeoCheckCollisionsLibrary::getObjectsShape(std::vector<ShapeCollisionObject> & shapesCollisionObjects){
        printf("getObjectsShape\n");
        for(int i=0; i<m_shapesCollisionObjects.size();i++)
            shapesCollisionObjects.push_back(m_shapesCollisionObjects[i]);
    }

    void TeoCheckCollisionsLibrary::configureEnvironmentFixedObjects()
    {
        yarp::os::Property fullConfig;
        fullConfig.fromConfigFile(m_fixedObjectsFileFullPath.c_str());
        m_numFixedObjects = fullConfig.check("numFixedObjects", yarp::os::Value(DEFAULT_NUM_COLLISION_OBJETCS)).asInt32();
        printf("numFixedObjects: %d\n", m_numFixedObjects);

        for (int fixedObjectIndex = 0; fixedObjectIndex < m_numFixedObjects; fixedObjectIndex++)
        {
            std::string fixedObjectStr = "fixedObject_" + std::to_string(fixedObjectIndex);
            yarp::os::Bottle &bFixedObject = fullConfig.findGroup(fixedObjectStr);
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

                if (!bCollisionObjectBoxShape.isNull())
                {
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
                else
                {
                    printf("Error: The shape of fixedObject %d is defined wrong in file %s\n", fixedObjectIndex, m_fixedObjectsFileFullPath.c_str());
                }
            }
            else
            {
                printf("Error: fixedObject %d not found in file %s\n", fixedObjectIndex, m_fixedObjectsFileFullPath.c_str());
            }
        }
    }

    bool TeoCheckCollisionsLibrary::collision()
    {
        if (!selfCollision())
        {
            fcl::CollisionRequestf requestType;
            fcl::CollisionResultf collisionResult;
            for (int link1 = 0; link1 < m_collisionObjects.size(); link1++)
            {
                for (int fixedObject = 0; fixedObject < m_fixedCollisionObjects.size(); fixedObject++)
                {
                    fcl::collide(&m_collisionObjects[link1], &m_fixedCollisionObjects[fixedObject], requestType, collisionResult);
                    if (collisionResult.isCollision())
                    {
                        printf("collision betwenn robot segment %d and fixed object %d\n", link1, fixedObject);
                        return true;
                    }
                }
                for (int movingObject=0; movingObject<m_environmentCollisionObjects.size(); movingObject++)
                {
                    fcl::collide(&m_collisionObjects[link1], &m_environmentCollisionObjects[movingObject], requestType, collisionResult);
                    if (collisionResult.isCollision())
                    {
                        printf("collision between robot segment %d and environment object %d\n", link1, movingObject);
                        return true;
                    }

                }
            }
            printf("collision() not collide\n");
            return false;
        }
        else
        {
            printf("selfCollision() collide\n");
            return true;
        }
    }
    void TeoCheckCollisionsLibrary::setSuperquadrics(const std::vector<int> label_idx, const std::vector<std::array<float, 11>> params)
    {
        m_superquadrics.clear();
        for (int i = 0; i < label_idx.size(); i++)
        {
            SuperQuadric s;
            s.label_idx = label_idx[i];
            s.params = params[i];
            m_superquadrics.push_back(s);
        }
        printf("Superquadrics set!\n");
    }

    void TeoCheckCollisionsLibrary::getSuperquadrics(std::vector<int> &label_idx, std::vector<std::array<float, 11>> &params)
    {
        label_idx.clear();
        params.clear();
        for (int i = 0; i < m_superquadrics.size(); i++)
        {
            label_idx.push_back(m_superquadrics[i].label_idx);
            params.push_back(m_superquadrics[i].params);
        }
    }

    void TeoCheckCollisionsLibrary::updateEnvironmentCollisionObjects()
    {
        m_environmentCollisionObjects.clear();
        m_shapesCollisionObjects.clear();
        for (int i = 0; i < m_superquadrics.size(); i++)
        {
            if (m_superquadrics[i].params[3] < DEFAULT_E_LIMIT1)
            {
                if (m_superquadrics[i].params[4] < DEFAULT_E_LIMIT1)
                { // Box
                    printf("Add box\n");
                    fcl::Transform3f tfTest;
                    // TODO: MODIFY POSITION AND ROTATION
                    CollisionGeometryPtr_t collisionGeometryAux{new fcl::Boxf{2*m_superquadrics[i].params[0]+DEFAULT_INCREASE_OBJECTS_SIZE, 2*m_superquadrics[i].params[1]+DEFAULT_INCREASE_OBJECTS_SIZE, 2*m_superquadrics[i].params[2]+DEFAULT_INCREASE_OBJECTS_SIZE}};
                    m_environmentCollisionObjects.push_back(fcl::CollisionObjectf{collisionGeometryAux, tfTest});
                    modifyTransformation(i, m_superquadrics[i].params);
                    std::vector<float> length = {m_superquadrics[i].params[0]+DEFAULT_INCREASE_OBJECTS_SIZE,m_superquadrics[i].params[1]+DEFAULT_INCREASE_OBJECTS_SIZE, m_superquadrics[i].params[2]+DEFAULT_INCREASE_OBJECTS_SIZE};
                    ShapeCollisionObject aux = setObjectShape(m_environmentCollisionObjects[i], SHAPE_TYPE::BOX, m_superquadrics[i].label_idx, length);
                    m_shapesCollisionObjects.push_back(aux);    
                }
                if ((m_superquadrics[i].params[4] >= DEFAULT_E_LIMIT1) && (m_superquadrics[i].params[4] < DEFAULT_E_LIMIT2))
                { // Cylinder
                    printf("Add cylinder\n");
                    printf("e1: %f e2: %f\n",m_superquadrics[i].params[3],m_superquadrics[i].params[4]);
                    fcl::Transform3f tfTest;
                    // TODO: MODIFY POSITION AND ROTATION

                    int index_height = 0;
                    float height = m_superquadrics[i].params[0];
                    if(m_superquadrics[i].params[1]>height){
                        height = m_superquadrics[i].params[1];
                    }                    
                    if(m_superquadrics[i].params[2]>height){
                        height = m_superquadrics[i].params[2];
                    }

                    float radius = 0;
                    for(int j=0; j<3; j++){
                        if(m_superquadrics[i].params[j]>radius && m_superquadrics[i].params[j]<height){
                            radius = m_superquadrics[i].params[j];
                        }
                    }
                    radius+=DEFAULT_INCREASE_OBJECTS_SIZE;
                    height+=DEFAULT_INCREASE_OBJECTS_SIZE;

                    CollisionGeometryPtr_t collisionGeometryAux{new fcl::Cylinderf{radius, 2*height}};
                    m_environmentCollisionObjects.push_back(fcl::CollisionObjectf{collisionGeometryAux, tfTest});
                    modifyTransformation(i, m_superquadrics[i].params);

                    std::vector<float> length = {radius, height};


                    ShapeCollisionObject aux = setObjectShape(m_environmentCollisionObjects[i], SHAPE_TYPE::CYLINDER, m_superquadrics[i].label_idx, length);
                    m_shapesCollisionObjects.push_back(aux);   
  
                }
                if ((m_superquadrics[i].params[4] >= DEFAULT_E_LIMIT2))
                { // Box
                    printf("Add box\n");
                    fcl::Transform3f tfTest;
                    // TODO: MODIFY POSITION AND ROTATION
                    CollisionGeometryPtr_t collisionGeometryAux{new fcl::Boxf{2*m_superquadrics[i].params[0]+DEFAULT_INCREASE_OBJECTS_SIZE, 2*m_superquadrics[i].params[1]+DEFAULT_INCREASE_OBJECTS_SIZE, 2*m_superquadrics[i].params[2]+DEFAULT_INCREASE_OBJECTS_SIZE}};
                    m_environmentCollisionObjects.push_back(fcl::CollisionObjectf{collisionGeometryAux, tfTest});
                    modifyTransformation(i, m_superquadrics[i].params);
                    std::vector<float> length = {m_superquadrics[i].params[0]+DEFAULT_INCREASE_OBJECTS_SIZE,m_superquadrics[i].params[1]+DEFAULT_INCREASE_OBJECTS_SIZE, m_superquadrics[i].params[2]+DEFAULT_INCREASE_OBJECTS_SIZE};
                    ShapeCollisionObject aux = setObjectShape(m_environmentCollisionObjects[i], SHAPE_TYPE::BOX, m_superquadrics[i].label_idx, length);
                    m_shapesCollisionObjects.push_back(aux);   
                }
            }
            if (m_superquadrics[i].params[3] >= DEFAULT_E_LIMIT1)
            {
                printf("Add ellipsoid\n");
                fcl::Transform3f tfTest;
                // TODO: MODIFY POSITION AND ROTATION
                CollisionGeometryPtr_t collisionGeometryAux{new fcl::Ellipsoidf{m_superquadrics[i].params[0]+DEFAULT_INCREASE_OBJECTS_SIZE, m_superquadrics[i].params[1]+DEFAULT_INCREASE_OBJECTS_SIZE, m_superquadrics[i].params[2]+DEFAULT_INCREASE_OBJECTS_SIZE}};
                m_environmentCollisionObjects.push_back(fcl::CollisionObjectf{collisionGeometryAux, tfTest});
                modifyTransformation(i, m_superquadrics[i].params);
                std::vector<float> length = {m_superquadrics[i].params[0]+DEFAULT_INCREASE_OBJECTS_SIZE,m_superquadrics[i].params[1]+DEFAULT_INCREASE_OBJECTS_SIZE, m_superquadrics[i].params[2]+DEFAULT_INCREASE_OBJECTS_SIZE};
                ShapeCollisionObject aux = setObjectShape(m_environmentCollisionObjects[i], SHAPE_TYPE::ELLIPSOID, m_superquadrics[i].label_idx, length);
                m_shapesCollisionObjects.push_back(aux); 
            }
        }
    }

    void TeoCheckCollisionsLibrary::modifyTransformation(int index, const std::array<float, 11> params)
    {

        Eigen::AngleAxisf rollAngle(params[8], Eigen::Vector3f::UnitZ());
        Eigen::AngleAxisf yawAngle(params[9], Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf pitchAngle(params[10], Eigen::Vector3f::UnitZ());

        Eigen::Quaternionf q = rollAngle * yawAngle * pitchAngle;

        fcl::Quaternionf rotation(q.w(), q.x(), q.y(), q.z());
        fcl::Vector3f translation(params[5], params[6], params[7]);

        m_environmentCollisionObjects[index].setTransform(rotation, translation);
        // transform.setTransform(rotation, translation);
    }

    void TeoCheckCollisionsLibrary::checkNodeTypes(){
        for(int index = 0; index<m_environmentCollisionObjects.size(); index++){
            int type =m_environmentCollisionObjects[index].getCollisionGeometry()->getNodeType();
            switch(type){
                case fcl::NODE_TYPE::GEOM_BOX:
                {
                    printf("shape defined as a box\n");
                    break;
                }
                case fcl::NODE_TYPE::GEOM_CYLINDER:
                {
                    printf("shape defined as a cylinder\n");
                    break;
                }
                case fcl::NODE_TYPE::GEOM_ELLIPSOID:
                {
                    printf("shape defined as a ELLIPSOID\n");
                    break;
                }

                default:
                {
                    printf("what I have done? I haven't defined this shape\n");
                }

            }
        }

    }

    /************************************************************************/

    /************************************************************************/

    /************************************************************************/

} // namespace roboticslab
