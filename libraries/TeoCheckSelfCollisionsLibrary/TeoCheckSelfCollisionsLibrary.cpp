// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TeoCheckSelfCollisionsLibrary.hpp"

constexpr auto DEFAULT_NUM_COLLISION_OBJETCS = 1;
constexpr auto DEFAULT_NUM_LINKS = 1;

namespace roboticslab
{
    bool TeoCheckSelfCollisionsLibrary::getMatrixFromProperties(const yarp::os::Searchable & options, const std::string & tag, Eigen::MatrixXd & mat)
    {
        const auto * bH = options.find(tag).asList();

        if (!bH)
        {
            return false;
        }

        int i = 0;
        int j = 0;

        for (int cnt = 0; cnt < bH->size() && cnt < mat.rows() * mat.cols(); cnt++)
        {
            mat(i, j) = bH->get(cnt).asFloat64();

            if (++j >= mat.cols())
            {
                i++;
                j = 0;
            }
        }

        std::stringstream ss;
        ss << "Matrix " << tag << ":\n" << mat;
        printf("%s\n", ss.str().c_str());

        return true;
    }

    bool TeoCheckSelfCollisionsLibrary::getChainFromKinematicsFile(const yarp::os::Property &fullConfig){

    //*** Copy from https://github.com/roboticslab-uc3m/kinematics-dynamics/blob/master/libraries/YarpPlugins/KdlSolver/DeviceDriverImpl.cpp *** //

    int numLinks = fullConfig.check("numLinks", yarp::os::Value(DEFAULT_NUM_LINKS), "chain number of segments").asInt32();
    printf("numLinks: %d\n", numLinks);

    //-- gravity (default)
    yarp::os::Value defaultGravityValue;
    yarp::os::Bottle * defaultGravityBottle = defaultGravityValue.asList();
    defaultGravityBottle->addFloat64(0.0);
    defaultGravityBottle->addFloat64(0.0);
    defaultGravityBottle->addFloat64(-9.81);

    //-- gravity
    yarp::os::Value gravityValue = fullConfig.check("gravity", defaultGravityValue, "gravity vector (SI units)");
    yarp::os::Bottle * gravityBottle = gravityValue.asList();
    KDL::Vector gravity(gravityBottle->get(0).asFloat64(), gravityBottle->get(1).asFloat64(), gravityBottle->get(2).asFloat64());

    printf("gravity: %s\n", gravityBottle->toString().c_str());

    //-- H0
    Eigen::MatrixXd H0 = Eigen::MatrixXd::Identity(4, 4);

    if (!getMatrixFromProperties(fullConfig, "H0", H0))
    {
        printf("Warning: Failed to parse H0, using default identity matrix\n");
    }

    KDL::Vector kdlVec0(H0(0, 3), H0(1, 3), H0(2, 3));
    KDL::Rotation kdlRot0(H0(0, 0), H0(0, 1), H0(0, 2), H0(1, 0), H0(1, 1), H0(1, 2), H0(2, 0), H0(2, 1), H0(2, 2));
    m_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(kdlRot0, kdlVec0)));

    //-- links
    for (int linkIndex = 0; linkIndex < numLinks; linkIndex++)
    {
        std::string link = "link_" + std::to_string(linkIndex);
        yarp::os::Bottle & bLink = fullConfig.findGroup(link);

        if (!bLink.isNull())
        {
            //-- Kinematic
            double linkOffset = bLink.check("offset", yarp::os::Value(0.0), "DH joint angle (degrees)").asFloat64();
            double linkD = bLink.check("D", yarp::os::Value(0.0), "DH link offset (meters)").asFloat64();
            double linkA = bLink.check("A", yarp::os::Value(0.0), "DH link length (meters)").asFloat64();
            double linkAlpha = bLink.check("alpha", yarp::os::Value(0.0), "DH link twist (degrees)").asFloat64();

            KDL::Joint axis(KDL::Joint::RotZ);
            KDL::Frame H = KDL::Frame::DH(linkA, KinRepresentation::degToRad(linkAlpha), linkD, KinRepresentation::degToRad(linkOffset));

            //-- Dynamic
            if (bLink.check("mass") && bLink.check("cog") && bLink.check("inertia"))
            {
                double linkMass = bLink.check("mass", yarp::os::Value(0.0), "link mass (SI units)").asFloat64();
                yarp::os::Bottle linkCog = bLink.findGroup("cog", "vector of link's center of gravity (SI units)").tail();
                yarp::os::Bottle linkInertia = bLink.findGroup("inertia", "vector of link's inertia (SI units)").tail();

                KDL::Vector cog(linkCog.get(0).asFloat64(), linkCog.get(1).asFloat64(), linkCog.get(2).asFloat64());
                KDL::RotationalInertia inertia(linkInertia.get(0).asFloat64(), linkInertia.get(1).asFloat64(), linkInertia.get(2).asFloat64());

                m_chain.addSegment(KDL::Segment(axis, H, KDL::RigidBodyInertia(linkMass, cog, inertia)));

 
                printf("Added: %s (offset %f) (D %f) (A %f) (alpha %f) (mass %f) (cog %f %f %f) (inertia %f %f %f)\n",
                       link.c_str(), linkOffset, linkD, linkA, linkAlpha, linkMass,
                       linkCog.get(0).asFloat64(), linkCog.get(1).asFloat64(), linkCog.get(2).asFloat64(),
                       linkInertia.get(0).asFloat64(), linkInertia.get(1).asFloat64(), linkInertia.get(2).asFloat64());
            }
            else
            {
                m_chain.addSegment(KDL::Segment(axis, H));
                printf("Added: %s (offset %f) (D %f) (A %f) (alpha %f)\n", link.c_str(), linkOffset, linkD, linkA, linkAlpha);
            }
        }
        
    }

    //-- HN
    Eigen::MatrixXd HN = Eigen::MatrixXd::Identity(4, 4);

    if (!getMatrixFromProperties(fullConfig, "HN", HN))
    {
        printf("WARNING: Failed to parse HN, using default identity matrix\n");
    }

    KDL::Vector kdlVecN(HN(0, 3), HN(1, 3), HN(2, 3));
    KDL::Rotation kdlRotN(HN(0, 0), HN(0, 1), HN(0, 2), HN(1, 0), HN(1, 1), HN(1, 2), HN(2, 0), HN(2, 1), HN(2, 2));
    m_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(kdlRotN, kdlVecN)));

    printf("Chain number of segments: %d\n", m_chain.getNrOfSegments());
    printf("Chain number of joints: %d\n", m_chain.getNrOfJoints());
    return true;
    //*** End Copy from https://github.com/roboticslab-uc3m/kinematics-dynamics/blob/master/libraries/YarpPlugins/KdlSolver/DeviceDriverImpl.cpp *** //

    }
    void TeoCheckSelfCollisionsLibrary::configureCollisionObjects(){
        yarp::os::Property fullConfig;
        fullConfig.fromConfigFile(m_selfCollisionsFileFullPath.c_str());
        m_numCollisionObjects = fullConfig.check("numCollisionObjects", yarp::os::Value(DEFAULT_NUM_COLLISION_OBJETCS)).asInt32();
        printf("numCollisionObjects: %d\n", m_numCollisionObjects);

        for (int collisionObjectIndex = 0; collisionObjectIndex < m_numCollisionObjects; collisionObjectIndex++)
        {
            std::string collisionObjectStr = "collisionObject_" + std::to_string(collisionObjectIndex);
            yarp::os::Bottle & bCollisionObject = fullConfig.findGroup(collisionObjectStr);
             if (!bCollisionObject.isNull())
            {
                int segment = bCollisionObject.check("segment", yarp::os::Value(0.0)).asInt32();
                m_segments.push_back(segment);

                yarp::os::Bottle collisionObjectOffset = bCollisionObject.findGroup("offset", "offset (SI units)").tail();
                m_offsetCollisionObjects.push_back(std::array<float, 3>{collisionObjectOffset.get(0).asFloat32(),
                                                                        collisionObjectOffset.get(1).asFloat32(),
                                                                        collisionObjectOffset.get(2).asFloat32()});

                yarp::os::Bottle collisionObjectBoxShape = bCollisionObject.findGroup("boxShape", "collision box shape (SI units)").tail();                                                        

                m_boxShapes.push_back(std::array<float, 3>{collisionObjectBoxShape.get(0).asFloat32(),
                                                            collisionObjectBoxShape.get(1).asFloat32(),
                                                            collisionObjectBoxShape.get(2).asFloat32()});                                                    
                printf("segment: %d\n", segment);
                printf("offset: %f %f %f\n", m_offsetCollisionObjects.back()[0], m_offsetCollisionObjects.back()[1], m_offsetCollisionObjects.back()[2]);
                printf("boxShape: %f %f %f\n", m_boxShapes.back()[0], m_boxShapes.back()[1], m_boxShapes.back()[2]);

                fcl::Transform3f tfTest;
                CollisionGeometryPtr_t collisionGeometryAux{new fcl::Boxf{m_boxShapes.back()[0], m_boxShapes.back()[1], m_boxShapes.back()[2]}};
                m_collisionObjects.push_back(fcl::CollisionObjectf{collisionGeometryAux, tfTest});
            }
            else{
                printf("Error: collisionObject %d not found in file %s\n", collisionObjectIndex, m_selfCollisionsFileFullPath.c_str());
            }
        }
    }

    KDL::JntArray TeoCheckSelfCollisionsLibrary::jointsDeg2Rad(const std::vector<double> &q)
    {
        KDL::JntArray qRad(q.size());
        for (int i = 0; i < q.size(); i++)
        {
            qRad(i) = q[i] * KDL::deg2rad;
            //printf("q(%d) = %f", i, q(i));
        }
        return qRad;
    }

    bool TeoCheckSelfCollisionsLibrary::jointsInsideBounds(const std::vector<double> &q){
        printf("%ld %ld %ld",q.size(),m_qmin.size(), m_qmax.size());

        if (q.size() != m_qmin.size() || q.size() != m_qmax.size())
        {
            throw std::runtime_error("q size is not equal to qmin and qmax size");
        }
        for (int i = 0; i < q.size(); i++)
        {
            printf("q: %f qmin: %f qmax: %f \n", q[i], m_qmin[i], m_qmax[i]);
            if (q[i] < (m_qmin[i]) || q[i] > (m_qmax[i]))
            {
                printf("Joints outside bounds\n");
                return false;
            }
        }
        printf("Joints inside bounds\n");
        return true;

    }
    void TeoCheckSelfCollisionsLibrary::getBoxShapes(std::vector<std::array<float,3>> & boxShapes){
        boxShapes = m_boxShapes;
    }

    void TeoCheckSelfCollisionsLibrary::getTransformations(std::vector<std::array<double,7>> & transformations){
        for(int i=0; i<m_collisionObjects.size(); i++){
            fcl::Quaternionf quat = m_collisionObjects[i].getQuatRotation();
            fcl::Vector3f translation = m_collisionObjects[i].getTranslation();

            // printf("Quat: %f %f %f %f\n", quat.x(), quat.y(), quat.z(), quat.w());
            // printf("Translation: %f %f %f\n", translation[0], translation[1], translation[2]);
            std::array<double,7> transformation = {quat.x(), quat.y(), quat.z(), quat.w(), translation[0], translation[1], translation[2]};
            transformations.push_back(transformation);
        }
    }
    bool TeoCheckSelfCollisionsLibrary::selfCollision()
    {
        // printf("SelfCollision()\n");
        // printf("m_collisionObjects.size(): %ld\n", m_collisionObjects.size());
        fcl::CollisionRequestf requestType;
        fcl::CollisionResultf collisionResult;
        for (int link1 = 0; link1<m_collisionObjects.size()-1; link1++)
        {
            int link2 = link1 + 2;
            while (link2 < m_collisionObjects.size())
            {   
                fcl::collide(&m_collisionObjects[link1], &m_collisionObjects[link2], requestType, collisionResult);
                if (collisionResult.isCollision())
                {
                    printf("collision betwenn segments %d and %d\n", link1, link2);
                    return true;
                }
                link2++;
            }
        }
        // printf("SelfCollision() not collide\n");
        return false;
    }

    double TeoCheckSelfCollisionsLibrary::minDistance(){
        fcl::DistanceRequestf request;
        request.enable_nearest_points = true;
        request.enable_signed_distance = true;
        fcl::DistanceResultf distanceResult;
        double minDistance = 1000;
        for (int link1 = 0; link1<m_collisionObjects.size(); link1++)
        {
            int link2 = link1 + 2;
            while (link2 < m_collisionObjects.size())
            {   
                fcl::distance(&m_collisionObjects[link1],&m_collisionObjects[link2], request, distanceResult);
                // printf("link %d %d minDistance: %f", link1, link2, distanceResult.min_distance);
                if(distanceResult.min_distance<minDistance){
                    
                    minDistance = distanceResult.min_distance;
                }
                link2++;
            }
        }
        return minDistance;
    }

    bool TeoCheckSelfCollisionsLibrary::twoLinksDistance(const std::vector<double> &q, int segment1, int segment2, double &minDistance){
        fcl::DistanceRequestf request;
        request.enable_nearest_points = true;
        request.enable_signed_distance = true;
        fcl::DistanceResultf distanceResult;

        std::vector<int> collisionIndex1;
        std::vector<int> collisionIndex2;
        for (int i=0; i<m_segments.size(); i++){
            printf("segment: %d\n", m_segments[i]);
            if(m_segments[i]==segment1){
                 collisionIndex1.push_back(i);
                printf("collisionIndex1: %d\n", i);
            }
            if(m_segments[i]==segment2){
                printf("collisionIndex2: %d\n", i);
                collisionIndex2.push_back(i);
            }
        }
        if(collisionIndex1.size()==0 || collisionIndex2.size()==0){
            printf("WARNING: Segment not found");
            return false;
        }

        // if(collisionIndex1 == (collisionIndex2+1))
        //     return 0.0;

        
        minDistance = 1000;
        for (int i=0; i<collisionIndex1.size(); i++)
        {
            for (int j=0; j<collisionIndex2.size(); j++)
            {
                fcl::distance(&m_collisionObjects[collisionIndex1[i]],&m_collisionObjects[collisionIndex2[j]], request, distanceResult);
                if(distanceResult.min_distance<minDistance){
                    minDistance = distanceResult.min_distance;
                }
            }
        }
    
    
        return true;
        

        // printf("%f\n",distanceResult.min_distance);
        // fcl::distance(collisionObjects[link1].computeAABB(), )
        // fcl::distance(&collisionObjects[link1], &collisionObjects[link2], requestType, distanceResult);        
    }


    bool TeoCheckSelfCollisionsLibrary::updateCollisionObjectsTransform(const std::vector<double> &q)
    {

        int kinematics_status;
        KDL::JntArray qRad = jointsDeg2Rad(q);
        KDL::ChainFkSolverPos_recursive fksolver(m_chain);
        //printf("\n");
        // if (jointsInsideBounds(q))
        // {
            // printf("jointsInsideBounds\n");
            for (int i = 0; i < m_segments.size(); i++)
            {
                KDL::Frame frameJoint;
                kinematics_status = fksolver.JntToCart(qRad, frameJoint, m_segments[i]);
                // printf("kinematic status %d\n", kinematics_status);
                // printf("%d %f %f %f\n",m_segments[i],frameJoint.p.x(), frameJoint.p.y(), frameJoint.p.z());

                KDL::Frame frameLink;
                frameLink.p = KDL::Vector(m_offsetCollisionObjects[i][0], m_offsetCollisionObjects[i][1], m_offsetCollisionObjects[i][2]);

                KDL::Frame frameCenterLink = frameJoint * frameLink;
                fcl::Vector3f translation(frameCenterLink.p[0], frameCenterLink.p[1], frameCenterLink.p[2]);
                double x, y, z, w;
                frameCenterLink.M.GetQuaternion(x, y, z, w);
                fcl::Quaternionf rotation(w, x, y, z);

                // printf("trans: %d %f %f %f \n", m_segments[i], translation[0], translation[1], translation[2]);
                // printf("rot: %f %f %f %f\n", x, y, z, w);
                m_collisionObjects[i].setTransform(rotation, translation);
            }
            return true;
        // }
        // return false;
    }
/************************************************************************/

/************************************************************************/

/************************************************************************/

} // namespace roboticslab
