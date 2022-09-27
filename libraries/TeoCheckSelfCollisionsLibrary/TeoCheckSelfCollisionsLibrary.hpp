// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TEO_CHECK_SELF_COLLISIONS_LIBRARY_HPP__
#define __TEO_CHECK_SELF_COLLISIONS_LIBRARY_HPP__

#include <string>
#include <yarp/os/Property.h>
#include <yarp/dev/all.h>
#include "KinematicRepresentation.hpp"
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/joint.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/rotationalinertia.hpp>
#include <kdl/segment.hpp>
#include <kdl/chain.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>

#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/narrowphase/distance.h>
using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometryf>;

namespace roboticslab
{

/**
 * @ingroup teoCheckSelfCollisionsLibrary
 *
 * @brief teoCheckSelfCollisionsLibrary
 */
class TeoCheckSelfCollisionsLibrary
{
public:
    TeoCheckSelfCollisionsLibrary()
    {}
    TeoCheckSelfCollisionsLibrary(const std::string & t_kinematicsFileFullPath,
                                 const std::string & t_selfCollisionsFileFullPath, 
                                 const std::vector<double> & t_qmin,
                                 const std::vector<double> & t_qmax):
    m_kinematicsFileFullPath(t_kinematicsFileFullPath),
    m_selfCollisionsFileFullPath(t_selfCollisionsFileFullPath)
    {
        yarp::os::Property fullConfig;

        if (!fullConfig.fromConfigFile(m_kinematicsFileFullPath)) //-- Put first because defaults to wiping out.
        {
            printf("Could not configure from %s \n",  m_kinematicsFileFullPath.c_str());
            return;
        }
        
    
        if(!getChainFromKinematicsFile(fullConfig)){
            return;
        }


        m_qmin = t_qmin;
        m_qmax = t_qmax;

        configureCollisionObjects();
    }

    void setKinematicsFileFullPath(const std::string & t_kinematicsFileFullPath){
        m_kinematicsFileFullPath = t_kinematicsFileFullPath;

        yarp::os::Property fullConfig;

        if (!fullConfig.fromConfigFile(m_kinematicsFileFullPath)) //-- Put first because defaults to wiping out.
        {
            printf("Could not configure from %s \n",  m_kinematicsFileFullPath.c_str());
            return;
        }
        
    
        if(!getChainFromKinematicsFile(fullConfig)){
            return;
        }

    }

    void setSelfCollisionsFileFullPath(const std::string & t_selfCollisionsFileFullPath){
        m_selfCollisionsFileFullPath = t_selfCollisionsFileFullPath;
    }

    void setQMin(const std::vector<double> & t_qmin){
        m_qmin = t_qmin;
    }

    void setQMax(const std::vector<double> & t_qmax){
        m_qmax = t_qmax;
    }

    bool getMatrixFromProperties(const yarp::os::Searchable & options, const std::string & tag, Eigen::MatrixXd & mat);
    
    bool getChainFromKinematicsFile(const yarp::os::Property &fullConfig);

    void configureCollisionObjects();

    bool updateCollisionObjectsTransform(const std::vector<double> &q);

    KDL::JntArray jointsDeg2Rad(const std::vector<double> &q);

    void getBoxShapes(std::vector<std::array<float,3>> & boxShapes);

    void getTransformations(std::vector<std::array<double,7>> & transformations);

    bool jointsInsideBounds(const std::vector<double> &q);

    bool selfCollision();
    
    double minDistance();

    bool twoLinksDistance(const std::vector<double> &q, int segment1, int segment2, double &minDistance);


protected:
    std::string m_kinematicsFileFullPath;
    std::string m_selfCollisionsFileFullPath;
    int m_numCollisionObjects;

    std::vector<int>m_segments;
    std::vector<fcl::CollisionObjectf> m_collisionObjects;
    std::vector<std::array<float,3>> m_offsetCollisionObjects;
    std::vector<std::array<float,3>> m_boxShapes;
    KDL::Chain m_chain;
    std::vector<double> m_qmin;
    std::vector<double> m_qmax;


};

} // namespace roboticslab

#endif // __TEO_CHECK_SELF_COLLISIONS_LIBRARY_HPP__
