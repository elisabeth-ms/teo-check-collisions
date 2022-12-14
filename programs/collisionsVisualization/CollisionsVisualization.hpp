// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __COLLISIONS_VISUALIZATION_HPP__
#define __COLLISIONS_VISUALIZATION_HPP__

#include <yarp/os/RFModule.h>
#include <yarp/dev/all.h>
#include <yarp/os/Node.h>
#include <yarp/os/Publisher.h>
#include <yarp/rosmsg/std_msgs/String.h>
#include <yarp/rosmsg/visualization_msgs/Marker.h>
#include <yarp/rosmsg/visualization_msgs/MarkerArray.h>
#include <yarp/os/all.h>

#include <TeoCheckCollisionsLibrary.hpp>

namespace roboticslab
{

/**
 * @ingroup collisionsVisualization
 *
 * @brief collisionsVisualization
 */
class CollisionsVisualization : public yarp::os::RFModule
{
public:
    virtual bool configure(yarp::os::ResourceFinder & rf) override;

protected:
    virtual bool interruptModule() override;
    virtual double getPeriod() override;
    virtual bool updateModule() override;
    bool openDevices();

    void getSuperquadrics(std::vector<int> &label_idx,  std::vector<std::array<float,11>> &params);
    bool addMarkerShape(const int index, const std::array<float, 4> &rgba);

    std::string m_robot;
    std::string m_deviceName;
    std::string m_frameId;
    yarp::dev::PolyDriver m_device;
    yarp::dev::IEncoders *m_iEncoders;
    yarp::dev::IControlLimits *m_iControlLimits;
    yarp::os::RpcClient m_port;

    int m_numJoints;

    TeoCheckCollisionsLibrary * m_checkCollisions;
    std::vector<double> m_qmin;
    std::vector<double> m_qmax;

    std::vector<std::array<float,3>>m_boxShapes;
    std::vector<std::array<float,3>>m_boxShapesFixedObjects;
    std::vector<ShapeCollisionObject> m_collisionObjectsShape;

    //Rviz visualization
    yarp::os::Node * m_rosNode;
    yarp::rosmsg::visualization_msgs::MarkerArray m_markerArray;
    yarp::rosmsg::visualization_msgs::MarkerArray m_markerObjectsArray;


    bool addMarker(const int numberLink, const std::array<double,7>& transformation, const std::array<float,3> & boxSize, const std::array<float, 4> &rgba);
    yarp::os::Publisher<yarp::rosmsg::visualization_msgs::MarkerArray>* m_robotCollisionTopic;
    yarp::os::Publisher<yarp::rosmsg::visualization_msgs::MarkerArray>* m_collisionObjectsTopic;


};

} // namespace roboticslab

#endif // __COLLISIONS_VISUALIZATION_HPP__
