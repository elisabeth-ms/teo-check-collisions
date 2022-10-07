// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CollisionsVisualization.hpp"

#include <cstdio>

using namespace roboticslab;

/************************************************************************/

bool CollisionsVisualization::configure(yarp::os::ResourceFinder & rf)
{

    std::printf("--------------------------------------------------------------\n");

    if (rf.check("help"))
    {
        std::printf("CollisionsVisualization options:\n");
        std::printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        //std::printf("\t--file (default: \"%s\")\n", fileName.c_str());
    }




    rf.setDefaultConfigFile("CollisionsVisualization.ini");

    m_robot = rf.find("robot").asString();
    m_deviceName = rf.find("deviceName").asString();
    m_frameId = rf.find("frameId").asString();
    if(!openDevices())
        return false;
    
    //  Getting the limits of each joint
    printf("---- Joint limits of %s\n",m_deviceName.c_str());
    m_qmin.resize(m_numJoints);
    m_qmax.resize(m_numJoints);

    for(unsigned int joint = 0; joint < m_numJoints; joint++){
        double min, max;
        m_iControlLimits->getLimits(joint, &min, &max);
        m_qmin[joint] = min;
        m_qmax[joint] = max;
    }

    rf.setDefaultContext("kinematics");
    std::string kinematicsFileFullPath = rf.findFileByName( "teo-trunk-rightArm-fetch.ini" );

    rf.setDefaultContext("teoCheckSelfCollisions");
    std::string selfCollisionsFileFullPath = rf.findFileByName( "teo-trunk-RightArm-fetch-collisions.ini");

    rf.setDefaultContext("teoCheckCollisions");
    std::string fixedObjectsFileFullPath = rf.findFileByName("fixed-table-collision.ini");


    m_checkCollisions = new TeoCheckCollisionsLibrary(fixedObjectsFileFullPath);
    m_checkCollisions->setSelfCollisionsFileFullPath(selfCollisionsFileFullPath);
    m_checkCollisions->setKinematicsFileFullPath(kinematicsFileFullPath);
    m_checkCollisions->setQMin(m_qmin);
    m_checkCollisions->setQMax(m_qmax);
    m_checkCollisions->configureCollisionObjects();
    m_checkCollisions->getBoxShapes(m_boxShapes);
    m_checkCollisions->configureEnvironmentFixedObjects();
    m_checkCollisions->getBoxShapesFixedObjects(m_boxShapesFixedObjects);

    m_rosNode = new yarp::os::Node("/collisionsVisualization");


    m_collisionObjectsTopic = new yarp::os::Publisher<yarp::rosmsg::visualization_msgs::MarkerArray>;

    if (m_collisionObjectsTopic->topic("/collisionObjects")==false)
    {
        yError("Error opening collisionObjects topic.\n");
    }
    else{
        yInfo("Opening collisionObjects topic succesfully.\n");
    
    }

    m_robotCollisionTopic = new yarp::os::Publisher<yarp::rosmsg::visualization_msgs::MarkerArray>;

    if (m_robotCollisionTopic->topic("/robotCollisionObjects")==false)
    {
        yError("Error opening robotCollisionObjects topic.\n");
    }
    else{
        yInfo("Opening robotCollisionObjects topic succesfully.\n");
    
    }


    m_port.open("/client");

    yarp::os::Network::connect("/client", "/getGraspingPoses/rpc:s");
    


    yarp::os::Time::delay(1);

    std::printf("--------------------------------------------------------------\n");



    return true;
}

/************************************************************************/

bool CollisionsVisualization::openDevices(){
    
    yInfo() <<"Lets open the devices";
    
    yarp::os::Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/"+m_robot+"/"+m_deviceName);
    options.put("local", "/" +m_robot + "/"+m_deviceName);
    m_device.open(options);
    if (!m_device.isValid())
    {
        yError() << "Robot "<<m_deviceName<<" device not available";
        m_device.close();
        yarp::os::Network::fini();
        return false;
    }

    // connecting our device with "IEncoders" interface
    if (!m_device.view(m_iEncoders))
    {
        yError() << "Problems acquiring IEncoders interface in "<<m_deviceName;
        return false;
    }
    else
    {
        yInfo() << "Acquired IEncoders interface in "<<m_deviceName;
        if (!m_iEncoders->getAxes(&m_numJoints))
            yError() << "Problems acquiring numJoints";
        else
            yWarning() << "Number of joints:" << m_numJoints;
    }

    if (!m_device.view(m_iControlLimits))
    {
        yError() << "Could not view iControlLimits in "<<m_deviceName;
        return false;
    }

    yInfo()<<"Devices open";
    return true;
}

/************************************************************************/

double CollisionsVisualization::getPeriod()
{
    return 0.5;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/

bool CollisionsVisualization::updateModule()
{
    yInfo()<<"Module update";

    // get the current encoder positions
    std::vector<double> currentQ(m_numJoints);
    if(!m_iEncoders->getEncoders(currentQ.data())){
        yError() << " Failed getEncoders() of "<<m_deviceName;
        return false;
    }
    // update collisions objects transformations
    if(m_checkCollisions->jointsInsideBounds(currentQ))
        yInfo() << "Joints inside bounds";
    m_checkCollisions->updateCollisionObjectsTransform(currentQ);

    // yarp::rosmsg::visualization_msgs::Marker marker;
    // yarp::rosmsg::visualization_msgs::MarkerArray m_markerObjectsArray;

    // marker.header.frame_id = m_frameId;
    // static int counter = 0;
    // marker.header.stamp.nsec = 0;
    // marker.header.stamp.sec = 0;

    // marker.action = yarp::rosmsg::visualization_msgs::Marker::DELETEALL;
    // m_markerObjectsArray.markers.push_back(marker);
    // if (m_collisionObjectsTopic) {
    //     yInfo("Publish...\n");
    //     m_collisionObjectsTopic->write(m_markerObjectsArray);
    // }



    std::vector<int> label_idx; 
    std::vector<std::array<float,11>> params;
    m_markerArray.clear();
    m_markerObjectsArray.clear();
    getSuperquadrics(label_idx, params);
    if(label_idx.size() != 0){
        m_checkCollisions->setSuperquadrics(label_idx, params);
        m_checkCollisions->updateEnvironmentCollisionObjects();
        // m_checkCollisions->checkNodeTypes();
        std::vector<ShapeCollisionObject> auxcollisionObjectsShape;
        m_checkCollisions->getObjectsShape(auxcollisionObjectsShape);

        m_collisionObjectsShape.clear();
        copy(auxcollisionObjectsShape.begin(), auxcollisionObjectsShape.end(), back_inserter(m_collisionObjectsShape)); 

        yInfo()<<"collisionObjectsShape.size(): "<<m_collisionObjectsShape.size();
        yInfo()<<m_collisionObjectsShape[0].shape;

        
        for(int index = 0; index<m_collisionObjectsShape.size(); index++)
            addMarkerShape(index, std::array<float, 4>{0,1,0,1});
    }

  

    std::vector<std::array<double,7>>transformations;
    m_checkCollisions->getTransformations(transformations);


    for(unsigned int i=0; i<transformations.size(); i++){
        addMarker(i, transformations[i], m_boxShapes[i], std::array<float, 4>{0,1,0,1});
    }

    std::vector<std::array<double,7>>fixedObjectTransformations;
    m_checkCollisions->getFixedObjectTransformations(fixedObjectTransformations);
    for(unsigned int i=0; i<fixedObjectTransformations.size(); i++){
        addMarker(i+transformations.size(), fixedObjectTransformations[i], m_boxShapesFixedObjects[i], std::array<float, 4>{0,0,1,1});
    }



    // for(int idx=0; idx<collisionObjectsShape.size(); idx++){
    //     yInfo()<<"idx: "<<idx;
    //     yInfo()<<"shape: "<<collisionObjectsShape[idx].shape;
    //     addMarkerShape(collisionObjectsShape[idx], std::array<float, 4>{0,1,0,1});
    // }

    // m_checkCollisions->collision();

    // yInfo()<<"Min Distance: "<< m_checkCollisions->minDistance();
 
    // // publish the collision objects
    // double minDistance;
    // m_checkCollisions->twoLinksDistance(currentQ, 0, 3, minDistance);
    // yInfo()<<"Distance between objects: "<< minDistance;


    if (m_collisionObjectsTopic) {
        yInfo("Publish...\n");
        m_collisionObjectsTopic->write(m_markerObjectsArray);
    }

    if (m_robotCollisionTopic) {
        yInfo("Publish...\n");
        m_robotCollisionTopic->write(m_markerArray);
    }
    return true;
}

/************************************************************************/

bool CollisionsVisualization::interruptModule()
{
    return true;
}

bool CollisionsVisualization::addMarkerShape(const int index, const std::array<float, 4> &rgba){
    yInfo()<<"Add marker shape: ";
    yarp::rosmsg::visualization_msgs::Marker marker;

    ShapeCollisionObject col = m_collisionObjectsShape[index];
    marker.header.stamp = yarp::os::Time::now();
    // yInfo()<<"Frame id: "<<m_frameId;
    marker.header.frame_id = m_frameId;
    marker.header.seq = 0;
    marker.id = index;
    marker.action = yarp::rosmsg::visualization_msgs::Marker::ADD;
    yarp::rosmsg::geometry_msgs::Point p;
    p.x = col.transform[4];
    p.y = col.transform[5];
    p.z = col.transform[6];
    marker.pose.position = p;
    marker.pose.orientation.x = col.transform[0];
    marker.pose.orientation.y = col.transform[1];
    marker.pose.orientation.z = col.transform[2];
    marker.pose.orientation.w = col.transform[3];
    if(col.shape == SHAPE_TYPE::BOX)
    {
            yInfo()<<"AddMarkerShape: Box";
            marker.type = yarp::rosmsg::visualization_msgs::Marker::CUBE;
            marker.scale.x = 2*col.size[0];
            marker.scale.y = 2*col.size[1];
            marker.scale.z = 2*col.size[2];

            marker.color.a = rgba[3]; // Don't forget to set the alpha!
            marker.color.r = rgba[0];
            marker.color.g = rgba[1];
            marker.color.b = rgba[2];
            marker.header.stamp = yarp::os::Time::now();
    }
    else if(col.shape == SHAPE_TYPE::CYLINDER)
    {  
            yInfo()<<"AddMarkerShape: CYLINDER";
            marker.type = yarp::rosmsg::visualization_msgs::Marker::CYLINDER;
            marker.scale.x = 2*col.size[0];
            marker.scale.y = 2*col.size[0];
            marker.scale.z = 2*col.size[1];

            marker.color.a = rgba[3]; // Don't forget to set the alpha!
            marker.color.r = rgba[0];
            marker.color.g = rgba[1];
            marker.color.b = rgba[2];
            marker.header.stamp = yarp::os::Time::now();
    }
    else if(col.shape == SHAPE_TYPE::ELLIPSOID)
    {           
            yInfo()<<"AddMarkerShape: ELLIPSOID";
            marker.type = yarp::rosmsg::visualization_msgs::Marker::SPHERE;
            marker.scale.x = 2*col.size[0];
            marker.scale.y = 2*col.size[1];
            marker.scale.z = 2*col.size[2];

            marker.color.a = rgba[3]; // Don't forget to set the alpha!
            marker.color.r = rgba[0];
            marker.color.g = rgba[1];
            marker.color.b = rgba[2];
            marker.header.stamp = yarp::os::Time::now();

    }

    m_markerObjectsArray.markers.push_back(marker);
    return true;
}


/************************************************************************/
bool CollisionsVisualization::addMarker(const int numberLink, const std::array<double,7>& transformation, const std::array<float,3> & boxSize, const std::array<float, 4> &rgba){
    yarp::rosmsg::visualization_msgs::Marker marker;

    marker.header.stamp = yarp::os::Time::now();
    // yInfo()<<"Frame id: "<<m_frameId;
    marker.header.frame_id = m_frameId;
    marker.header.seq = numberLink;
    marker.id = numberLink;
    marker.action = yarp::rosmsg::visualization_msgs::Marker::ADD;
    marker.type = yarp::rosmsg::visualization_msgs::Marker::CUBE;
    yarp::rosmsg::geometry_msgs::Point p;
    p.x = transformation[4];
    p.y = transformation[5];
    p.z = transformation[6];
    marker.pose.position = p;
    marker.pose.orientation.x = transformation[0];
    marker.pose.orientation.y = transformation[1];
    marker.pose.orientation.z = transformation[2];
    marker.pose.orientation.w = transformation[3];

    marker.scale.x = boxSize[0];
    marker.scale.y = boxSize[1];
    marker.scale.z = boxSize[2];

    marker.color.a = rgba[3]; // Don't forget to set the alpha!
    marker.color.r = rgba[0];
    marker.color.g = rgba[1];
    marker.color.b = rgba[2];
    marker.header.stamp = yarp::os::Time::now();
    m_markerArray.markers.push_back(marker);

    return true;
}
/************************************************************************/
void CollisionsVisualization::getSuperquadrics(std::vector<int> &label_idx, std::vector<std::array<float,11>> &params){
    yInfo()<<"Lets get the superquadrics";
    yarp::os::Bottle cmd;
    cmd.addString("gsup");
    yInfo()<<"Sending message..."<<cmd.toString();
    yarp::os::Bottle response;
    m_port.write(cmd, response);
    yInfo()<<"Got response:"<<response.toString();


    for(int i=0; i<response.size(); i++){
        label_idx.push_back(response.get(i).find("label_idx").asInt32());
        std::array<float,11> object_params;
        object_params[0] = response.get(i).find("axes0").asFloat32();
        object_params[1] = response.get(i).find("axes1").asFloat32();
        object_params[2] = response.get(i).find("axes2").asFloat32();
        object_params[3] = response.get(i).find("e1").asFloat32();
        object_params[4] = response.get(i).find("e2").asFloat32();
        object_params[5] = response.get(i).find("x").asFloat32();
        object_params[6] = response.get(i).find("y").asFloat32();
        object_params[7] = response.get(i).find("z").asFloat32();
        object_params[8] = response.get(i).find("roll").asFloat32();
        object_params[9] = response.get(i).find("pitch").asFloat32();
        object_params[10] = response.get(i).find("yaw").asFloat32();
        params.push_back(object_params);

        yInfo()<<label_idx[i];
        yInfo()<<object_params;
    }
}
