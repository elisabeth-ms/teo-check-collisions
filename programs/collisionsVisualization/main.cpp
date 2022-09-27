// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup teo_check_collisions_programs
 * \defgroup collisionsVisualization
 *
 * @brief Creates an instance of roboticslab::CollisionsVisualization
 *
 * @section collisionsVisualization_legal Legal
 *
 * Copyright: 2022 (C) Universidad Carlos III de Madrid
 *
 * Author: Elisabeth Menendez
 *
 * CopyPolicy: This program is free software; you can redistribute it and/or modify
 * it under the terms of the LGPLv2.1 or later
 *
 * <hr>
 *
 * This file can be edited at collisionsVisualization
 */

#include <cstdio>

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "CollisionsVisualization.hpp"

int main(int argc, char ** argv)
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("collisionsVisualization");
    rf.setDefaultConfigFile("collisionsVisualization.ini");
    rf.configure(argc, argv);

    roboticslab::CollisionsVisualization mod;

    if (rf.check("help"))
    {
        return mod.runModule(rf);
    }

    std::printf("Run \"%s --help\" for options.\n", argv[0]);
    std::printf("%s checking for yarp network... ", argv[0]);
    std::fflush(stdout);

    yarp::os::Network yarp;

    if (!yarp.checkNetwork())
    {
        std::fprintf(stderr, "[fail]\n%s found no yarp network (try running \"yarpserver &\"), bye!\n", argv[0]);
        return 1;
    } else std::printf("[ok]\n");

    return mod.runModule(rf);
}
