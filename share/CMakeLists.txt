# Copyright: Universidad Carlos III de Madrid (C) 2022
# Authors: Elisabeth Menendez
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

### Go through single applications
yarp_install(DIRECTORY contexts/teoCheckSelfCollisions
             DESTINATION ${TEO-CHECK-COLLISIONS_CONTEXTS_INSTALL_DIR})

yarp_install(DIRECTORY contexts/teoCheckCollisions
             DESTINATION ${TEO-CHECK-COLLISIONS_CONTEXTS_INSTALL_DIR})
