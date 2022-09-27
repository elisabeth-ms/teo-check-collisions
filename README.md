Installation:

```
git clone https://github.com/elisabeth-ms/teo-check-collisions.git
cd teo-check-collisions && mkdir build && cd build && cmake ..
make -j$(nproc)  # compile
sudo make install  # Install 
```

Device trunkAndRightArm:

```
yarpdev --device remotecontrolboardremapper --name /teoSim/trunkAndRightArm --robot remotecontrolboardremapper --localPortPrefix /local/bcc         --axesNames "(AxialWaist FrontalWaist FrontalRightShoulder SagittalRightShoulder AxialRightShoulder FrontalRightElbow AxialRightWrist FrontalRightWrist)"    --remoteControlBoards "(/teoSim/trunk /teoSim/rightArm)" --solver KdlTreeSolver --ik nrjl --from teo-trunk-rightArm-fetch.ini --eps 0.001 --maxIter 1000   
```

To visualize the collisions Model:

```
cd repos/teo-check-collisions/build
./bin/collisionsVisualization --robot teoSim --deviceName trunkAndRightArm --frameId waist

```
