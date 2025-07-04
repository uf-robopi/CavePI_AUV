This repository contains the design files, operational codes, and digital twin system's packages for the **CavePI AUV** (Autonomous Underwater Vehicle) from the RoboPI group at ECE, UF. The paper is accepted for publication at the Robotics: Science and Systems (RSS-2025). 

<p align="center">
  <img src="/assets/CavePI_Ginnie.gif" alt="cavepi_field" width="55%">
  <img src="/assets/cavepi_demo.png" alt="cavepi_demo" width="44%%">
</p>

## Resources

- **Pointers:** [[ArXiv Pre-print]](https://arxiv.org/pdf/2502.05384) [[Project Page]](https://robopi.ece.ufl.edu/cavepi.html)  [[Video Demo]](https://youtu.be/9BPpB1nu98E)
- The subdirectories contain the following 
  - **CAD_designs:** Contains the SolidWork design files for all the parts used to build CavePI.
  - **auv_nano:** Contains the codes for CavePI's onboard Jetson device.
  - **auv_rpi:** Contains the codes for CavePI's onboard Raspberry Pi 5 device.
  - **auv_dt:** Contains the codes for ROS-Gazebo based digital twin (DT) of CavePI.




### License and Bibliography 
- Please check the LICENSE file 
- Citation entry:
	```
  @inproceedings{gupta2025demonstrating,
    title={Demonstrating CavePI: Autonomous Exploration of Underwater Caves by Semantic Guidance},
    author={Gupta, Alankrit and Abdullah, Adnan and Li, Xianyao and Ramesh, Vaishnav and Rekleitis, Ioannis and Islam, Md Jahidul},
    booktitle={Robotics: Science and Systems (RSS)},
    year={2025}
  }
	```


### Acknowledgements
This research is supported in part by the U.S. National Science Foundation (NSF) grants #2330416, #2024741, and #1943205; and the University of Florida (UF) Research grant #132763. We are thankful to Dr. Nare Karapetyan, Ruo Chen, and David Blow for facilitating our field trials at Ginnie open-water springs and Blue Grotto. 