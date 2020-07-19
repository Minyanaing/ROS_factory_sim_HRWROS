# ROS_factory_sim_HRWROS
Hello (Real) World with ROS – edxCourse_complete_code

This is the `/src` file for the course: Hello (Real) World with ROS – Robot Operating System offered by TUDelft(edX).

### Download requirements
Download the CCS(Course Command Shell) - [For Ubuntu 16.06](https://courses.edx.org/assets/courseware/v1/4beeefd3ac34401321609d7697f9097f/asset-v1:DelftX+ROS1x+1T2020+type@asset+block/singularity-container_2.6.1-1_amd64_ubuntu_xenial16.04.deb), [For Ubuntu 18.04](https://courses.edx.org/assets/courseware/v1/626dbd92c8dd641c5657f8d9ec43ead2/asset-v1:DelftX+ROS1x+1T2020+type@asset+block/singularity-container_2.6.1-1_amd64_ubuntu_bionic18.04.deb)

After downloading, navigate to a new terminal.
```
$ sudo dpkg -i $HOME/<path_of_download>/<download_filename>
$ sudo apt-get install -f
```
Download [singularity image](https://surfdrive.surf.nl/files/index.php/s/pp59nr2PLr2QGNg/download) & [Installation script](https://courses.edx.org/assets/courseware/v1/7d7f9ae82dfbbf2be2ce126022e63811/asset-v1:DelftX+ROS1x+1T2020+type@asset+block/install-hrwros-starter.sh) 

**The restriction of the installation script, the singularity image filename must be `hrwros-09.simg` and both file must be in the same directory**

```
$ bash $HOME/<path_downloaded_installation_script>/install-hrwros-starter.sh
```

Then, we can open the application from the menu bar, named **HRWROS**

Source ROS and create a **new workspace**
```
$ source /opt/ros/<distro>/setup.bash
$ rosdep update
$ mkdir -p $HOME/hrwros_ws/src/hrwros
$ cd $HOME/hrwros_ws
$ catkin b
```
**Note** Since the tutorial is inside the CCS, `catkin_make` is not allowed.

