# lidar-data-annotation-interface
CSCE 482 Senior Capstone Design
## Repo Setup
### Clone
```bash
git clone git@github.tamu.edu:trevor-ledbetter/lidar-data-annotation-interface.git
```
### Catkins Workspace set up
```bash
cd lidar-data-annotation-interface
chmod +x ws_setup.sh
./ws_setup.sh
source ~/.bashrc
```
### Launching rqt plugin
#### Launch roscore on separate shell
```bash
roscore
```
#### Launch rqt_plugin 
```bash
rqt --standalone rqt_mypkg
```
#### Clean up and rebuild when switching branches 
##### Clean unecessary files for clean build
```bash
git clean -xfd
```

##### Build
```bash
catkin_make
```