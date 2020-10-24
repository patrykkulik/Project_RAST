# Compile ORB-SLAM2 with RTAB-MAP
RTAB-MAP has provided with the option to compile with ORB-SLAM2 using ORB's odometry. But no binary installation is available so RTAB-MAP has to be compiled from source.

## Compile ORB-SLAM2
Detailed instuctions can be found at: 
```
https://github.com/raulmur/ORB_SLAM2
```
Install all the dependencies as instructed. (Note you do not have to download DBoW2 and g2o as they are included inside ORB). Before compiling ORB-SLAM2, a patch needs to be applied. Download the ORB.patch and copy it inside the ORB directory and open the directory in terminal:
```
git apply ORB.patch
```
After this you can compile as instructed. 

## Compile RTAB-MAP
Follow the instructions in the Build from Source section for step 0 and 1:
```
https://github.com/introlab/rtabmap_ros
```
For step 2:
```
$ cd ~
$ git clone https://github.com/introlab/rtabmap.git rtabmap
$ cd rtabmap/build
$ cmake -DWITH_G2O=OFF ..  [<---double dots included]
$ make
$ sudo make install
```
Note this is slightly different from the official instruction (Remove g2o dependency). After this follow step 3 from RTABMAP's github and compile rtabmap_ros. If you use Parallels desktop to run Ubuntu, you might encounter a compilation error when compiling rtabmap_ros. You need to disable 3D acceleration from Parallels control centre and restart. 

## Launch RTAB-MAP with ORB odometry
A sample launch file is attached, modify the name of the topics accordingly and launch. <br/>
Note that if you encounter:
```
rtabmap: error while loading shared libraries: libg2o.so: cannot open shared object file: No such file or directory
```
You need to add:
```
$ export LD_LIBRARY_PATH=/home/mathieu/workspace/ORB_SLAM2/lib:/home/mathieu/workspace/ORB_SLAM2/Thirdparty/g2o/lib:/home/mathieu/workspace/ORB_SLAM2/Thirdparty/DBoW2/lib:$LD_LIBRARY_PATH
```
to your .bashrc file by using:
```
gedit ~/.bashrc
```
Please change the directory to fit your own.
