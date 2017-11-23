# Particle Filter
---

**Kidnapped Vehicle Project**

The goals / steps of this project are the following:

* Particle filter localizes vehicle within the desired accuracy
* Program runs within specified time of 100 seconds
* Particle filter localizes the robot

[//]: # (References)
[simulator]: https://github.com/udacity/self-driving-car-sim/releases
[win 10 update]: https://support.microsoft.com/de-de/help/4028685/windows-get-the-windows-10-creators-update
[uWebSocketIO]: https://github.com/uWebSockets/uWebSockets
[linux on win 10]: https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/
[MinGW]: http://www.mingw.org/
[CMake]: https://cmake.org/install/
[udacity code]: https://github.com/udacity/CarND-Kidnapped-Vehicle-Project
[output image]: ./imgs/particle-filter.gif "Kidnapped Vehicle Project (Particle Filter)"

---

## Files Submitted & Code Quality

### 1. Submission includes all required files and every TODO task has been accomplished 

For this project, I have used the [Kidnapped Vehicle Project Starter Code][udacity code] from Udacity and I have modified the following two files:
```cpp
particle_filter.cpp
particle_filter.h
```

The ```particle_filter.cpp``` file reads in the GPS data and initializes the particles. I decided to initialize 100 particles for this project. After the first initialization, the state gets predicted in the `void ParticleFilter::prediction()` function on line 42. After the prediction, we update the weight of each particle in the `void ParticleFilter::updateWeights()` function. In this function, the observation points of the vehicle are transformed from vehicle coordinates to map coordinates (line 123 - 127) and the landmarks of the map are predicted by calculating the difference between every landmark's position and every particle's position (line 130 - 134). Then the transformed observations points and the predictions of the landmarks get associated with each other by calling the `void ParticleFilter::dataAssociation()` function on line 136. Afterwards, the weight of the particle is calculated on line 149.
After the calculation of the weights of each particle, the `void ParticleFilter::resample()` function on line 159 is called which creates new particles from the old ones in proportion to the importance weight.

### 2. Code must compile without errors

This project was done on Windows 10. In order to set up this project I had to:
* update my Windows 10 Version with the [Windows 10 Creators Update][win 10 update]
* install the [Linux Bash Shell][linux on win 10] (with Ubuntu 16.04) for Windows 10
* set up and install [uWebSocketIO][uWebSocketIO] through the Linux Bash Shell for Windows 10
* [download the simulator from Udacity][simulator]

**To update the Linux Bash Shell to Ubuntu 16.04 the Windows 10 Creators Update has to be installed!**

Also, [CMake][CMake] and a gcc/g++ compiler like [MinGW][MinGW] is required in order to compile and build the project.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory in the Linux Bash Shell.

1. `mkdir build`
2. `cd build`
3. `cmake .. -G "Unix Makefiles" && make` on Windows 10 or `cmake .. && make` on Linux or Mac
4. `./particle_filter`

Alternatively, some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. `./clean.sh`
2. `./build.sh`
3. `./run.sh`

Then the simulator has to be started and *Project 3: Kidnapped Vehicle* has to be selected. When everything is set up the Linux Bash Shell should print: 
```bash 
Listening to Port 4567
Connected
```

### 3. Final Output:
![Visualization][output image]

---

## Discussion

### 1. Briefly discuss any problems / issues you faced in your implementation of this project.
Even though the weight of every particle is initialized with 1.0 at the very beginning (line 36 in `particle_filter.cpp`), the weight of each particle has to be re-initialized in the `void ParticleFilter::updateWeights()` function (line 139) in order to keep the error nearly stable during the turn. Otherwise, the x and y errors increase and the program fails. 