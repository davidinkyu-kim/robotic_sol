# Robotic_HW
* Solution written by David Inkyu Kim(gimming9@gmail.com)
## Dependencies
* robotic_sol_interfaces (https://github.com/davidinkyu-kim/robotic_sol_interfaces)

## How to setup
1. Make a workspace, clone `robotic_sol` and `robotic_sol_interfaces` under the workspace
```
$ mkdir -p machina_ws/src
$ cd machina_ws
$ git clone https://github.com/davidinkyu-kim/robotic_sol.git
$ git clone https://github.com/davidinkyu-kim/robotic_sol_interfaces.git
```
2. Build 
```
$ cd ~/machina_ws
~/machina_ws$ colcon build
```

3. Launch through the script
```
~/machina_ws$ source install/setup.bash
~/machina_ws$ ros2 launch robotic_sol sensor_network_launch.py
```

4. Confirm the topic being published
```
## Open another terminal ##
$ cd ~/machina_ws
~/machina_ws$ source install/setup.bash
~/machina_ws$ ros2 topic hz /robotic_sol/msg/sensors_output

average rate: 500.216
        min: 0.002s max: 0.002s std dev: 0.00005s window: 502
average rate: 500.103
        min: 0.001s max: 0.003s std dev: 0.00007s window: 1002
average rate: 500.059
        min: 0.001s max: 0.003s std dev: 0.00007s window: 1502
```

## Nodes
### `sensor_node`
  - This node is a sensor simulator, providing simulated signals via socket. Multiple instances of sensors can be run on individual thread.

### `sensor_mgr_node`
  - This node manages sensors by (per sensor)
    1. Connect to a sensor via socket.
    2. Continuously request samples and store as `buffered_data`.
    3. Provide a service to return `latest_data` when requested
  - `number_of_samples` per call.
    - Is computed as 
    ```(integer)sensor_sample_rate/required_broadcast_rate - 1```.
    - Latency-wise, it is better to ask as many samples as possible per call, as the overhead and delay will be distributed amongst `n` samples.
    - For livelyness perspective, sensors should keep the data up as-fresh-as-possible. Client would like to broadcast live data, not the stale data. Therefore, as described in the requirement, at least one data should be updated within the `500hz` broadcast loop, meaning to deliver a signal with less than `2ms` of latency.
    - Therefore, the solution comes to maximize the number of samples per call while maintaining `< 2ms` latency per call.
    - For example, `sensor1` is sampled at `2000hz`, and number of samples per call is set as `4-1=3`, subtracting one to consider unidentified overhead and delay.
    
  - `timer_cb`
    - Each sensor uses same callback function for a timer with its own rate. Also, individual callback group is assigned per sensor so that multiple timers can run at the same time. The timer callback is not reentrant, so whenever callback is hung up on `sock.recv`, waiting for sensor signal, next cycle will happen once the loop is done.

  - `read_sensor_srv_`
    - Similar to the timer, each sensor uses same callback function for a service, with individual service server for concurrency. Target sensor is distinguished by the service topic name(e.g. `/robotic_sol/sensor1/srv/ReadSensor`), so service request does not contain identifier for sensor.

### `sensor_client_node`
  - This node call two sensor services and publishes as a topic at 500hz
  - A process loop runs on 500hz timer, and on each callback it runs
    1. Send asynchronous request per sensor. When the response is available, sensor data is updated
    2. If all sensors received at least one data, concatenate and publish as a `SensorsOutput` message of dynamic sized array
  - Depending on `sensor_mgr` configuration, response from the service can be slower than 500hz. The timer function is set as reentrant, so that `sensor_client_node` can keep up with 500hz cycle of broadcasting, regardless of server response.


## Context
The design of our cells in Machina Labs has evolved over the past years. Currently, each of our cells has two articulated industrial robots on rails (a total of 7 axes) and a frame with hydraulic clamps. For the parts to form correctly, we must exert and maintain a dynamic force during the forming in a very accurate location in space. Currently, each robot is equipped with a load cell. See a quick video about our process [here](https://www.youtube.com/watch?v=iqYMprTEXRI). We are using ROS2 to collect the data from the network and control the robots in real-time. As a robotic engineer, we keep developing different modules for our network to add features to the system.  
 
## Objective
The goal of This project is to build a ROS2 network that collects data from 3-DOF sensors and makes the filtered data available as a ROS service and topic. Since we cannot send a real sensor to all of our applicants, we made a simple simulator (sensor.py) that mimics the behavior of a real sensor but with random data. 
- The first task is to make a custom service for 3-DOF sensor 
- The second task is to make a ROS2 service server that continuously reads data from the sensor and has the latest filter data available for the client service that you make. 
- Finally, please make a simple client that calls two of these services and publishes them to a topic at 500Hz. Please keep in mind that your service servers can run slower than 500Hz. 
- You can define a second server in the simulator to modify the code and run two at the same time.
- You can check the example.py to see how to make calls to the sensor

## Grading Criteria
- We’re looking for code that is clean, readable, performant, and maintainable.
- The developer must think through the process of deploying and using the solution and provide the necessary documentation. 
- The sensor samples with 2000Hz, and you can request a specific number of samples in each call. Each call also has a ~1ms delay on top of the sampling time. We would like to hear your thought on picking the number of samples that you read in each call. 

## Submission
To submit the assignment, do the following:

1. Navigate to GitHub's project import page: [https://github.com/new/import](https://github.com/new/import)

2. In the box titled "Your old repository's clone URL", paste the homework repository's link: [https://github.com/Machina-Labs/robotic_hw](https://github.com/Machina-Labs/robotic_hw)

3. In the box titled "Repository Name", add a name for your local homework (ex. `Robotic_soln`)

4. Set the privacy level to "Public", then click "Begin Import" button at bottom of the page.

5. Develop your homework solution in the cloned repository and push it to GitHub when you're done. Extra points for good Git hygiene.

6. Send us the link to your repository.

## ROS2
Install instructions (Ubuntu): [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

ROS2 tutorials: [https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html)

