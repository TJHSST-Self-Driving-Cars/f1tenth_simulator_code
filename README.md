# The F1TENTH - Riders

Sample project for the [https://riders.ai/challenge/40](Trinity College F1Tenth Challenge). 

## Installation

### Local Development with F1Tenth Gym

First clone this repository:

```bash
git clone https://gitlab.com/acrome-colab/riders-poc/f1tenth-gym-quickstart --config core.autocrlf=input
cd f1tenth-gym-quickstart
pip install --user -e gym
```

Finally, check if the repo is working properly:

```bash
cd pkg/src
python -m pkg.main
```

### Testing with Docker

First build required image:

```bash
docker-compose build agent
```

Create an `.env` file at the root of the project with following contents:

```bash
RACE_MAP_PATH=/catkin_ws/src/f1tenth_gym_ros/maps/SILVERSTONE.yaml
RACE_MAP_IMG_EXT=.png
F1TENTH_AGENT_NAME=a1
F1TENTH_AGENT_IMAGE=a1
RIDERS_CHALLENGE_ID=40
RIDERS_API_HOST=https://api.riders.ai
RIDERS_F1TENTH_HOST=https://f1tenth.riders.ai
```

Start ROSCore & F1Tenth ROS Bridge:

```bash
docker-compose up --force-recreate roscore-dev bridge-dev
```

Go to http://localhost:6080 , if everything worked properly until now, you should see simulator window. 
Finally, launch agent:   

```bash
docker-compose up --force-recreate agent-dev
``` 

You should see your agent moving in a single direction and crashing to wall in ~2.9 seconds.


## Making your own Driver

### Structure of a Driver

Let's take a look at the most basic Driver, which is in the file [drivers.py](./pkg/src/pkg/drivers.py)

```python
class SimpleDriver:    

    def process_lidar(self, ranges):
        speed = 5.0
        steering_angle = 0.0
        return speed, steering_angle
```

A Driver is just a class that has a ```process_lidar``` function which takes in LiDAR data and returns a speed to drive at along with a steering angle.

```ranges```: an array of 1080 distances (ranges) detected by the LiDAR scanner. As the LiDAR scanner takes readings for the full 360&deg;, the angle between each range is 2&pi;/1080 (in radians).

```steering_angle```: an angle in the range [-&pi;/2, &pi;/2], i.e. [-90&deg;, 90&deg;] in radians, with 0&deg; meaning straight ahead.

### Choosing a Driver

Let's look at the [main.py](./pkg/src/pkg/main.py) file. The section shown below is all we need to worry about.

```python
...
# import your drivers here
from pkg.drivers import GapFollower

# choose your drivers here (1-4)
drivers = [GapFollower()]

# choose your racetrack here (SILVERSTONE, SILVERSTONE_OBS)
RACETRACK = 'SILVERSTONE'
...
```

As shown in the comments above, we can import Drivers and then choose which ones we want to use. Let's import our SimpleDriver and choose it

```python
...
# import your drivers here
from pkg.drivers import GapFollower, SimpleDriver

# choose your drivers here (1-4)
drivers = [SimpleDriver()]

# choose your racetrack here (SILVERSTONE, SILVERSTONE_OBS)
RACETRACK = 'SILVERSTONE'
...
```

Now if you run the main.py file again, it uses our SimpleDriver

```bash
$ python main.py
```

To see some more complex processing, take a look at the GapFollower Driver which implements the [Follow The Gap Method](https://www.youtube.com/watch?v=7VLYP-z9hTw&ab_channel=Real-TimemLABUPenn)! Notice that it still has a ```process_lidar``` function which takes in LiDAR data and returns a speed and steering angle. That's all we'll ever need.

### Multi-Agent Racing

To race multiple Drivers against eachother, simply choose multiple Drivers! You may choose up to 4 drivers, but in practice the simulator will usually run very slowly if you choose more than 2. You may race the same Driver against itself by choosing it twice. If you try racing GapFollower against itself, you will find that it is not good at multi-agent racing! 

Here's how we would race GapFollower against SimpleDriver:

```python
# import your drivers here
from pkg.drivers import GapFollower, SimpleDriver

# choose your drivers here (1-4)
drivers = [GapFollower(), SimpleDriver()]

# choose your racetrack here (SILVERSTONE, SILVERSTONE_OBS)
RACETRACK = 'SILVERSTONE'
```

### Changing Map

There are 3 clear racetracks and 1 obstacles racetrack provided. To switch between them simply change the name of the selected `RACETRACK`

```python
# import your drivers here
from pkg.drivers import GapFollower, SimpleDriver

# choose your drivers here (1-4)
drivers = [GapFollower()]

# choose your racetrack here (SILVERSTONE, SILVERSTONE_OBS)
RACETRACK = 'SILVERSTONE_OBS'
```

## Known issues (from original repo)

- If you run the `pip install...` command above and then later change your file structure in some way, you may get errors with `gym` such as `module 'gym' has no attribute 'make'`. The solution to this is to re-run the command `pip install --user -e gym/`.

- On MacOS Big Sur and above, when rendering is turned on, you might encounter the error:
```
ImportError: Can't find framework /System/Library/Frameworks/OpenGL.framework.
```
You can fix the error by installing a newer version of pyglet:
```bash
$ pip3 install pyglet==1.5.11
```
And you might see an error similar to
```
gym 0.17.3 requires pyglet<=1.5.0,>=1.4.0, but you'll have pyglet 1.5.11 which is incompatible.
```
which could be ignored. The environment should still work without error.

## Citing
If you find this Gym environment useful, please consider citing:

```
@inproceedings{okelly2020f1tenth,
  title={F1TENTH: An Open-source Evaluation Environment for Continuous Control and Reinforcement Learning},
  author={O’Kelly, Matthew and Zheng, Hongrui and Karthik, Dhruv and Mangharam, Rahul},
  booktitle={NeurIPS 2019 Competition and Demonstration Track},
  pages={77--89},
  year={2020},
  organization={PMLR}
}
```
