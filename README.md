## How to run

```
source install/setup.sh
ros2 run rosgpt rosgpt
```

```
source install/setup.sh
ros2 run rosgpt rosgpt_client_node 
```

```
source install/setup.sh
ros2 run rosgpt rosgptparser_drone 
```

```
source install/setup.sh
cd src/sjtu_drone/rosgpt/webapp
npm run dev
```

```
source install/setup.sh
ros2 run rosbridge_server rosbridge_websocket
```

```
source install/setup.sh
ros2 launch  sjtu_drone_bringup sjtu_drone_bringup.launch.py
```

## Credits

Simulation adapted from: https://github.com/NovoG93/sjtu_drone

```
@article{koubaa2023rosgpt,
  title={ROSGPT: Next-Generation Human-Robot Interaction with ChatGPT and ROS},
  author={Koubaa, Anis},
  journal={Preprints.org},
  year={2023},
  volume={2023},
  pages={2023040827},
  doi={10.20944/preprints202304.0827.v2}
}

```
