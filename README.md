# imm-pybullet-util

IM^2 pybullet utility scripts for motion planning et al.

## Demo

![demo](./fig/2022-05-18-demo.gif)

### [Optional] Configure virtualenv

```bash
$ python3 -m venv env
$ source env/bin/activate
$ python3 -m pip install --upgrade pip
$ python3 -m pip install -r requirements.txt
```

<<<<<<< Updated upstream
=======
If you are using Anaconda instead of venv, delete the following line in "requirements.txt"
```bash
pkg_resources==0.0.0
```

>>>>>>> Stashed changes
### Run the example script

```bash
$ git clone https://github.com/caelan/motion-planners.git /tmp/motion-planners
$ python3 -m pip install -e.
$ MOTION_LIB_PATH=/tmp/motion-planners python3 examples/arm_motion.py
```
