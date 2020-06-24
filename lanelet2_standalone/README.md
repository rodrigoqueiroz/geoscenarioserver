# lanelet2_standalone

## Overview
This is a standalone lanelet2 **python3/python2** package for the [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) library, which **does not** require the ROS and catkin.

## Installation
pylanelet2 uses cmake and setuptools for building and packaging and is targeted towards Linux.

At least C++14 is required.

### Tested Configuration
* `Ubuntu 16.04 LTS`
* `python2.7/python3.5`

### Dependencies
* `Boost` (from 1.58, for lanelet2_io)
* `eigen3`
* `pugixml` (for lanelet2_io)
* `boost-python2/python3`
* `geographiclib` (for lanelet2_projection)

For Ubuntu:
```shell
sudo apt-get install -y cmake \
    libboost-all-dev  \
    libeigen3-dev \
    libgeographic-dev \
    libpugixml-dev \
    libpython-dev \
    libboost-python-dev \
    libpython3-all-dev \
    python3-pip \
    python3-setuptools \
    python-pip \
    python-setuptools
```

### Installing
* Building
```bash
git clone https://github.com/yuzhangbit/lanelet2_standalone.git
cd lanelet2_standalone
```
* Install lanelet2 for system python3:
```shell
sudo python3 setup.py install
```
* Or install lanelet2 for system python2:
```bash
sudo python setup.py install
```
* Or install lanelet2 for other python version:
```
/path/to/python setup.py install
```
* For whom wants to use the lanelet2 standalone c++ library **(optional)**, do
```bash
cd lanelet2_standalone
mkdir -p build
cd build
cmake ..
make -j$(nproc)
sudo make install
```

## Usage
### Example
```python
#!/usr/bin/env python
import lanelet2
from lanelet2 import core, io, projection, geometry, routing, traffic_rules
# or
from lanelet2.core import Lanelet, LineString3d, Point2d, Point3d, getId, LaneletMap, BoundingBox2d, BasicPoint2d
p = Point3d(getId(), 1, 100, 1000)
print(p)
assert p.x == 1
p.id = getId()
```

**Note:
Importing submodules from the raw `liblanelet2_core_pyapi.so` library like below is not supported.**
If you do this,
```python
from lanelet2.liblanelet2_core_pyapi import AttributeMap # not supported!!!
```
you will get an error as below:
```
>>> from lanelet2.liblanelet2_core_pyapi import AttributeMap
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
ImportError: No module named 'lanelet2.liblanelet2_core_pyapi'
```
Instead, you can use
```python
from lanelet2 import core
# use core.AttributeMap
# or
from lanelet2.core import AttributeMap
```
, which is more elegant.
