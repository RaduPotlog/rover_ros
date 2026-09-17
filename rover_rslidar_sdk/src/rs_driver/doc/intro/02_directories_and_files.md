# 2 **Directory Structure**

## 2.1 Directories

Below are the directories of `rs_driver`.

+ `src` Source code to build the `rs_driver` library.
+ `demo` Demo apps based on the `rs_driver` library. It includes:
  + `demo_online.cpp` demo app to connect to an online LiDAR
  + `demo_online_muti_lidars.cpp` demo app to connect to multiple liDARs
  + `demo_pcap.cpp` demo app to parse PCAP file
+ `tool` Tool apps based on the `rs_driver` library.
  + `rs_driver_viewer.cpp` the point cloud visualization tool based on the PCL library.
  + `rs_driver_pcdsaver.cpp` A tool to save point cloud as PCD format.  On embedded Linux platform, the PCL library is unavailable, so `rs_driver_viewer` is unavailable either. `rs_driver_pcdsaver` is used instead.
+ `test` Unit test app based on `Google Test`. 
+ `doc` Help documents
  + `howto` Answers some frequently asked questions about `rs_driver`. For example, illustrations to `demo_online`/`demo_pcap`, and `rs_driver_viewer`, network configuration options, how to transform point cloud, how to port from v1.3.x to v15.x, how to split frames, how to handle packet loss and out of order, how to stamp point cloud, layout of points in point cloud, etc.
  + `intro` descriptions to `rs_driver`'s interface, such as parameters, error code, CMake macros.
  + `src_intro` documents to analysis `rs_driver`'s source code, such as design consideration of `rs_driver`, and some important implement details.
+ `win` `MSVC` project files with compilation options ready to compile  `demo_online`/`demo_pcap`/`rs_driver_viewer`。

```
├── src
│   └── rs_driver
├── demo
│   ├── demo_online.cpp
│   ├── demo_online_multi_lidars.cpp
│   └── demo_pcap.cpp
├── tool
│   ├── rs_driver_pcdsaver.cpp
│   └── rs_driver_viewer.cpp
├── test
├── doc
│   ├── howto
│   ├── intro
│   └── src_intro
├── win
├── CMakeLists.txt
├── README_CN.md
├── README.md
├── CHANGELOG.md
├── LICENSE
```

