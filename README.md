# 实验二 使用PCL过滤器以及读取LAS文件

本次实验主要使用了`PCL`库读取`pcd`点云文件，并了解了`filter`的原理和掌握了其使用。

## 项目结构

- `bin`                                               *项目编译后的可执行程序*
- `---data`                                        *程序需要使用的点云数据*
- `PointCloud`                                *`PCL`中使用`filter`的源代码*
- `PointCloudTest02_readlas`      *使用`pdal`库读取las文件的源代码*

## 使用说明

`bin/PointCloudTest02.exe`

​	使用了`conio.h`里的`_kbhit`重写了键盘事件监听，需要在激活控制台窗口的情况下才能生效（避免与`pcl`默认的键盘事件监听冲突），具体监听事件如下

|    键盘事件     | 效果                             |
| :-------------: | -------------------------------- |
| `press key C/c` | 重置相机参数，使其回到默认视角   |
| `press key P/p` | 使用直通滤波器对点云数据进行处理 |
| `press key R/r` | 使用ROR滤波器对点云数据进行处理  |
| `press key S/s` | 使用SOR滤波器对点云数据进行处理  |
| `press key I/i` | 使点云数据回到初始状态           |
| `press key V/v` | 使用体素滤波器对点云数据进行处理 |

> 注意：以上滤波器相互之间并不会叠加，即使用直通滤波器后再使用ROR滤波器并不会对数据先进行直通滤波器处理再进行ROR滤波器处理。

`bin/PointCloudTest02_readlas.exe`

​	可以直接运行，则 默认加载数据集`epsg_4326.las`，也可以在命令行中运行，`./bin/PointCloudTest02_readlas.exe -f filename`，读取las文件并可视化。