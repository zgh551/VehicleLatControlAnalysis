# VehicleSlidingControlAnalysis
本仓库使用滑模控制方法实现车辆横向控制

## PathTracking
###  非时间参考路径跟踪方法[Vehicle SMC Analysis (none-time reference)]
基于非时间的参考的路径跟踪方法最早应用于机器人的路径跟踪系统，通过引入非时间参考量代替时间参考量，解决传统路径跟踪方法中将期望轨迹视为时间函数的问题。
该方法选择移动机器人实际路径在某参考系下的x 轴投影作为非时间参考量，



根据如下控制系统框图，搭建仿真环境。
![](https://raw.githubusercontent.com/zgh551/FigureBed/master/img/20191225194515.png)

![](https://raw.githubusercontent.com/zgh551/FigureBed/master/img/20191225201717.png)

![](https://raw.githubusercontent.com/zgh551/FigureBed/master/img/20191225201732.png)

![](https://raw.githubusercontent.com/zgh551/FigureBed/master/img/20191225201746.png)

![](https://raw.githubusercontent.com/zgh551/FigureBed/master/img/20191225201811.png)

![](https://raw.githubusercontent.com/zgh551/FigureBed/master/img/20191225201827.png)

### 高阶滑模分析

