# VehicleLatControlAnalysis
项目包含车辆横向控制相关算法的分析.

## 依赖库
- numpy
- control
- matplotlib

## 路径跟踪算法
###  非时间参考路径跟踪方法
基于非时间的参考的路径跟踪方法[Vehicle SMC Analysis (none-time reference)]最早应用于机器人的路径跟踪系统，通过引入非时间参考量代替时间参考量，解决传统路径跟踪方法中将期望轨迹视为时间函数的问题。
该方法选择移动机器人实际路径在某参考系下的x 轴投影作为非时间参考量。

根据如下控制系统框图，搭建仿真环境。

![](https://i.loli.net/2021/08/30/Kj1D6hL95owRl3X.png)

![](https://i.loli.net/2021/08/30/CfZDvlVpHtUyw4x.png)

![](https://i.loli.net/2021/08/30/H4gr6iRjPyI5qpQ.png)

![](https://i.loli.net/2021/08/30/otDGPEIfgTMK1n3.png)

![](https://i.loli.net/2021/08/30/dmx1R2K4Cfnu9kF.png)

![](https://i.loli.net/2021/08/30/whXu3Rsz798Ike5.png)

### 高阶滑模分析

待补充



### 基于后轴反馈的跟踪控制

![](https://i.loli.net/2021/08/30/kpv1Ix37YgfRSE5.png)

仿真框图如上，基于上述框图搭建仿真环境。

#### 余弦曲线跟踪

![](https://i.loli.net/2021/08/30/OscNB5hg2X3aQEn.png)

上图可知，跟踪效果很好。

![](https://i.loli.net/2021/08/30/Dt6xKvG4a79M5uN.png)

输出控制抖动也较小。

#### 圆弧跟踪

![](https://i.loli.net/2021/08/30/DHR8nweaVcFCj5o.png)

![](https://i.loli.net/2021/08/30/5NIa2KPkSYX31vh.png)

### Stanley跟踪

基于Stanley的跟踪方法

#### 三角函数曲线跟踪

![](https://i.loli.net/2021/08/30/5NSV2konO3qphI4.png)

![](https://i.loli.net/2021/08/30/nKYLzQj453EyukG.png)

#### 圆弧跟踪

![](https://i.loli.net/2021/08/30/RvgOm6ZT9qFjQuG.png)

![](https://i.loli.net/2021/08/30/6OWN4KpDbSQqgmj.png)

## 参考

- [控制算法-非时间参考的车辆路径跟踪](https://zgh551.github.io/2019/12/23/%E6%8E%A7%E5%88%B6%E7%AE%97%E6%B3%95-%E9%9D%9E%E6%97%B6%E9%97%B4%E5%8F%82%E8%80%83%E7%9A%84%E8%BD%A6%E8%BE%86%E8%B7%AF%E5%BE%84%E8%B7%9F%E8%B8%AA/)
- [控制算法-后轮位置反馈](https://zgh551.github.io/2020/02/26/%E6%8E%A7%E5%88%B6%E7%AE%97%E6%B3%95-%E5%90%8E%E8%BD%AE%E4%BD%8D%E7%BD%AE%E5%8F%8D%E9%A6%88/)

- [控制算法-Stanley法](https://zgh551.github.io/2020/02/23/%E6%8E%A7%E5%88%B6%E7%AE%97%E6%B3%95-Stanley%E6%B3%95/)
