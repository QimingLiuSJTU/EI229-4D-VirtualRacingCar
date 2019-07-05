# EI229-4D-VirtualRacingCar
 在虚拟游戏环境中控制小车巡线、跟车和倒车。

## Description
  本项目是基于上海交通大学自动化系智能车实验室的虚拟仿真平台CyberTORCS进行的，在仿真环境中，小车接收到环境返回的参数，通过计算控制小车的油门、刹车、转向等控制变量。我们需要完成三个任务。巡线任务要求控制小车沿着道路中线尽可能快地行驶，最终分数与完成时间和与中线的误差有关。跟车任务要求小车跟着前方的一辆小车行驶，最终分数与跟车的精确程度有关，一旦撞上前方小车即宣告失败。倒车入库任务要求小车倒车进入地图中指定车位，最终分数与倒车精度和倒车时间有关。<br>
  
  小车的控制是高度灵活的，可以完成漂移、急刹车等复杂动作，仿真环境如下图所示。
  ![image](http://github.com/QimingLiuSJTU/EI229-4D-VirtualRacingCar/raw/master/1.jpg)
  ![image](http://github.com/QimingLiuSJTU/EI229-4D-VirtualRacingCar/raw/master/2.jpg)
  
  在VS中使用生成方式，得到相应的dll格式的文件，把其放入仿真环境相应文件夹中，即可运行观察代码控制的效果。

## File Functions
**driver_cruise.cpp**: 巡线代码<br>
**driver_follow.cpp**: 跟车代码<br>
**driver_parking.cpp**: 泊车代码<br>
**历史成绩记录.xlsx**: 历史调试的成绩，上述三个cpp文件是最好成绩<br>

## Project participant
赵寅杰、刘启明、严威豪 上海交通大学自动化系
