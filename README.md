强化学习与人工旅鼠算法融合的无人机路径规划研究
===

文件结构
---

##### 	1.**astar3dWeighted.m**

核心的加权A*三维路径搜索算法，根据障碍物、威胁、地形、风场等环境信息和权重向量计算一条可行路径，并返回路径及各项代价指标。

##### 	2.**buildEnvironmentMaps.m**

根据配置中的障碍物、威胁区域、地形参数和风场设置，构建三维栅格地图：障碍物占据图(`occupancy`)、威胁强度图(`threatMap`)、地形高度图(`terrainMap`)和三维风场(`windField`)。

##### 	3.buildParetoSet.m

利用优化过程中存档的非支配解和当前最优权重，生成并修剪Pareto前沿路径集，确保多样性并控制数量。

##### 	4.**clipVec.m**

辅助函数，将向量各元素限制在指定的下界和上界之间。

##### 	5.computePathMetrics.m

根据计算出的路径和权重，详细计算路径的总长度、威胁代价、能量消耗、平滑度、时间代价等多个指标，并组合成一个指标向量用于多目标评价。

##### 	6.**computeRouteStats.m**

对一条路径进行统计分析，包括节点数、直线距离、绕行系数、平均高度、爬升/下降距离、最大转弯角度和大于45°的转弯次数。

##### 	7.crowdingPrune.m

基于拥挤距离对多目标前沿解集进行修剪，保留指定数量的最具多样性代表解。

##### 	8.**defaultConfig.m**

返回默认的系统配置结构体，包含地图尺寸、起止点、飞行高度范围、权重上下限、种子权重、障碍物、威胁区、地形、风场以及PSO‑ALA‑RL算法参数等。

##### 	9.**dominates.m**

判断向量a是否Pareto支配向量b（a所有分量 ≤ b 且至少一个分量严格小于）。

##### 	10.**inputObstaclesFromConsole.m**

通过命令行交互输入障碍物（每个障碍物的x, y, 高度），用于无图形界面时的障碍物设定。

##### 	11.**inputSceneFromFigure.m**

通过图形窗口交互输入障碍物、威胁区域、起点和终点，用户可直接在二维平面点击并输入高度等信息，生成自定义场景。

##### 	12.**optimizeWeightsPSO_ALA_RL.m**

主优化函数，结合粒子群(PSO)、人工蜂群启发的ALA变异策略和强化学习(RL)自适应参数控制，搜索最优权重向量，同时维护一个非支配解存档。

##### 	13.**plotCostFigures.m**

绘制Pareto路径集中各路径在长度、风险、能耗、平滑、时间及总成本上的柱状图，并标记最优值。

##### 	14.**plotDetailedReport.m**

生成详细的路径分析表格，展示每条路径的各类指标（成本、节点数、绕行系数、转弯统计等），并自动标注每个指标的最优路径。

#####  	15.**plotSceneAndParetoPaths.m**

在三维图形中显示环境（障碍物柱体、威胁球体）以及Pareto前沿路径，并用不同颜色和线宽突出最优路径，同时绘制安全气泡。

##### 	16.**printParetoSummary.m**

在命令行打印Pareto路径集的摘要信息，包括各路径的节点数、总成本、长度、风险、能耗、平滑度、时间以及推荐路径的权重向量。

##### 	17.**run\_uav\_astar\_pso\_ala\_rl.m**

主程序入口。初始化配置，根据设定调用图形或命令行输入场景，构建环境地图，执行PSO‑ALA‑RL优化，生成Pareto路径集，并调用所有绘图函数输出结果。

##### 	18.**uav\_metrics**

汇集了与指标计算和存档维护相关的辅助函数（`updateArchive`、`dominates`、`crowdingPrune`、`buildParetoSet`、`computeRouteStats`、`computePathMetrics`），实际上这些函数在其他独立文件中已分别实现，本文件可能是整合备份。

##### 	19.**uav_plot.m**

汇集了与绘图相关的辅助函数（`printParetoSummary`、`plotSceneAndParetoPaths`、`plotCostFigures`、`plotDetailedReport`及内部绘图工具），同样可能是整合备份。

##### 	20.**updateArchive.m**

更新非支配解存档：判断新解是否被存档解支配，若不被支配则加入，同时剔除被其支配的存档解，并在存档超过最大容量时用拥挤距离剪枝。
