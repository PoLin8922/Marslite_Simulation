#!/usr/bin/env python3
import numpy as np
from scipy.stats import multivariate_normal
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 定义均值向量
mean = np.array([0, 0])

# 协方差矩阵
covariance = np.array([[0.1, 0.0], [0.0, 180.0]])

# 创建多变量正态分布对象
mv_normal = multivariate_normal(mean=mean, cov=covariance)

# 生成二维网格
x, y = np.mgrid[-5:5:.01, -5:5:.01]
pos = np.dstack((x, y))

# 计算概率密度函数的值
pdf_values = mv_normal.pdf(pos)
print(mv_normal.pdf([0, 0]))
print(mv_normal.pdf([0.1, 0]))
print(mv_normal.pdf([0.5, 0]))
print(mv_normal.pdf([1.5, 0]))
print("-----------")
print(mv_normal.pdf([0, 0]))
print(mv_normal.pdf([0.0, 1]))
print(mv_normal.pdf([0.0, 3]))
print(mv_normal.pdf([0.0, 5]))
print(mv_normal.pdf([0.0, 10]))
print("-----------")
print(mv_normal.pdf([0, 0]))
print(mv_normal.pdf([0.1, 1]))
print(mv_normal.pdf([0.1, 5]))
print(mv_normal.pdf([0.1, 10]))
print(mv_normal.pdf([0.1, 15]))
print("-----------")
print(mv_normal.pdf([0, 0]))
print(mv_normal.pdf([0.3, 1]))
print(mv_normal.pdf([0.3, 5]))
print(mv_normal.pdf([0.3, 10]))
print(mv_normal.pdf([0.3, 15]))

# 绘制三维图
fig_3d = plt.figure(figsize=(8, 6))
ax_3d = fig_3d.add_subplot(111, projection='3d')

# 将网格和概率密度函数的值传递给3D图
surf = ax_3d.plot_surface(x, y, pdf_values, cmap='viridis')

# 标注轴
ax_3d.set_xlabel('X')
ax_3d.set_ylabel('Y')
ax_3d.set_zlabel('PDF')

# 添加颜色条
fig_3d.colorbar(surf, ax=ax_3d, shrink=0.5, aspect=10)

plt.title('Multivariate Normal Distribution PDF')
plt.show()

