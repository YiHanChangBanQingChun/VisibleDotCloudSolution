import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import rcParams
import numpy as np

# 设置字体为黑体
rcParams['font.sans-serif'] = ['Times New Roman']
rcParams['axes.unicode_minus'] = False
rcParams['font.size'] = 15

# 读取CSV文件
df = pd.read_csv(r'D:\Users\admin\Downloads\chromedownload\dotcloud\point_cloud_data.csv')

# 提取Intensity和Z列的数据
intensity_data = df['Intensity']
z_data = df['Z']

# 绘制Intensity的直方图
plt.figure(figsize=(10, 5))
plt.hist(intensity_data, bins=30, color='blue', edgecolor='black')
plt.title('Intensity histogram')
plt.xlabel('Intensity')
plt.ylabel('count')
plt.grid(True)
plt.show()

# 绘制Z的直方图
plt.figure(figsize=(10, 5))
plt.hist(z_data, bins=30, color='green', edgecolor='black')
plt.title('Z histogram')
plt.xlabel('Z')
plt.ylabel('count')
plt.grid(True)
plt.show()

# 绘制Intensity和Z的热力图
plt.figure(figsize=(10, 5))
# 计算每个 bin 的计数
hb = plt.hexbin(intensity_data, z_data, gridsize=50, cmap='rainbow')
# 获取计数和边界
counts = hb.get_array()
verts = hb.get_offsets()
# 计算百分比
percentages = (counts / counts.sum()) * 100
# 创建新的颜色条，显示百分比
cb = plt.colorbar(hb, label='Percentage')
cb.set_ticks(np.linspace(0, counts.max(), 11))
cb.set_ticklabels([f'{p:.1f}%' for p in np.linspace(0, percentages.max(), 11)])
plt.title('Intensity vs Z Heatmap')
plt.xlabel('Intensity')
plt.ylabel('Z')
plt.grid(True)
plt.show()

# 绘制Z和intensity的热力图
plt.figure(figsize=(10, 5))
hb = plt.hexbin(z_data, intensity_data, gridsize=50, cmap='rainbow')
counts = hb.get_array()
verts = hb.get_offsets()
percentages = (counts / counts.sum()) * 100
cb = plt.colorbar(hb, label='Percentage')
cb.set_ticks(np.linspace(0, counts.max(), 11))
cb.set_ticklabels([f'{p:.1f}%' for p in np.linspace(0, percentages.max(), 11)])
plt.title('Z vs Intensity Heatmap')
plt.xlabel('Z')
plt.ylabel('Intensity')
plt.grid(True)
plt.show()
