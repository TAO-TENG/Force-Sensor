import numpy as np
import matplotlib.pyplot as plt

# 加载 .npy 文件
data = np.load('20231101_134524.npy')

# 绘制数据
plt.plot(data)
plt.xlabel('X Label')
plt.ylabel('Y Label')




plt.title('Title')
plt.show()