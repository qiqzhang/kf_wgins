import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import random
 
# 读取文件
data = np.fromfile("IMU_ERR.txt", dtype='double', )
# 设置列名
data_columns = ['time (s)', 'X axes gyroscope biases (deg/h)', 'Y axes gyroscope biases (deg/h)',
                'Z axes gyroscope biases (deg/h)'
    , 'X axes accelerometer biases (mGal)', 'Y axes accelerometer biases (mGal)', 'Z axes accelerometer biases (mGal)'
    , 'X axes gyroscope scale factors (ppm)', 'Y axes gyroscope scale factors (ppm)',
                'Z axes gyroscope scale factors (ppm)'
    , 'X axes accelerometer scale factors (ppm)', 'Y axes accelerometer scale factors (ppm)',
                'Z axes accelerometer scale factors (ppm)']
# 381017个数据，13列，29309行数据
print(type(data.shape[0]))
row_n = int(data.shape[0]/13)
df = pd.DataFrame(data.reshape(row_n,13),columns=data_columns)
 
 
# xyz一起版， 折线
# 颜色列表
color_choice = ['#e791c8', '#d695e5', '#a8aceb', '#54acd2', '#4faaaf'
    , '#4aab98', '#4ab067', '#8ca547', '#b39d4a', '#dd9250'
    , '#ea96a3', ]
time = df['time (s)'] - df.iloc[0, 0]
 
# 设置画布
fig = plt.figure(figsize=(12, 8))
 
# 随机在color_choice中选择三个颜色
color_a, color_b, color_c = random.sample(color_choice, 3)
 
# 选择 -->  'X axes gyroscope biases (deg/h)' 列
col_name1 = data_columns[1]
# 线段图
plt.plot(time, df[col_name1], label=col_name1[:1], color=color_a)
# 散点图
#     plt.scatter(time, df[col_name1],label=col_name1[:1], color=color_a)
 
col_name2 = data_columns[2]
plt.plot(time, df[col_name2], label=col_name2[:1], color=color_b)
col_name3 = data_columns[2]
plt.plot(time, df[col_name3], label=col_name3[:1], color=color_c)
 
# 显示图例
plt.legend(frameon=False, fontsize='16')
plt.ylabel(col_name1[6:], fontsize='20')
plt.xlabel(data_columns[0], fontsize='20')
# 显示网格
plt.grid()
plt.xticks(fontsize='16')
plt.yticks(fontsize='16')
 
# 设置保存图片的的名称
file_name_save = 'xxx .jpg'
# 保存图片
plt.savefig(file_name_save, dpi=600, format='jpg', bbox_inches='tight')
 
#############################################################################
# 循环版
color_choice = ['#e791c8', '#d695e5', '#a8aceb', '#54acd2', '#4faaaf'
    , '#4aab98', '#4ab067', '#8ca547', '#b39d4a', '#dd9250'
    , '#ea96a3', ]
time = df['time (s)'] - df.iloc[0, 0]
 
for i in range(4):
    fig = plt.figure(figsize=(12, 8))
 
    number = i * 3
    color_a, color_b, color_c = random.sample(color_choice, 3)
    col_name1 = data_columns[number + 1]
    plt.plot(time, df[col_name1], label=col_name1[:1], color=color_a)
    col_name2 = data_columns[number + 2]
    plt.plot(time, df[col_name2], label=col_name2[:1], color=color_b)
    col_name3 = data_columns[number + 3]
    plt.plot(time, df[col_name3], label=col_name3[:1], color=color_c)
    plt.legend(frameon=False, fontsize='16')
    plt.ylabel(col_name1[6:], fontsize='20')
    plt.xlabel(data_columns[0], fontsize='20')
    plt.grid()
    plt.xticks(fontsize='16')
    plt.yticks(fontsize='16')
 
    file_name_save = col_name1[7:-7] + '.jpg'
    plt.savefig(file_name_save, dpi=600, format='jpg', bbox_inches='tight')
 
#################  STD code     #################
# 读取数据
data = np.fromfile("STD.bin", dtype='double', )
data_columns = ['time (s)', 'North 3-D position STD (m)', 'East 3-D position STD (m)', 'Down 3-D position STD (m)'
    , 'North 3-D velocity STD ', 'East 3-D velocity STD ', 'Down 3-D velocity STD '
    , 'Roll 3-D attitude STD (deg)', 'Pitch 3-D attitude STD (deg)', 'yaw 3-D attitude STD (deg)'
    , 'X axes gyroscope biases STD (deg/h)', 'Y axes gyroscope biases STD (deg/h)',
                'Z axes gyroscope biases STD (deg/h)'
    , 'X axes accelerometer biases STD (mGal)', 'Y axes accelerometer biases STD (mGal)',
                'Z axes accelerometer biases STD (mGal)'
    , 'X axes gyroscope scale factor STD (ppm)', 'Y axes gyroscope scale factor STD (ppm)',
                'Z axes gyroscope scale factor STD (ppm)'
    , 'X axes accelerometer scale factor STD (ppm)', 'Y axes accelerometer scale factor STD (ppm)',
                'Z axes accelerometer scale factor STD (ppm)']
row_n = int(data.shape[0]/22)
df = pd.DataFrame(data.reshape(row_n,22),columns=data_columns)
 
# 颜色
color_choice = ['#e791c8', '#d695e5', '#a8aceb', '#54acd2', '#4faaaf'
    , '#4aab98', '#4ab067', '#8ca547', '#b39d4a', '#dd9250'
    , '#ea96a3', ]
time = df['time (s)'] - df.iloc[0, 0]
 
for i in range(7):
    fig = plt.figure(figsize=(12, 8))
 
    number = i * 3
    color_a, color_b, color_c = random.sample(color_choice, 3)
    col_name1 = data_columns[number + 1]
    label_a = col_name1.split(' ')[0]
    label_y = ' '.join(col_name1.split(' ')[1:])
    if 'axes ' in label_y:
        label_y = label_y.strip('axes ')
 
    plt.plot(time, df[col_name1], label=label_a, color=color_a)
 
    col_name2 = data_columns[number + 2]
    label_b = col_name2.split(' ')[0]
    plt.plot(time, df[col_name2], label=label_b, color=color_b)
 
    col_name3 = data_columns[number + 3]
    label_c = col_name3.split(' ')[0]
    plt.plot(time, df[col_name3], label=label_c, color=color_c)
 
    plt.legend(frameon=False, fontsize='16')
    plt.ylabel(label_y, fontsize='20')
    plt.xlabel(data_columns[0], fontsize='20')
    plt.grid()
    plt.xticks(fontsize='16')
    plt.yticks(fontsize='16')
    if i >= 3:
        file_name_save = label_y[:-7] + '.jpg'
    else:
        file_name_save = label_y + '.jpg'
 
    plt.savefig(file_name_save, dpi=600, format='jpg', bbox_inches='tight')