import numpy as np
import matplotlib.pyplot as plt

def gaussian(x, mean, std_dev):
    return np.exp(-((x - mean) ** 2) / (2 * std_dev ** 2))

def triangular(x, a, b, c):
    return np.maximum(0, np.minimum((x - a) / (b - a), (c - x) / (c - b)))

def trapezoidal(x, a, b, c, d):
    return np.maximum(0, np.minimum(np.minimum((x-a)/(b-a), 1), (d-x)/(d-c)))

def plot_membership_functions():
    # Gausion 3
    # x = np.linspace(0, 10, 10000)  

    # # vl_mean, vl_std_dev = 0.0, 0.1
    # l_mean, l_std_dev = 0.0, 1.5
    # m_mean, m_std_dev = 5, 1.5
    # h_mean, h_std_dev = 10.0, 1.5
    # # vh_mean, vh_std_dev = 1.0, 0.1

    # # vl_membership = gaussian(x, vl_mean, vl_std_dev)
    # l_membership = gaussian(x, l_mean, l_std_dev)
    # m_membership = gaussian(x, m_mean, m_std_dev)
    # h_membership = gaussian(x, h_mean, h_std_dev)
    # # vh_membership = gaussian(x, vh_mean, vh_std_dev)

    # plt.figure(figsize=(10, 6))
    # # plt.plot(x, vl_membership, 'k-', linewidth=4)
    # plt.plot(x, l_membership, 'deepskyblue', linewidth=3)  
    # plt.plot(x, m_membership, 'limegreen', linewidth=3)  
    # plt.plot(x, h_membership, 'tomato', linewidth=3)  
    # # plt.plot(x, vh_membership, 'k-', linewidth=4)
    # # plt.xlabel('Semantic Level', fontsize=14)  
    # # plt.ylabel('Degree of Membership', fontsize=14)  
    # plt.legend()
    # plt.grid(False)
    # plt.xticks(np.arange(0, 11, 1), fontsize=18)  
    # plt.yticks(np.arange(0, 1.1, 0.1), fontsize=18)  
    # plt.ylim(0, 1.1) 
    # plt.xlim(0, 10) 
    # plt.show()

    # # # Gausion 5
    # lb = 0
    # ub = 1

    # x = np.linspace(lb, ub, (int)(ub-lb)*1000)  
    
    # vl_mean, vl_std_dev = lb, (ub-lb)/10
    # l_mean, l_std_dev = lb + (ub-lb)/4, (ub-lb)/10
    # m_mean, m_std_dev = lb + (ub-lb)/2, (ub-lb)/10
    # h_mean, h_std_dev = lb + (ub-lb)*3/4, (ub-lb)/10
    # vh_mean, vh_std_dev = ub, (ub-lb)/10

    # vl_membership = gaussian(x, vl_mean, vl_std_dev)
    # l_membership = gaussian(x, l_mean, l_std_dev)
    # m_membership = gaussian(x, m_mean, m_std_dev)
    # h_membership = gaussian(x, h_mean, h_std_dev)
    # vh_membership = gaussian(x, vh_mean, vh_std_dev)

    # plt.figure(figsize=(10, 6))
    # plt.plot(x, vl_membership, '#6A0DAD', linewidth=3)  # Purple
    # plt.plot(x, l_membership, '#1E90FF', linewidth=3)   # Deep sky blue
    # plt.plot(x, m_membership, '#32CD32', linewidth=3)   # Lime green
    # plt.plot(x, h_membership, '#FF6347', linewidth=3)   # Tomato red
    # plt.plot(x, vh_membership, '#FFD700', linewidth=3)  # Gold
    # # plt.xlabel('Navigability', fontsize=14)  
    # # plt.ylabel('Degree of Membership', fontsize=14)  
    # plt.legend()
    # plt.grid(False)
    # plt.xticks(np.arange(lb, ub + (ub-lb)/10, (ub-lb)/10), fontsize=18)  
    # plt.yticks(np.arange(0, 1.1, 0.1), fontsize=18)  
    # plt.ylim(0, 1.1) 
    # plt.xlim(lb, ub) 
    # plt.show()


    # # Triangular
    # x = np.linspace(0, 1, 1000)  # 输入范围

    # # 隶属函数的参数设置
    # s_a, s_b, s_c = 0.0, 0.4, 0.8  # S (Small) 的参数
    # l_a, l_b, l_c = 0.6, 0.8, 1.0  # L (Large) 的参数

    # # 计算隶属度
    # s_membership = triangular(x, s_a, s_b, s_c)
    # l_membership = triangular(x, l_a, l_b, l_c)

    # # 绘制隶属函数
    # plt.figure(figsize=(10, 6))
    # plt.plot(x, s_membership, '#1E90FF', linewidth=3)
    # plt.plot(x, l_membership, '#FF6347', linewidth=3)
    # # plt.hlines(y=0, xmin=x[0], xmax=x[-1], color='black', linewidth=4)
    # plt.xlabel('Prediction Mode Indicator', fontsize=14)
    # plt.ylabel('Degree of Membership', fontsize=14)
    # plt.legend()
    # plt.grid(False)
    # plt.show()

    # trapezoidal`
    x = np.linspace(0, 1, 1000)  # 输入范围

    # 隶属函数的参数设置
    s_a, s_b, s_c, s_d = 0.0, 0.0, 0.4, 0.8  # S (Small) 的参数
    l_a, l_b, l_c, l_d = 0.6, 0.8, 1.0, 1.0  # L (Large) 的参数

    # 计算隶属度
    s_membership = trapezoidal(x, s_a, s_b, s_c, s_d)
    l_membership = trapezoidal(x, l_a, l_b, l_c, l_d)

    # 绘制隶属函数
    plt.figure(figsize=(10, 6))
    plt.plot(x, s_membership, '#1E90FF', linewidth=4, label='S (Small)')
    plt.plot(x, l_membership, 'tomato', linewidth=4, label='L (Large)')
    # plt.xlabel('Prediction Mode Indicator', fontsize=14)
    # plt.ylabel('Degree of Membership', fontsize=14)
    # plt.legend()
    plt.grid(False)
    plt.xticks(np.arange(0, 1.1, 0.1), fontsize=18)  
    plt.yticks(np.arange(0, 1.1, 0.1), fontsize=18)  
    # plt.ylim(0, 1.1) 
    # plt.xlim(0, 1) 
    plt.show()


    # # Linear
    # x = np.linspace(0, 1, 1000)

    # s_start = 0.0 # S (Small) 
    # m_start = 0.5  # M (Medium) 
    # l_start = 1.0 # L (Large) 

    # # 绘制隶属函数
    # plt.figure(figsize=(10, 6))
    # plt.axvline(x=s_start, color='black', linewidth=6)
    # plt.axvline(x=m_start, color='black', linewidth=6)
    # plt.axvline(x=l_start, color='black', linewidth=6)
    # plt.hlines(y=0, xmin=x[0], xmax=x[-1], color='black', linewidth=4)
    # plt.title('Fuzzy Membership Functions')
    # plt.xlabel('x')
    # plt.ylabel('Membership')
    # plt.legend()
    # plt.grid(False)
    # plt.ylim(0, 1)  # 设置 y 轴范围
    # plt.show()


if __name__ == "__main__":
    plot_membership_functions()
