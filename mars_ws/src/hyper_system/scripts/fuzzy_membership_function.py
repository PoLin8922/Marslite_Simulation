import numpy as np
import matplotlib.pyplot as plt

def gaussian(x, mean, std_dev):
    return np.exp(-((x - mean) ** 2) / (2 * std_dev ** 2))

def triangular(x, a, b, c):
    return np.maximum(0, np.minimum((x - a) / (b - a), (c - x) / (c - b)))

def plot_membership_functions():
    # # Gausion
    # x = np.linspace(0, 1, 1000)  

    # vl_mean, vl_std_dev = 0.0, 0.1
    # l_mean, l_std_dev = 0.25, 0.1
    # m_mean, m_std_dev = 0.5, 0.1
    # h_mean, h_std_dev = 0.75, 0.1
    # vh_mean, vh_std_dev = 1.0, 0.1

    # vl_membership = gaussian(x, vl_mean, vl_std_dev)
    # l_membership = gaussian(x, l_mean, l_std_dev)
    # m_membership = gaussian(x, m_mean, m_std_dev)
    # h_membership = gaussian(x, h_mean, h_std_dev)
    # vh_membership = gaussian(x, vh_mean, vh_std_dev)

    # plt.figure(figsize=(10, 6))
    # plt.plot(x, vl_membership, 'k-', linewidth=4)
    # plt.plot(x, l_membership, 'k-', linewidth=4)
    # plt.plot(x, m_membership, 'k-', linewidth=4)
    # plt.plot(x, h_membership, 'k-', linewidth=4)
    # plt.plot(x, vh_membership, 'k-', linewidth=4)
    # plt.title('Fuzzy Membership Functions')
    # plt.xlabel('x')
    # plt.ylabel('Membership')
    # plt.legend()
    # plt.grid(False)
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
    # plt.plot(x, s_membership, 'k-', linewidth=4)
    # plt.plot(x, l_membership, 'k-', linewidth=4)
    # plt.hlines(y=0, xmin=x[0], xmax=x[-1], color='black', linewidth=4)
    # plt.title('Fuzzy Membership Functions')
    # plt.xlabel('x')
    # plt.ylabel('Membership')
    # plt.legend()
    # plt.grid(False)
    # plt.show()

    # Linear
    x = np.linspace(0, 1, 1000)

    s_start = 0.0 # S (Small) 
    m_start = 0.5  # M (Medium) 
    l_start = 1.0 # L (Large) 

    # 绘制隶属函数
    plt.figure(figsize=(10, 6))
    plt.axvline(x=s_start, color='black', linewidth=6)
    plt.axvline(x=m_start, color='black', linewidth=6)
    plt.axvline(x=l_start, color='black', linewidth=6)
    plt.hlines(y=0, xmin=x[0], xmax=x[-1], color='black', linewidth=4)
    plt.title('Fuzzy Membership Functions')
    plt.xlabel('x')
    plt.ylabel('Membership')
    plt.legend()
    plt.grid(False)
    plt.ylim(0, 1)  # 设置 y 轴范围
    plt.show()


if __name__ == "__main__":
    plot_membership_functions()
