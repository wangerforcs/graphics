zyx欧拉角的旋转矩阵为$R_z*R_y*R_x$，课件里和认知里是这样，但在课程提供的框架代码里要用$R_x*R_y*R_z$(测试发现的)，不知道原因。

万向锁：
绕第二个旋转轴旋转90度时，第三个旋转轴和第一个旋转轴重合，此时绕第一个旋转轴旋转和绕第二个旋转轴旋转效果一样，失去了一个自由度，这个缺陷是由将一个旋转分解为绕三个轴的旋转带来的，导致某些情况下使用三个旋转合成的旋转和预料的姿态转变不一样。网上很多文章感觉都在乱回答，没有找到重点。
四元数转旋转矩阵
四元数可由旋转轴$n$(过原点)和旋转角$\theta$表示，以此表示的四元数为
$$
q=\begin{bmatrix}
q_1\\
q_2\\
q_3\\
q_4\\
\end{bmatrix}=\begin{bmatrix}
n_x\sin\frac{\theta}{2}\\
n_y\sin\frac{\theta}{2}\\
n_z\sin\frac{\theta}{2}\\
\cos\frac{\theta}{2}\\
\end{bmatrix}
$$
对应的旋转矩阵为
$$
cos\theta I+(1-cos\theta)nn^T+sin\theta\begin{bmatrix}
0&-n_z&n_y\\
n_z&0&-n_x\\
-n_y&n_x&0\\
\end{bmatrix}
$$

模型变换
视角变换
$$
T_A^B=\begin{bmatrix}
(R_B^A)^T&-(R_B^A)^TP_B^A\\
0&1\\
\end{bmatrix}
$$
此处$R_B^A=[up x eye,up,eye]$，$P_B^A=center$
主要关注方向，暂时不考虑三个向量的模，实际计算normalize即可。
这eye,up,center确定了相机坐标系，利用逆变换求出了从世界坐标到相机坐标的变换矩阵。
投影变换







射线和AABB包围盒的相交检测，
[参考](https://zhuanlan.zhihu.com/p/438812301)
[参考](https://zhuanlan.zhihu.com/p/610258258)