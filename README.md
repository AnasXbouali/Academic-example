# Regularization of Optimal Control Problems on Stratified Domains Using Additional Controls

* [Academic Example](Example.jl): We provide an academic example illustrating numerically the convergence of approximated optimal solutions (to the penalized problem) to an optimal solution of the academic example.

We consider a dynamics in the plane defined over three strata:

$$
\dot{x} = \begin{cases} 
     g_1(x,u) & x_2 > 0, \\
     g_2(x,u) & x_1 > 0, \; x_2 < 0, \\
     g_3(x,u) & x_1 < 0, \; x_2 < 0,
\end{cases}
$$

where

$$
g_1(x,u) = \begin{pmatrix} 
    0 \\ -2 - u 
\end{pmatrix}, \quad
g_2(x,u) = \begin{pmatrix} 
    -2 + u \\ x_1 - 1 
\end{pmatrix}, \quad
g_3(x,u) = \begin{pmatrix} 
    -\left(x_1 + \frac{1}{2}\right)^2 \\ -2 + u 
\end{pmatrix},
$$

and \( u \) is a control that takes values in \( U = [-1, 1] \).

In this example, the three strata are defined by the signs of the functions \( \varphi_1(x) = x_1 \) and \( \varphi_2(x) = x_2 \). Following Section \ref{sec-partition} and its notations, four sets are associated with these two functions:

$$
\begin{align*}
& X_1 = \{ x \in \mathbb{R}^2 ; \; x_1 < 0, \; x_2 < 0 \}, \\
& X_2 = \{ x \in \mathbb{R}^2 ; \; x_1 < 0, \; x_2 > 0 \}, \\
& X_3 = \{ x \in \mathbb{R}^2 ; \; x_1 > 0, \; x_2 < 0 \}, \\
& X_4 = \{ x \in \mathbb{R}^2 ; \; x_1 > 0, \; x_2 > 0 \},
\end{align*}
$$

which define a stratification of the plane with \( N = 2^2 = 4 \) regions: \( \mathbb{R}^2 = \overline{X}_1 \cup \overline{X}_2 \cup \overline{X}_3 \cup \overline{X}_4 \). We write the dynamics as:

$$
\dot{x} = f(x,u) = \begin{cases} 
    f_1(x,u) = g_3(x,u), & x \in X_1, \\
    f_2(x,u) = g_1(x,u), & x \in X_2, \\
    f_3(x,u) = g_2(x,u), & x \in X_3, \\
    f_4(x,u) = g_1(x,u), & x \in X_4.
\end{cases}
$$
The problem of interest:

$$
\begin{aligned}
    \text{minimize} & \quad \int_0^T x_2^2(t) \, \mathrm{d}t, \\
    \text{subject to} & \quad (x(\cdot), u(\cdot)) \in \mathrm{AC}_{[0, \frac{7}{2}]} \times \mathcal{U}_{[0, \frac{7}{2}]}, \\
    & \quad \dot{x}(t) = f(x(t), u(t)), \quad \text{a.e. } t \in [0, T], \\
    & \quad x(0) = (1,1).
\end{aligned}
$$

where \( T = \frac{7}{2} \) and \( \mathcal{U}_{[0, \frac{7}{2}]} \) denotes the set of time-measurable functions from \( [0, \frac{7}{2}] \) to \( [-1,1] \).
