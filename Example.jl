using OptimalControl
using NLPModelsIpopt
using Plots
using Plots.PlotMeasures
using LaTeXStrings
using Roots


# regularizaiton parameter for |.|
m    = 600
function abs_reg(x)
    return x*tanh(m*x)
end
g(x) = abs_reg(x)


# regularizaiton parameter for interfaces
function max_reg(x,y,k)
    num = x * exp(k * x) + y * exp(k * y)
    denom = exp(k * x) + exp(k * y)
    return num / denom
end

function φ1(x,y,k)
    return -y
end

function φ2(x, y,k)
    return max_reg(-x,y, k) + 1/(sqrt(k)*k)
end

function φ3(x, y,k)
    return max_reg(x,y,k)  + 1/(sqrt(k)*k)
end




# OCP parameters
tf  = 3.5
a   = 1
b   = 1
umax= 1
k=150


# Batch resolution
function batch_ocp(εk,k)
    return  @def begin
        t  ∈ [ 0., tf],          time
        q  = [ x1, x2]    ∈ R^2, state
        ω  = [u, v1, v2]  ∈ R^3, control
    
        x1(0) == a
        x2(0) == b
        
        -umax ≤ u(t)        ≤ umax
        
            0 ≤ v1(t)       ≤ 1
            0 ≤ v2(t)       ≤ 1
                v1(t)+v2(t) ≤ 1
    
        -1.2 ≤   x1(t) ≤ 1.2     # to accelerate convergence
        -2.1 ≤   x2(t) ≤ 1.2     # to accelerate convergence
    
        q̇(t) == [                     v2(t)*(-2+u(t))  - (1-v1(t)-v2(t))*(x2(t)+0.5)^2 ,
                   v1(t)*(-2-u(t))  + v2(t)*(x1(t)-1)  + (1-v1(t)-v2(t))*(-2+u(t))]
    
        ∫((x2(t))^2
         +(1/εk)*((g(φ1(x1(t), x2(t),k)) - (1-2*v1(t))          *φ1(x1(t), x2(t),k))^2 +
                  (g(φ2(x1(t), x2(t),k)) - (1-2*v2(t))          *φ2(x1(t), x2(t),k))^2 +
                  (g(φ3(x1(t), x2(t),k)) - (1-2*(1-v1(t)-v2(t)))*φ3(x1(t), x2(t),k))^2 
        ))  → min
    end
end


trajectory_plot = plot( xlabel=L"x_1", ylabel=L"x_2")
control_u_plot  = plot( xlabel=L"t", ylabel=L"u")
control_v1_plot = plot( xlabel=L"t", ylabel=L"v_1")
control_v2_plot = plot( xlabel=L"t", ylabel=L"v_2")
control_v3_plot = plot( xlabel=L"t", ylabel=L"v_3")
costate_p1_plot = plot( xlabel=L"t", ylabel=L"p_1")
costate_p2_plot = plot( xlabel=L"t", ylabel=L"p_2")

colors = [:deepskyblue2, :dodgerblue1, :blue, :navyblue]


for (i, εk) in enumerate([0.01, 0.005, 0.001])
    
    OCP = batch_ocp(εk,150)
    
    # Solve OCP
    sol_50 = solve(OCP, init = (state = t -> [-t, 0.1], control = [umax,1,0]), grid_size=50, print_level=4)
    sol_100 = solve(OCP, init = sol_50, grid_size=100, print_level=0)
    sol_200 = solve(OCP, init = sol_100, grid_size=200, print_level=0)
    sol_300 = solve(OCP, init = sol_200, grid_size=300, print_level=0)
    sol = solve(OCP, init = sol_300, grid_size=500, print_level=0)
    
    y1(t)   = sol.state(t)[1]
    y2(t)   = sol.state(t)[2]
    u(t)    = sol.control(t)[1]
    v1(t)   = sol.control(t)[2]
    v2(t)   = sol.control(t)[3]
    v3(t)   = 1 - v1(t) - v2(t)
    qq1(t)  = sol.costate(t)[1]
    qq2(t)  = sol.costate(t)[2]
    
    plot!(trajectory_plot, y1, y2, 0, tf, label=L"$\varepsilon_k=$" * "$(εk)", color = colors[i], lw=1, grid=false)
    plot!(control_u_plot,  u,      0, tf, label=L"$\varepsilon_k=$" * "$(εk)", color=colors[i], lw=1, grid=false)
    plot!(control_v1_plot, v1,     0, tf, label=L"$\varepsilon_k=$" * "$(εk)", color=colors[i], lw=1, grid=false)
    plot!(control_v2_plot, v2,     0, tf, label=L"$\varepsilon_k=$" * "$(εk)", color=colors[i], lw=1, grid=false)
    plot!(control_v3_plot, v3,     0, tf, label=L"$\varepsilon_k=$" * "$(εk)", color=colors[i], lw=1, grid=false)
    plot!(costate_p1_plot, qq1,    0, tf, label=L"$\varepsilon_k=$" * "$(εk)", color=colors[i], lw=1, grid=false)
    plot!(costate_p2_plot, qq2,    0, tf, label=L"$\varepsilon_k=$" * "$(εk)", color=colors[i], lw=1, grid=false)

end

plot!(trajectory_plot, [0, 0], [0, -2], label=false, color="black", lw=1, linestyle=:dash)
hline!(trajectory_plot, [0],            label=false, color="black", lw=1, linestyle=:dash)


display(trajectory_plot)
display(control_u_plot)
display(control_v1_plot)
display(control_v2_plot)
display(control_v3_plot)
display(costate_p1_plot)
display(costate_p2_plot)

savefig(trajectory_plot, "x.pdf")  
savefig(control_u_plot,  "u.pdf")  
savefig(control_v1_plot, "v1.pdf")  
savefig(control_v2_plot, "v2.pdf")  
savefig(control_v3_plot, "v3.pdf")  
savefig(costate_p1_plot, "p1.pdf")  
savefig(costate_p2_plot, "p2.pdf")


# Time spent in boundary layer

k  = 150 # 10, 50, 100, 150
εk = 0.001
OCP_ = batch_ocp(εk,k)

sol_50  = solve(OCP_, init = (state = t -> [-t, 0.1], control = [umax,1,0]), grid_size=50, print_level=4)
sol_100 = solve(OCP_, init = sol_50, grid_size=100, print_level=4)
sol_200 = solve(OCP_, init = sol_100, grid_size=200, print_level=4)
sol_300 = solve(OCP_, init = sol_200, grid_size=300, print_level=4)
sol_500 = solve(OCP_, init = sol_300, grid_size=500, print_level=4)
sol     = solve(OCP_, init = sol_500, grid_size=1000, print_level=4)

plot(sol)
objective(sol)
x1_(t) = sol.state(t)[1]
x2_(t) = sol.state(t)[2]

f1(t) = φ1(x1_(t), x2_(t),k)
f2(t) = φ2(x1_(t), x2_(t),k)
f3(t) = φ3(x1_(t), x2_(t),k)

plot(f1, 0, tf)
plot!(f2, 0 , tf)
plot!(f3, 0 , tf)

t1_ = find_zero(t -> f1(t), (0.0, tf), Bisection())
t2_ = find_zero(t -> f2(t), (0.0, tf/2), Bisection())
t3_ = find_zero(t -> f2(t), (tf/2, tf), Bisection())
t4_ = find_zero(t -> f3(t), (tf/2, tf), Bisection())

time_spent = (t2_-t1_) + (t4_-t3_)




