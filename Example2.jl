using OptimalControl
using NLPModelsIpopt
using Plots
using Plots.PlotMeasures
using LaTeXStrings
using Roots
using MadNLP

# regularizaiton parameter for |.|
m = 600
function abs_reg(x)
    return x * tanh(m * x)
end
g(x) = abs_reg(x)

# regularizaiton parameter for interfaces
function max_reg(x, y, k)
    return (1 / k) * log(0.5 * g((exp(k * x) + exp(k * y))))
end


function φ1(x, y, k)
    return max_reg(-x, -y - 0.5, k) + 1 / k
end

function φ2(x, y, k)
    return max_reg(-x, y + 0.5, k) + 1 / k
end

function φ3(x, y, k)
    return x + 1 / (2 * k)
end

# Parameters
k = 150  # Regularization parameter
x = range(-0.5, 0.5, length=500)
y = range(-1, 0.0, length=500)  # Corrected range

# Compute level sets
Z1 = [φ1(a, b, k) for a in x, b in y]
Z2 = [φ2(a, b, k) for a in x, b in y]
Z3 = [φ3(a, b, k) for a in x, b in y]

# Plot contour lines
contour(x, y, Z1', levels=[0], linecolor=:red, label="φ1=0", aspect_ratio=:equal, xlabel="x", ylabel="y")
contour!(x, y, Z2', levels=[0], linecolor=:blue, label="φ2=0")
contour!(x, y, Z3', levels=[0], linecolor=:green, label="φ3=0")


# OCP parameters
tf = 2
umax = 1

# Batch resolution
function batch_ocp(εk, k)
    return @def begin
        t ∈ [0.0, tf], time
        q = [x1, x2] ∈ R^2, state
        ω = [u, v1, v2] ∈ R^3, control

        x1(0) == 1
        x2(0) == 0
    
        -umax ≤ u(t) ≤ umax
    
        0 ≤ v1(t) ≤ 1
        0 ≤ v2(t) ≤ 1
        v1(t) + v2(t) ≤ 1
    
        -4 ≤ x1(t) ≤ 2
        -5 ≤ x2(t) ≤ 1
    
    
        q̇(t) == [v1(t) * (-2 + u(t)) + v2(t) * (u(t)) - (1 - v1(t) - v2(t)) * (x2(t) + 0.5)^2,
                 v1(t) * (x1(t) - 1) + v2(t) * (-10)   + (1 - v1(t) - v2(t)) * (-2 - u(t))]
    
        ∫((x2(t))^2
          +
          (1 / εk) * ((g(φ1(x1(t), x2(t), k)) - (1 - 2 * v1(t)) * φ1(x1(t), x2(t), k))^2 +
                      (g(φ2(x1(t), x2(t), k)) - (1 - 2 * v2(t)) * φ2(x1(t), x2(t), k))^2 +
                      (g(φ3(x1(t), x2(t), k)) - (1 - 2 * (1 - v1(t) - v2(t))) * φ3(x1(t), x2(t), k))^2
        )) → min
    end
end


trajectory_plot = plot(xlabel=L"x_1", ylabel=L"x_2")
control_u_plot = plot(xlabel=L"t", ylabel=L"u")
control_v1_plot = plot(xlabel=L"t", ylabel=L"v_1")
control_v2_plot = plot(xlabel=L"t", ylabel=L"v_2")
control_v3_plot = plot(xlabel=L"t", ylabel=L"v_3")


colors = [:lightblue, :blue, :darkblue, :midnightblue]

last_y1 = nothing
last_y2 = nothing

for (i, k) in enumerate([10, 20, 50, 150])

    OCP = batch_ocp(0.001, k)

    # Solve OCP
    sol_1 = solve(OCP, :madnlp; grid_size=50, display=true)
    sol_2 = solve(OCP, :madnlp; init=sol_1, grid_size=100, display=true)
    sol_3 = solve(OCP, :madnlp; init=sol_2, grid_size=200, display=true)
    sol_4 = solve(OCP, :ipopt; init=sol_3, grid_size=300, display=true)
    sol = solve(OCP, :ipopt; init=sol_4, grid_size=1000, display=true)

    y1(t) = sol.state(t)[1]
    y2(t) = sol.state(t)[2]
    u(t) = sol.control(t)[1]
    v1(t) = sol.control(t)[2]
    v2(t) = sol.control(t)[3]
    v3(t) = 1 - v1(t) - v2(t)

    plot!(trajectory_plot, y1, y2, 0, tf, label=L"$k=$" * "$k", color=colors[i], lw=1, grid=false)
    plot!(control_u_plot, u, 0, tf, label=L"$k=$" * "$k", color=colors[i], lw=1, grid=false)
    plot!(control_v1_plot, v1, 0, tf, label=L"$k=$" * "$k", color=colors[i], lw=1, grid=false)
    plot!(control_v2_plot, v2, 0, tf, label=L"$k=$" * "$k", color=colors[i], lw=1, grid=false)
    plot!(control_v3_plot, v3, 0, tf, label=L"$k=$" * "$k", color=colors[i], lw=1, grid=false)

    last_y1 = y1
    last_y2 = y2
end

plot!(trajectory_plot, [0, 0], [0.3, -1.7], label=false, color="black", lw=1, linestyle=:dash)
plot!(trajectory_plot, [0, 1], [-0.5, -0.5], label=false, color="black", lw=1, linestyle=:dash)


display(trajectory_plot)
display(control_u_plot)
display(control_v1_plot)
display(control_v2_plot)
display(control_v3_plot)
