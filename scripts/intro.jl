using DrWatson
@quickactivate "Attitude Control"

using Plots
using LinearAlgebra
using DiffEqCallbacks

include("../src/dynamics.jl")

function state_feedback(int)
    # Disregard the real part of the quaternion
    x = int.u[2:7]
    
    # Negative state feedback
    K = -1e-3*[I(3) I(3)] 
    
    #Control output -- assuming some actuator without momentum buildup
    int.p.u = K*x
end

q0 = randn(4); q0 = q0 / norm(q0);
ω0 = randn(3); ω0 = ω0 / norm(ω0) * deg2rad(1);

J = [0.2 0.0 0.0; 0.0 0.2 0.0; 0.0 0.0 0.4]
x0 = [q0; ω0]
sol = SpacecraftModel.run_simulation(x0, J, 1500, PeriodicCallback(state_feedback, 1));

params = Dict(
    "q₁" => x0[1],
    "q₂" => x0[2],
    "q₃" => x0[3],
    "q₄" => x0[4],
    
    "ω₀" => x0[5],
    "ω₁" => x0[6],
    "ω₂" => x0[7],
    
    "sol" => sol,
)

path = datadir("simulations", savename(params, "jld2"))
@show path
@tagsave(path, params, storepatch=false)