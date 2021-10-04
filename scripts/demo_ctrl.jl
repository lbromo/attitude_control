using DrWatson
@quickactivate "Attitude Control"

using Plots
using LinearAlgebra
using DifferentialEquations, DiffEqCallbacks
using SatelliteToolbox, ReferenceFrameRotations

include("../src/dynamics.jl")
include("../src/demo_controller.jl")

# . due to it begin in the Main.<module> scope and not global scope
using .SpacecraftModel
using .DemoController

tle_str = """
AAUSAT3
1 39087U 13009B   21104.06202311  .00000136  00000-0  61744-4 0  9999
2 39087  98.4313 308.7965 0013354 104.8269 255.4393 14.35917261425956
"""
tle = read_tle_from_string(tle_str)[1];

# Setup parameters
J = [0.2 0.0 0.0; 0.0 0.2 0.0; 0.0 0.0 0.4]
L = pinv([0.5 0.8660 0; 0.5 0 0.8660;  0.5 -0.8660 0; 0.5 0 -0.8660])
orbit = init_orbit_propagator(Val(:J4), tle)

K = 1e-3*[I(3) I(3)]
h_ref=[10; -10; 10; -10]*1e-3
extra = ExtraParameters(K=K, h_ref=h_ref)

pars = Parameters(J=J, L=L, orbit=orbit, extra_pars=extra)

# Random initial conditions
q₀ = randn(4); q₀ = q₀ / norm(q₀);
ω₀ = randn(3); ω₀ = ω₀ / norm(ω₀) * deg2rad(1);
x₀ = [q₀; ω₀]

# Simulation time
t_orb = 24*60*60 / tle.n
sim_n = 5
sim_t = sim_n*t_orb

# Setup callback functions
saved_values = SavedValues(Float64, safe_func_type)
callbacks = CallbackSet(
    PeriodicCallback(demo_ctrl, pars.Δt),
    SavingCallback(save_func, saved_values),
)

# run!
sol = run_simulation(x₀, pars, sim_t, callbacks);

# Perforamnce calculations based on saved values during the simulation
# Extract all the values with map functions (and reduce to colloct some of the in matrices)
q =  map(v -> v[1], saved_values.saveval)
qᵣ = map(v -> v[2], saved_values.saveval)

# mapreduce for takes a really long time for some reason. To it in two steps
u = map(v -> v[3], saved_values.saveval) # Get the u vector
u = reduce(hcat, u)                      # Collect into a matrix
h = map(v -> v[4], saved_values.saveval) # Get the h vector
h = reduce(hcat, h)                      # Collect into a matrix

u_mag = map(v -> v[5], saved_values.saveval)
u_mag = reduce(hcat, u_mag)  

qₑ = map((q, qᵣ) -> qᵣ * q, q, qᵣ);

ang = map(rot -> [rot.a1, rot.a2, rot.a3],
    map(q -> quat_to_angle(q, :XYZ), qₑ)
)
θ = map(a -> rad2deg(a[1]), ang)
ϕ = map(a -> rad2deg(a[2]), ang)
ψ = map(a -> rad2deg(a[3]), ang);

# plots all the stuff
saved_t = saved_values.t / t_orb
ode_t = sol.t / t_orb

ape = plot(saved_t , [θ ϕ ψ],
    title="APE",
    label=["θ" "ϕ" "ψ"],
    ylabel="Angle [deg]",
    xlims=(0, sim_n),
    ylims=(-10,10),
)

u_rw_plot = plot(saved_t , 1e3*u',
    title="Control Input (RWs)",
    label=["u ₁" "u ₂" "u ₃" "u ₄"],
    ylabel="Torque [mNm]",
    xlims=(0, sim_n),
    ylims=(),
)

u_mag_plot = plot(saved_t , 1e3*u_mag',
    title="Control Input (mag)",
    label=["u ₁" "u ₂" "u ₃"],
    ylabel="Torque [mNm]",
    xlims=(0, sim_n),
    ylims=(),
)

h_plot = plot(saved_t , 1e3*h',
    title="Momentum buildup (RW)",
    label=["h ₁" "h ₂" "h ₃" "h ₄"],
    ylabel="Momentum [mNms]",
    xlims=(0, sim_n),
    ylims=(),
)

q_plot = plot(sol.t / t_orb, sol[1:4, :]', 
    title="Quaternion",
    #ylabel="Value [⋅]",
    xlims=(0, sol.t[end] / t_orb),
    ylims=(-1,1),
    label=["q ᵣ" "q ᵢ" "q ⱼ" "q ₖ"]
)

ω_plot = plot(sol.t / t_orb, rad2deg.(sol[5:7, :]'), 
    title="Angular rate [deg/s]",
    #ylabel="[deg/s]",
    xlims=(0, sol.t[end] / t_orb),
    ylims=(-5,5),
    label=["ω ₀" "ω ₁" "ω ₂"]
)

plot(ape, h_plot, u_rw_plot, q_plot, u_mag_plot, ω_plot,
    size=(1600,900),
    layout=@layout[a; b; c d; e f]
)


