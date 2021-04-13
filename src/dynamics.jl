module SpacecraftModel
    using LinearAlgebra, DifferentialEquations, StaticArrays, Parameters, SatelliteToolbox
    
    export Parameters, run_simulation, get_states, get_parameters
    export get_ctrl_torque, get_ctrl_torque, get_disturbance_torque, set_ctrl_torque, set_ctrl_torque, set_disturbance_torque

    ## Helper functions 
    get_states(int) = (Quaternion(int.u[1:4]), int.u[5:7]); 
    get_parameters(int) = int.p.pars;
    get_ctrl_torque(int) = int.p.u; 
    get_internal_momentum(int) = int.p.h; 
    get_disturbance_torque(int) = int.p.d
    set_ctrl_torque!(int, u) = int.p.u = u; 
    set_internal_momentum!(int, h) = int.p.h = h; 
    set_disturbance_torque!(int, d) = int.p.d = d

    # Configuration parameters
    @with_kw mutable struct Parameters
        J::SMatrix{3,3,Real}
        Δt::Float64 = 1.0
        orb::Union{Orbit, Nothing} = nothing

        qᵣ::Union{Quaternion{Float64}, Nothing} = nothing
        ωᵣ::Union{SVector{3,Real}, Nothing} = nothing
    
        extra_pars :: Any = nothing
    end

    # Internal states and handle to parameters
    @with_kw mutable struct Internal_State{T,N} <: DEDataArray{T,N}
        x::Array{T,N} = zeros(3) # Not used, needs to be there for DEDataArray
        u::Array{T,N} = zeros(3) # Control torque
        h::Array{T,N} = zeros(3) # Internal momentum
        d::Array{T,N} = zeros(3) # Disturbance torque
    
        pars::Parameters
    end
    
    # Helper function for dynamics
    q(x) = x[1:4]; ω(x) = x[5:7]
    ∂q(x) = ∂q(q(x), ω(x)); ∂ω(x, p) = ∂ω(ω(x), p.pars.J, p.u, p.h, p.d)
    
    # Dynamic model
    Ω(ω) = [
        0    -ω[1] -ω[2] -ω[3]
        ω[1]  0     ω[3] -ω[2]
        ω[2] -ω[3]  0     ω[1]
        ω[3]  ω[2]  -ω[1] 0
    ]
    
    ∂q(q, ω) = 1/2 * Ω(ω) * q
    ∂ω(ω, J, u, h, d) = inv(J) * (-cross(ω, J*ω + h) + u + d)

    function ∂f!(∂x, x, p::Internal_State, t)
        ∂x[:] = [∂q(x); ∂ω(x, p)]
    end

    function run_simulation(x0, pars::Parameters, t_end::Real, callback=nothing)
        tspan = (0.0, t_end)
        s = Internal_State(pars=pars)
        prob = ODEProblem(∂f!, x0, tspan, s, callback=callback)
        solve(prob)
    end

end