module SpacecraftModel
    using LinearAlgebra, DifferentialEquations, StaticArrays, Parameters, SatelliteToolbox
    
    export Parameters, run_simulation, get_states, get_parameters, get_time

    ## Helper functions 
    get_states(int) = (Quaternion(int.u[1:4]), int.u[5:7]); 
    get_parameters(int) = int.p
    get_time(int) = int.t

    # Configuration parameters
    @with_kw mutable struct Parameters
        J::SMatrix{3,3,Real}
        L::SMatrix{3,4,Real}
        Δt::Float64 = 1.0
        orbit::Union{OrbitPropagator, Nothing} = nothing
        
        u_rw::SVector{4,Real} = zeros(4)
        h_rw::SVector{4,Real} = zeros(4)
        
        u_mag::SVector{3,Real} = zeros(3)
        u_dist::SVector{3,Real} = zeros(3)
    
        qᵣ::Quaternion{Float64} = Quaternion([1.0; 0.0; 0.0; 0.0])
        ωᵣ::SVector{3,Real} = zeros(3)

        extra_pars::Any = nothing
    end
    
    # Helper function for dynamics
    q(x) = x[1:4]; 
    ω(x) = x[5:7]
    ∂q(x) = ∂q(q(x), ω(x)); 
    ∂ω(x, p) = ∂ω(ω(x), p.J, p.L * p.u_rw + p.u_mag, p.L * p.h_rw, p.u_dist)
    
    # Dynamic model
    Ω(ω) = [
        0    -ω[1] -ω[2] -ω[3]
        ω[1]  0     ω[3] -ω[2]
        ω[2] -ω[3]  0     ω[1]
        ω[3]  ω[2]  -ω[1] 0
    ]
    
    ∂q(q, ω) = 1/2 * Ω(ω) * q
    ∂ω(ω, J, u, h, d) = inv(J) * (-cross(ω, J*ω + h) + u + d)

    function ∂f!(∂x, x, p::Parameters, t)
        ∂x[:] = [∂q(x); ∂ω(x, p)]
    end

    function run_simulation(x0, pars::Parameters, t_end::Real, callback_set=nothing)
        tspan = (0.0, t_end)
        #s = Internal_State(pars=pars)
        prob = ODEProblem(∂f!, x0, tspan, pars, callback=callback_set)
        solve(prob)
    end

end