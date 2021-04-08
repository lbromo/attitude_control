module SpacecraftModel
    using LinearAlgebra, DifferentialEquations, StaticArrays, Parameters
    
    export run_simulation

    @with_kw mutable struct Internal_State{T,N} <: DEDataArray{T,N}
        x :: Array{T,N} = zeros(3) # Not used, needs to be there for DEDataArray
        u :: Array{T,N} = zeros(3) # Control torque
        h :: Array{T,N} = zeros(3) # Internal momentum
        d :: Array{T,N} = zeros(3) # Disturbance torque
    
        J :: SMatrix{3,3,Real}
    end
    
    Ω(ω) = [
        0    -ω[1] -ω[2] -ω[3]
        ω[1]  0     ω[3] -ω[2]
        ω[2] -ω[3]  0     ω[1]
        ω[3]  ω[2]  -ω[1] 0
    ]

    q(x) = x[1:4]
    ω(x) = x[5:7]

    ∂q(q, ω) = 1/2 * Ω(ω) * q
    ∂q(x) = ∂q(q(x), ω(x))

    ∂ω(ω, J, u, h, d) = inv(J) * (-cross(ω, J*ω + h) + u + d)
    ∂ω(x, p) = ∂ω(ω(x), p.J, p.u, p.h, p.d)

    function ∂f!(∂x, x, p::Internal_State, t)
        ∂x[:] = [∂q(x); ∂ω(x, p)]
    end

    function run_simulation(x0, J::Array{<:Real,2}, t_end::Real, callback=nothing)
        tspan = (0.0, t_end)
        s = Internal_State(J=J)
        prob = ODEProblem(∂f!, x0, tspan, s, callback=callback)
        solve(prob)
    end

end