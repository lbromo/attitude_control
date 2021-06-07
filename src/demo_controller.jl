module DemoController
    using LinearAlgebra, StaticArrays, SatelliteToolbox, ReferenceFrameRotations, Parameters
    
    include("../src/dynamics.jl")
    using .SpacecraftModel

    export demo_ctrl, save_func, safe_func_type, ExtraParameters

    @with_kw mutable struct ExtraParameters
        # State feedback
        K::SMatrix{3,6,Real} = 500e-6*[I(3) I(3)]
        
        # Momentum man
        k_rw::Float64 = 1e-3
        k_body::Float64 = 100e-6
        h_ref::SVector{4,Real} = zeros(4)
    end

    function monetum_man(h, J, L, p)
        h_body = L*h;
        h₀ = h - pinv(L)*h_body

        hₑ = p.h_ref- h₀
        u_rw = p.k_rw*hₑ

        # For now we just assume we can produce this torque directly.
        # A detumble alorithm on h_body could be used instead 
        u_mag = p.k_body*h_body

        (u_rw, u_mag)
    end

    function demo_ctrl(int)
        p = get_parameters(int)
        q, ω = get_states(int)

        qᵣ = LVLH_reference(int)
        ωᵣ = zeros(3)

        qₑ = inv(qᵣ * q)
        ωₑ = ω - ωᵣ
        e = [imag(qₑ); ωₑ]

        u = -p.extra_pars.K * e
        u_rw₀, u_mag = monetum_man(p.h_rw, p.J, p.L, p.extra_pars)

        p.u_rw = pinv(p.L) * u + u_rw₀
        p.h_rw = p.h_rw + p.u_rw * p.Δt
        p.u_mag = u_mag
    end

    function save_func(x, t, int)
        q = Quaternion(x[1:4])
        p = get_parameters(int)
    
        (q, p.qᵣ, p.u_rw, p.h_rw, p.u_mag)
    end

    safe_func_type = Tuple{Quaternion{Float64}, Quaternion{Float64}, SVector{4, Float64}, SVector{4, Float64}, SVector{3, Float64}}


    ## Private stuff
    function get_q_lvlh(r, v)
        r = r / norm(r)
        v = v / norm(v)

        x =  v           # Velocity vector
        y = -cross(r, v) # negative momentum vector
        z = -r           # Nadir vector

        A_lvlh = DCM([
            x y z
        ])

        q = dcm_to_quat(A_lvlh)
        end

    function LVLH_reference(int)
        p = get_parameters(int)
        _, r, v = propagate!(p.orbit, SpacecraftModel.get_time(int))

        q = get_q_lvlh(r, v)

        #Hanlde q sign
        if(p.qᵣ' * q < 0)
            q = -q
        end

        p.qᵣ = q
    end
end