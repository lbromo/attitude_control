module DemoController
    using LinearAlgebra, StaticArrays, SatelliteToolbox, ReferenceFrameRotations, Parameters
    
    include("../src/dynamics.jl")
    include("../src/utils.jl")
    using .SpacecraftModel
    using .Utils

    export demo_ctrl, save_func, safe_func_type, ExtraParameters

    const safe_func_type = Utils.default_save_func_types;
    save_func = Utils.default_save_func;

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

        hₑ = p.h_ref - h₀
        u_rw = p.k_rw*hₑ

        # For now we just assume we can produce this torque directly.
        # A detumble alorithm on h_body could be used instead 
        u_mag = p.k_body*h_body

        (u_rw, u_mag)
    end

    function demo_ctrl(int)
        p = get_parameters(int)
        q, ω = get_states(int)

        qᵣ, ωᵣ = LVLH_reference(int)

        qₑ = inv(qᵣ * q)
        ωₑ = ω - ωᵣ
        e = [imag(qₑ); ωₑ]

        u = -p.extra_pars.K * e
        u_rw₀, u_mag = monetum_man(p.h_rw, p.J, p.L, p.extra_pars)

        p.u_rw = pinv(p.L) * u + u_rw₀
        p.h_rw = p.h_rw + p.u_rw * p.Δt
        p.u_mag = u_mag
    end
end