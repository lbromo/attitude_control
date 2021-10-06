module Utils
    export LVLH_reference, get_B_field_ECI, get_B_field_body
    export default_save_func, default_save_func_types

    using Dates
    using DayCounts
    using StaticArrays
    using LinearAlgebra
    using SatelliteToolbox, ReferenceFrameRotations

    include("dynamics.jl")
    using .SpacecraftModel
    
    const default_save_func_types = Tuple{Quaternion{Float64}, Quaternion{Float64}, SVector{4, Float64}, SVector{4, Float64}, SVector{3, Float64}}
    
    function default_save_func(x, t, int)
        q = Quaternion(x[1:4])
        p = get_parameters(int)
    
        (q, p.qᵣ, p.u_rw, p.h_rw, p.u_mag)
    end

    function get_q_lvlh(r, v)
        r = r / norm(r)
        v = v / norm(v)

        x =  v           # Velocity vector
        y = -cross(r, v) # negative momentum vector
        z = -r           # Nadir vector

        A_lvlh = DCM([
            x y z
        ])

        dcm_to_quat(A_lvlh)
    end

    function LVLH_reference(orb::OrbitPropagatorSGP4{Float64}, t::Float64)
        r, v = propagate!(orb, t)
        
        qᵣ = get_q_lvlh(r, v)
        ωᵣ = [0; -deg2rad(orb.sgp4d.n_0); 0]

        (qᵣ, ωᵣ)
    end

    function LVLH_reference(int)
        p = get_parameters(int)
        
        qᵣ, ωᵣ = LVLH_reference(p.orbit, get_time(int))
        
        #Hanlde q sign
        if(p.qᵣ' * qᵣ < 0)
            qᵣ = -qᵣ
        end
        p.qᵣ = qᵣ # Save for sign check

        (qᵣ, ωᵣ)
    end

    function get_B_field_ECI(int)
        p = get_parameters(int)

        #if(!isnothing(p.orbit))
        #    return [0.0; 0.0; 0.0]
        #end
    
        if typeof(p.orbit) == OrbitPropagatorSGP4{Float64}
            jd = p.orbit.sgp4d.epoch + get_time(int) * 1/(24*60*60)
        else
            jd = p.orbit.orb.t + get_time(int) * 1/(24*60*60)
        end
    
        r, _ = propagate!(p.orbit, jd)
        R = SatelliteToolbox.rECItoECEF(J2000(), PEF(), jd)
        
        r_ecef = R * r

        (lat, lon, alt) = ECEFtoGeodetic(r_ecef)
        
        date_tuple = JDtoDate(jd)
        date = Date(date_tuple[1:3]...)
        start_date = Date(date_tuple[1], 1, 1)
        year = date_tuple[1] + yearfrac(start_date, date, DayCounts.Actual365Fixed())

        igrf(year, alt, lat, lon,  Val(:geodetic))
    end

    function get_B_field_body(int)
        q, _ = get_states(int)
        B = get_B_field_ECI(int)
        B = imag(inv(q) * B * q)
    end
end