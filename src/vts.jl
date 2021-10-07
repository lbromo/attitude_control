module VTS

    export generate_position_and_attitude_files_file, generate_position_file, generate_attitude_file
    using SatelliteToolbox

    POS_VTS_HEADER = """
CIC_OEM_VERS    = 2.0
CREATION_DATE   = 2021-10-01T12:00:00
ORIGINATOR      = GOMSPACE

META_START

OBJECT_NAME     = JULIASIM
OBJECT_ID       = JULIASIM

CENTER_NAME     = EARTH
REF_FRAME       = EME2000
TIME_SYSTEM     = UTC

META_STOP
"""

    QUAT_VTS_HEADER = """
CIC_AEM_VERS    = 1.0
CREATION_DATE   = 2021-10-01T12:00:00
ORIGINATOR      = GOMSPACE

META_START

OBJECT_NAME     = JULIASIM
OBJECT_ID       = JULIASIM

REF_FRAME_A     = EME2000
REF_FRAME_B     = SC_BODY_1
ATTITUDE_DIR    = A2B

TIME_SYSTEM     = UTC
ATTITUDE_TYPE   = QUATERNION
QUATERNION_TYPE = LAST

META_STOP
""";

    sec_per_day = (24*60*60)

    function get_days_and_secs(jd)
        days = Int(floor(jd))
        secs = (jd - days) * 24*60*60  
    
        (days, secs)
    end

    function gen_file(header, lines, fname)
        file = join(
            vcat([header], lines),
            "\n"
        )

        open(fname, "w") do io
            write(io, file)
        end
        
        file
    end

    function generate_position_file(jd₀::Float64, p, t::Vector{Float64}, fname="pos.cic")
        lines = String[]
        for (t, p) in zip(t, p)
            jd = jd₀ + t / sec_per_day - 2400000.5
            days, secs = get_days_and_secs(jd)
    
            x,y,z = p * 1e-3
            #dx,dy,dz = v * 1e3
            line = "$days $secs $x $y $z"
    
            push!(lines, line)
        end
        gen_file(POS_VTS_HEADER, lines, fname)
    end

    function generate_attitude_file(jd₀::Float64, q::Vector{Quaternion{Float64}}, t::Vector{Float64}, fname="q.cic")
        lines = String[]
        for (t, q) in zip(t, q)
            jd = jd₀ + t / sec_per_day - 2400000.5
            days, secs = get_days_and_secs(jd)
            
            qr = real(q)
            qi, qj, qk = imag(q)
            
            line = "$days $secs $qi $qj $qk $qr"
            
            push!(lines, line)
        end

        gen_file(QUAT_VTS_HEADER, lines, fname)
    end

    function generate_position_file(orb::OrbitPropagatorSGP4{Float64}, t::Vector{Float64}, fname="pos.cic")
        r, v = propagate!(orb, t);
        jd₀ = orb.sgp4d.epoch

        generate_position_file(jd₀, r, t, fname);
    end

    function generate_position_and_attitude_files_file(orb::OrbitPropagatorSGP4{Float64}, q::Vector{Quaternion{Float64}}, t::Vector{Float64}, pos_name="pos.cic", att_name="q.cic")
        r, v = propagate!(orb, t);
        jd₀ = orb.sgp4d.epoch

        (
            generate_position_file(jd₀, r, t, pos_name),
            generate_attitude_file(jd₀, q, t, att_name)
        )
    end
end