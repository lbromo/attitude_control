module SMC
    using CBinding
    using StaticArrays
    using LinearAlgebra
    using SatelliteToolbox

    include("utils.jl")
    include("dynamics.jl")
    using .Utils
    using .SpacecraftModel

    export conf
    export set_inertia, set_default_ctrl_conf, set_wheel_matrix, smc

    const c"int8_t"  = Int8
    const c"uint8_t"  = UInt8

    const c"int16_t"  = Int16
    const c"uint16_t"  = UInt16

    const c"int32_t"  = Int32
    const c"uint32_t" = UInt32

    const c"int64_t"  = Int64
    const c"uint64_t" = UInt64;

    LD_PATH = [
    "../build/",
    ]

    INC_PATH = [
        "../lib/libutil/include/",
        "../lib/libadcs/include/",
        "../lib/libadcs/lib/libadcs_client/include",
    ]

    LD_PATH = map(p -> "-L"*p, LD_PATH)
    INC_PATH = map(i -> "-I"*i, INC_PATH)
    LIB = "shadcs";

    ## C environment
    c`$(INC_PATH) $(LD_PATH) -l$(LIB)`
    # Setup includes

    # Don't konw why these are needed, but for some reason, they are :S
    c"""
    #include <gs/util/types.h>
    #include <gs/adcs/adcs_types.h>

    #include <gs/adcs/param/host/sensor_css.h>
    #include <gs/adcs/param/host/sensor_common.h>
    #include <gs/adcs/param/host/sensor_fss.h>
    #include <gs/adcs/param/host/sensor_extgyro.h>
    #include <gs/adcs/param/host/sensor_horizon.h>
    #include <gs/adcs/param/host/sensor_extmag.h>
    #include <gs/adcs/param/host/sensor_startracker.h>
    #include <gs/adcs/param/host/sensor_a3200.h>

    #include <gs/adcs/param/host/act_magnetorquer.h>
    #include <gs/adcs/param/host/act_rw.h>

    #include <gs/adcs/param/host/gnc_common.h>
    #include <gs/adcs/param/host/gnc_ads.h>
    #include <gs/adcs/param/host/gnc_ctrl_pointing.h>
    #include <gs/adcs/param/host/gnc_ctrl_spin.h>
    """

    c"""
    #include <gs/adcs/adcs_param_types.h>
    #include <gs/adcs/adcs_telem_types.h>

    #include <gs/adcs/acs/controller/gs_adcs_wheel.h>
    """;


    gnc = Libc.malloc(c"gs_adcs_config_gnc_t"())

    periph = Libc.malloc(c"gs_adcs_config_periph_t"())
    fss = Libc.malloc(c"gs_adcs_sensor_fss_memory_t"())

    conf = Libc.malloc(c"gs_adcs_config_t"(
            gnc=gnc, periph=periph
        )
    )

    ephem = Libc.malloc(c"gs_adcs_ephem_t"())

    state_est = Libc.malloc(c"GS_ADCS_UKF_Data_t"())

    act_data = Libc.malloc(c"gs_adcs_actuatordata_t"())
    ctrl = Libc.malloc(c"gs_adcs_ctrl_t"())

    _smc = c"gs_adcs_wheel_ctrl"[]

    function set_inertia(J)
        conf.gnc.common.inertia[1] = J[1,1]
        conf.gnc.common.inertia[2] = J[2,2]
        conf.gnc.common.inertia[3] = J[3,3]

        return nothing
    end

    function set_wheel_matrix(L)
        for i in 1:length(L)
            conf.periph.act.mw.layout[i] = L'[i]
            conf.periph.act.mw.inv_layout[i] = pinv(L)[i]
        end
    end

    function set_default_ctrl_conf()
        conf.gnc.ctrl.mw_smc[1]  =  0.2000
        conf.gnc.ctrl.mw_smc[2]  = -0.0001
        conf.gnc.ctrl.mw_smc[3]  =  0.0010
        conf.gnc.ctrl.mw_gain[1] = -0.0005

        conf.gnc.ctrl.mw_dumpgain[]  = -0.01
        conf.gnc.ctrl.mw_mw2trq[]    = -1.0
        conf.gnc.ctrl.mw_mangain[]   = -0.005
        conf.gnc.ctrl.mw_manoffset[] =  0.010

        conf.gnc.ctrl.mw_mannull[1] = -1
        conf.gnc.ctrl.mw_mannull[2] =  1
        conf.gnc.ctrl.mw_mannull[3] = -1
        conf.gnc.ctrl.mw_mannull[4] =  1

        conf.gnc.ctrl.en_mwman[]  = true
        conf.gnc.ctrl.en_mwdump[] = true

        conf.periph.act.torquer.am[1] = 1
        conf.periph.act.torquer.am[2] = 1
        conf.periph.act.torquer.am[3] = 1;

        return nothing
    end

    function smc(int, qᵣ, ωᵣ)
        p = get_parameters(int)
        q, ω = get_states(int)
        B = get_B_field_ECI(int)

        smc(q, ω, qᵣ, ωᵣ, B, p.h_rw)
    end

    function smc(q, ω, qᵣ, ωᵣ, B, h_rw)
        for i in 1:3
            ephem.magECI[i] = B[i]
        end
        
        q_imag = imag(q)
        state_est.EstimatedState[1] = q_imag[1]
        state_est.EstimatedState[2] = q_imag[2]
        state_est.EstimatedState[3] = q_imag[3]
        state_est.EstimatedState[4] = real(q)

        state_est.EstimatedState[5] = ω[1]
        state_est.EstimatedState[6] = ω[2]
        state_est.EstimatedState[7] = ω[3]

        qᵣ = inv(qᵣ)
        qᵣ_imag = imag(qᵣ)
        
        ctrl.ref_q.q1[] = qᵣ_imag[1]
        ctrl.ref_q.q2[] = qᵣ_imag[2]
        ctrl.ref_q.q3[] = qᵣ_imag[3]
        ctrl.ref_q.q4[] = real(qᵣ)

        ctrl.ref_rate[1] = ωᵣ[1]
        ctrl.ref_rate[2] = ωᵣ[2]
        ctrl.ref_rate[3] = ωᵣ[3]

        for i in 1:4
            act_data.wheelMomentum[i] = h_rw[i]
        end
        
        _smc(conf, ephem, state_est, act_data, ctrl)
            
        u_rw = SA[ctrl.mw_torque[][1], ctrl.mw_torque[][2], ctrl.mw_torque[][3], ctrl.mw_torque[][4]]
        
        for i ∈ 1:3
            if(abs(ctrl.M[][i]) > 100)
                ctrl.M[i] = sign(ctrl.M[][i]) * 100
            end
            ctrl.M[i] = ctrl.M[][i] * periph.act.torquer.am[][i] / 100
        end
        u_mag = SA[ctrl.M[][1], ctrl.M[][2], ctrl.M[][3]]
        
        (u_rw, u_mag)
    end
end