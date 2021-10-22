module SlewingMode

using LinearAlgebra, ReferenceFrameRotations

export Axis, roll, yaw, target_plane_map, get_elevation, get_slew_reference, slew_offset

@enum Axis begin
    roll = 1
    yaw = 3
end

get_angle(u, v) = acos(dot(u, v) / (norm(u) * norm(v)))
function get_elevation(r_sc, gs)
    d = r_sc - gs

    90 - (rad2deg(get_angle(gs, d)))
end

function get_angle(q::Quaternion, axis::Axis)
    if axis == roll
        quat_to_angle(q, :XYZ).a1
    elseif axis == yaw
        quat_to_angle(q, :XYZ).a3
    end
end

function angle2quat(θ, axis::Axis)
    if axis == roll
        angle_to_quat(θ, 0, 0, :XYZ)
    elseif axis == yaw
        angle_to_quat(0, 0, θ, :XYZ)
    end
end

function target_plane_map(r, v, axis::Axis)
    x = v / norm(v)
    y = -cross(r / norm(r), v / norm(v))
    z = -r / norm(r)

    A = [x y z]

    A[1:end, 1:end .!= Int(axis)]'
end

function get_slew_reference(r, v, p, axis::Axis)
    p = p - r
    p /= norm(p)

    A = target_plane_map(r, v, axis)
    p_map = A * p; p_map /= norm(p_map)

    θ = atan(p_map[2], p_map[1])

    if axis == roll
        θ = π/2 - θ
    elseif axis == yaw
        θ = -θ
    end

    θ
end

function slew_offset(θ, θᵣ, axis::Axis, k=0.01, Δθ_max=deg2rad(0.5))
    θₑ = θᵣ - θ
    
    Δθ = k * θₑ
    Δθ = abs(Δθ) > Δθ_max ? sign(Δθ) * Δθ_max : Δθ

    ωₒ = zeros(3)
    ωₒ[Int(axis)] = -Δθ

    qₒ = angle2quat(θ + Δθ, axis)

    (θ + Δθ, qₒ, ωₒ)
end

end