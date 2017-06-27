type LeadFollowRelationships
    index_fore::Vector{Int}
    index_rear::Vector{Int}
end

function Base.:(==)(A::LeadFollowRelationships, B::LeadFollowRelationships)
    return A.index_fore == B.index_fore &&
           A.index_rear == B.index_rear
end

function LeadFollowRelationships(scene::EntityFrame{PosSpeed1D, BoundingBoxDef, Int}, roadway::Union{Straight1DRoadway, Curve, Wraparound{Straight1DRoadway}, Wraparound{Curve}}, vehicle_indices::AbstractVector{Int} = 1:length(scene))

    nvehicles = length(scene)
    index_fore = zeros(Int, nvehicles)
    index_rear = zeros(Int, nvehicles)

    for vehicle_index in vehicle_indices
        index_fore[vehicle_index] = get_neighbor_fore(scene, vehicle_index, roadway).ind
        index_rear[vehicle_index] = get_neighbor_rear(scene, vehicle_index, roadway).ind
    end

    return LeadFollowRelationships(index_fore, index_rear)
end
function LeadFollowRelationships(scene::EntityFrame{VehicleState, BoundingBoxDef, Int}, roadway::Roadway, vehicle_indices::AbstractVector{Int} = 1:length(scene))

    nvehicles = length(scene)
    index_fore = zeros(Int, nvehicles)
    index_rear = zeros(Int, nvehicles)

    F = VehicleTargetPointFront()
    R = VehicleTargetPointRear()

    for vehicle_index in vehicle_indices
        index_fore[vehicle_index] = get_neighbor_fore_along_lane(scene, vehicle_index, roadway, F, R, F).ind
        index_rear[vehicle_index] = get_neighbor_rear_along_lane(scene, vehicle_index, roadway, R, F, R).ind
    end

    return LeadFollowRelationships(index_fore, index_rear)
end

####

immutable StateBounds
    Δlo::Float64
    Δhi::Float64
end
domain_size(bounds::StateBounds) = bounds.Δhi - bounds.Δlo
Distributions.Uniform(bounds::StateBounds) = Uniform(bounds.Δlo, bounds.Δhi)

###

# Variable instance for a particular scene
immutable Vars
    values::Vector{Float64}
    bounds::Vector{StateBounds}
    symbols::Vector{Symbol} # symbol associated with each variable; for convenience [:v, :ϕ, ...]
    vehicle_indices::Vector{Int} # index of the vehicle in the scene that each variable corresponds to
end
Base.length(vars::Vars) = length(vars.values)
function Base.findfirst(vars::Vars, vehicle_index::Int, sym::Symbol)
    for i in 1 : length(vars)
        if vars.vehicle_indices[i] == vehicle_index &&
           vars.symbols[i] == sym

           return i
        end
    end
    return 0
end

####

# const BOUNDS_ϕ = (deg2rad(-30),deg2rad(30))
# function get_state_bounds_s(scene::Scene, roadway::Roadway, lead_follow::LeadFollowRelationships, vehicle_index::Int)

#     veh_rear = scene[vehicle_index]
#     veh_fore = scene[lead_follow.index_fore[vehicle_index]]
#     relpos = get_frenet_relative_position(veh_fore, veh_rear, roadway)
#     Δs_fore = relpos.Δs - veh_rear.def.length/2 -  veh_fore.def.length/2
#     @assert !isnan(Δs_fore)

#     veh_rear = scene[lead_follow.index_rear[vehicle_index]]
#     veh_fore = scene[vehicle_index]
#     relpos = get_frenet_relative_position(veh_fore, veh_rear, roadway)
#     Δs_rear = relpos.Δs - veh_rear.def.length/2 -  veh_fore.def.length/2
#     @assert !isnan(Δs_rear)

#     return StateBounds(-Δs_rear, Δs_fore)
# end
# function get_state_bounds_t(scene::Scene, roadway::Roadway, lead_follow::LeadFollowRelationships, vehicle_index::Int)
#     d_left = get_markerdist_left(scene[vehicle_index], roadway)
#     d_right = get_markerdist_right(scene[vehicle_index], roadway)
#     return StateBounds(-d_right, d_left)
# end
# function get_state_bounds_v(scene::Scene, roadway::Roadway, lead_follow::LeadFollowRelationships, vehicle_index::Int)
#     v = scene[vehicle_index].state.v
#     return StateBounds(BOUNDS_V[1]-v, BOUNDS_V[2]-v)
# end
# function get_state_bounds_ϕ(scene::Scene, roadway::Roadway, lead_follow::LeadFollowRelationships, vehicle_index::Int)
#     ϕ = scene[vehicle_index].state.posF.ϕ
#     return StateBounds(BOUNDS_ϕ[1]-ϕ, BOUNDS_ϕ[2]-ϕ)
# end
