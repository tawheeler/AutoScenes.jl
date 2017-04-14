type LeadFollowRelationships # needed for bounds
    index_fore::Vector{Int}
    index_rear::Vector{Int}
end

function Base.:(==)(A::LeadFollowRelationships, B::LeadFollowRelationships)
    return A.index_fore == B.index_fore &&
           A.index_rear == B.index_rear
end
function get_active_vehicles(lead_follow::LeadFollowRelationships)
    active_vehicles = Set{Int}()
    for i in 1:length(lead_follow.index_fore)
        if lead_follow.index_fore[i] != 0 && lead_follow.index_rear[i] != 0
            push!(active_vehicles, i)
        end
    end
    return active_vehicles
end

function LeadFollowRelationships(scene::MobiusScene, roadway::StraightRoadway, vehicle_indices::AbstractVector{Int} = 1:length(scene))

    nvehicles = length(scene)
    index_fore = zeros(Int, nvehicles)
    index_rear = zeros(Int, nvehicles)

    for vehicle_index in vehicle_indices
        index_fore[vehicle_index] = get_neighbor_fore(scene, vehicle_index, roadway).ind
        index_rear[vehicle_index] = get_neighbor_rear(scene, vehicle_index, roadway).ind
    end

    return LeadFollowRelationships(index_fore, index_rear)
end
function LeadFollowRelationships(scene::Scene, roadway::Roadway, vehicle_indices::AbstractVector{Int} = 1:length(scene))

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

get_state_bounds_s{S,D,I,R}(scene::EntityFrame{S,D,I}, roadway::R, lead_follow::LeadFollowRelationships, vehicle_index::Int) = StateBounds(NaN,NaN)
get_state_bounds_t{S,D,I,R}(scene::EntityFrame{S,D,I}, roadway::R, lead_follow::LeadFollowRelationships, vehicle_index::Int) = StateBounds(NaN,NaN)
get_state_bounds_v{S,D,I,R}(scene::EntityFrame{S,D,I}, roadway::R, lead_follow::LeadFollowRelationships, vehicle_index::Int) = StateBounds(NaN,NaN)
get_state_bounds_ϕ{S,D,I,R}(scene::EntityFrame{S,D,I}, roadway::R, lead_follow::LeadFollowRelationships, vehicle_index::Int) = StateBounds(NaN,NaN)

function get_state_bounds_s(scene::MobiusScene, roadway::StraightRoadway, lead_follow::LeadFollowRelationships, vehicle_index::Int)

    ego = scene[vehicle_index]
    rear = scene[lead_follow.index_rear[vehicle_index]]
    fore = scene[lead_follow.index_fore[vehicle_index]]

    Δs_fore = get_headway(fore, ego, roadway)
    Δs_rear = get_headway(ego, rear, roadway)

    return StateBounds(-Δs_rear, Δs_fore)
end
function get_state_bounds_v(scene::MobiusScene, roadway::StraightRoadway, lead_follow::LeadFollowRelationships, vehicle_index::Int)
    v = scene[vehicle_index].state.v
    return StateBounds(BOUNDS_V[1]-v, BOUNDS_V[2]-v)
end

const BOUNDS_V = (0.0,32.0)
const BOUNDS_ϕ = (deg2rad(-30),deg2rad(30))
function get_state_bounds_s(scene::Scene, roadway::Roadway, lead_follow::LeadFollowRelationships, vehicle_index::Int)

    veh_rear = scene[vehicle_index]
    veh_fore = scene[lead_follow.index_fore[vehicle_index]]
    relpos = get_frenet_relative_position(veh_fore, veh_rear, roadway)
    Δs_fore = relpos.Δs - veh_rear.def.length/2 -  veh_fore.def.length/2
    @assert !isnan(Δs_fore)

    veh_rear = scene[lead_follow.index_rear[vehicle_index]]
    veh_fore = scene[vehicle_index]
    relpos = get_frenet_relative_position(veh_fore, veh_rear, roadway)
    Δs_rear = relpos.Δs - veh_rear.def.length/2 -  veh_fore.def.length/2
    @assert !isnan(Δs_rear)

    return StateBounds(-Δs_rear, Δs_fore)
end
function get_state_bounds_t(scene::Scene, roadway::Roadway, lead_follow::LeadFollowRelationships, vehicle_index::Int)
    d_left = get_markerdist_left(scene[vehicle_index], roadway)
    d_right = get_markerdist_right(scene[vehicle_index], roadway)
    return StateBounds(-d_right, d_left)
end
function get_state_bounds_v(scene::Scene, roadway::Roadway, lead_follow::LeadFollowRelationships, vehicle_index::Int)
    v = scene[vehicle_index].state.v
    return StateBounds(BOUNDS_V[1]-v, BOUNDS_V[2]-v)
end
function get_state_bounds_ϕ(scene::Scene, roadway::Roadway, lead_follow::LeadFollowRelationships, vehicle_index::Int)
    ϕ = scene[vehicle_index].state.posF.ϕ
    return StateBounds(BOUNDS_ϕ[1]-ϕ, BOUNDS_ϕ[2]-ϕ)
end

immutable VehicleBounds
    s::StateBounds
    t::StateBounds
    v::StateBounds
    ϕ::StateBounds
end
function VehicleBounds{S,D,I,R}(scene::EntityFrame{S,D,I}, roadway::R, lead_follow::LeadFollowRelationships, vehicle_index::Int)
    return VehicleBounds(
                get_state_bounds_s(scene, roadway, lead_follow, vehicle_index),
                get_state_bounds_t(scene, roadway, lead_follow, vehicle_index),
                get_state_bounds_v(scene, roadway, lead_follow, vehicle_index),
                get_state_bounds_ϕ(scene, roadway, lead_follow, vehicle_index))
end

