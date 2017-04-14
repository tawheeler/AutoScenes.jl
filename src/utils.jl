type LeadFollowRelationships # needed for bounds
    index_fore::Vector{Int}
    index_rear::Vector{Int}
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