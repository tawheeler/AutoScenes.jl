"""
SceneStructure - a factor graph for a particular scene.

This is parameterized by a typle of shared features.
The shared features are assigned to vehicles in the scene according to the assignments.
It also stores the lead-follow relationships and set of active vehicles for convenience.
"""
type SceneStructure{SharedFeatures <: Tuple{Vararg{Function}}} # a Factor Graph
    assignments::Dict{Function, Vector{Tuple{Vararg{Int}}}} # maps each shared feature to the vehicle sets it is assigned to
                # NOTE: hash is based on object pointer, which is what we want
    lead_follow::LeadFollowRelationships
    active_vehicles::Set{Int} # set of vehicles that can be manipulated (vehicle index)
end

function SceneStructure{SharedFeatures <: Tuple{Vararg{Function}}, S, D, I, R}(
    scene::EntityFrame{S,D,I},
    roadway::R,
    shared_features::SharedFeatures,
    vehicle_indices::AbstractVector{Int} = 1:length(scene),
    )

    lead_follow = LeadFollowRelationships(scene, roadway, vehicle_indices)
    active_vehicles = get_active_vehicles(lead_follow)

    assignments = Dict{Function, Vector{Tuple{Vararg{Int}}}}()
    for f in shared_features
        assignments[f] = assign_feature(f, scene, roadway, active_vehicles, lead_follow)
    end

    SceneStructure{SharedFeatures}(assignments, lead_follow, active_vehicles)
end