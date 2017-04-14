"""
F is the type associated with a feature extraction function:

f(scene, roadway, vehicle_indices::Tuple{Vararg{Int}}) -> Float64

These functions tell you whether a particular vehicle state is used
"""
uses_s{F<:Function}(ϕ::F) = error("uses_s not implemented for $F")
uses_t{F<:Function}(ϕ::F) = error("uses_t not implemented for $F")
uses_v{F<:Function}(ϕ::F) = error("uses_v not implemented for $F")
uses_ϕ{F<:Function}(ϕ::F) = error("uses_ϕ not implemented for $F")

"""
    For a given scene and roadway, return all of the vehicle_indices::NTuple{Int} that are valid for this shared feature
These values are returned as a Vector{Tuple{Vararg{Int}}}
"""
function assign_feature{F<:Function}(
    f::F,
    scene::Scene,
    roadway::Roadway,
    active_vehicles::Set{Int},
    lead_follow::LeadFollowRelationships,
    )

    error("assign_factors not implemented for $F")
end


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

function SceneStructure{SharedFeatures <: Tuple{Vararg{Function}}}(
    scene::Scene,
    roadway::Roadway,
    shared_features::SharedFeatures,
    vehicle_indices::AbstractVector{Int} = 1:length(scene),
    )

    lead_follow = LeadFollowRelationships(scene, roadway, vehicle_indices)
    active_vehicles = get_active_vehicles(lead_follow)

    assignments = Dict{Function, Vector{Tuple{Vararg{Int}}}}()
    for f in shared_features
        assignments[f] = assign_feature(f, scene, roadway, active_vehicles, lead_follow)
    end

    SceneStructure{Factors}(assignments, lead_follow, active_vehicles)
end