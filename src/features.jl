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