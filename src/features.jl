"""
    For a given scene and roadway, return all of the vehicle_indices::NTuple{Int} that are valid for this shared feature
These values are returned as a Vector{Tuple{Vararg{Int}}}
"""
function assign_feature{F<:Function,S,D,I,R}(
    f::F,
    scene::EntityFrame{S,D,I},
    roadway::R,
    )

    error("assign_feature not implemented for $F")
end

function assign_features{F<:Tuple{Vararg{Function}}, S,D,I, R}(
    features::F,
    scene::EntityFrame{S,D,I},
    roadway::R,
    vars::Vars,
    )

    assignments = Tuple{Int, Tuple{Vararg{Int}}}[]
    for (i, f) in enumerate(features)
        sub_assignments = assign_feature(f, scene, roadway, vars)
        for assignment in sub_assignments
            push!(assignments, (i, assignment))
        end
    end
    return assignments
end