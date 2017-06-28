"""
A particular instance of a factor graph
"""
immutable FactorGraph{R}
    vars::Vars
    assignments::Assignments # feature index -> assignment containing variable indices
    scopes::Vector{Vector{Int}} # var_index -> scope
    roadway::R
end
function FactorGraph{R}(
    vars::Vars,
    assignments::Assignments,
    roadway::R,
    )

    scopes = [scope(var_index, assignments) for var_index in 1 : length(vars)]
    return FactorGraph{R}(vars, assignments, scopes, roadway)
end
function FactorGraph{F<:Tuple{Vararg{Function}}, S,D,I, R}(
    features::F,
    scene::EntityFrame{S,D,I},
    roadway::R,
    )

    vars = Vars(scene, roadway)
    assignments = assign_features(features, scene, roadway, vars)
    return FactorGraph(vars, assignments, roadway)
end