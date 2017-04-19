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

"""
    Compute E[f(x ∣ other)]

    where xⱼ ~ P(⋅ | other)

Where f is the feature function,
vars are the inputs to the feature function set to their present value,
and i is the index of x, the variable we are running over.

This calculation is performed using Monte Carlo integration:

    E[f(x ∣ other )] ≈ [∑ f(xⱼ ∣ other) * Wⱼ ] / [∑ Wⱼ]

    where xⱼ ~ Uniform
      and Wⱼ = ptilde(xⱼ, other) / U(xⱼ)

Assignments is typically the subset of vectors that
"""
function calc_expectation_x_given_other{F<:Tuple{Vararg{Function}}, R}(
    i::Int, # index of the feature (in assignments) we are running this for
    j::Int, # index of the variable (in vars) we are running this for
    features::F, # shared feature functions
    θ::Vector{Float64}, # weights on the shared features
    vars::Vars, # all variables, set to current assignment
    assignments::Vector{Tuple{Int, Tuple{Vararg{Int}}}}, # assignment of index of shared feature → indeces of input vars
    roadway::R,
    nsamples::Int = 100, # number of Monte Carlo samples
    )::Float64

    feature_index, assignment = assignments[i]
    x₀ = vars.values[j] # store initial value
    U = Uniform(vars.bounds[j])
    f = features[feature_index]

    numerator = 0.0
    denominator = 0.0
    for k in 1 : nsamples
        Δx = rand(U)
        vars.values[j] = x₀ + Δx # set value
        W = ptilde(features, θ, vars, assignments, roadway) / pdf(U, Δx)
        numerator += W*f(vars, assignment, roadway) # unfortunately, this is computed twice
        denominator += W
    end

    vars.values[j] = x₀ # reset initial value

    return numerator/denominator
end