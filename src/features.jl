"""
    For a given scene and roadway, return all of the vehicle_indices::NTuple{Int} that are valid for this shared feature
These values are returned as a Vector{Assignment}
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

    assignments = Tuple{Int, Assignment}[]
    for (i, f) in enumerate(features)
        sub_assignments = assign_feature(f, scene, roadway, vars)
        for assignment in sub_assignments
            push!(assignments, (i, assignment))
        end
    end
    return assignments
end

"""
    Compute log_ptilde(assignment) = exp(θᵀf)
"""
function log_ptilde{F<:Tuple{Vararg{Function}}, R}(
    features::F,
    θ::Vector{Float64},
    vars::Vars,
    assignments::Assignments,
    roadway::R,
    )::Float64

    v = 0.0
    for (feature_index, assignment) in assignments
        f = features[feature_index]::Function
        w = θ[feature_index]::Float64
        v += w*f(vars, assignment, roadway)
    end
    return v
end
function log_ptilde{F<:Tuple{Vararg{Function}}, R}(
    features::F,
    θ::Vector{Float64},
    vars::Vars,
    assignments::Assignments,
    scope::Vector{Int},
    roadway::R,
    )::Float64

    v = 0.0
    for assignment_index in scope
        feature_index, assignment = assignments[assignment_index]
        f = features[feature_index]
        w = θ[feature_index]
        v += w*f(vars, assignment, roadway)
    end
    return v
end

"""
    Compute ptilde(assignment) = exp(θᵀf)
"""
function ptilde{F<:Tuple{Vararg{Function}}, R}(
    features::F,
    θ::Vector{Float64},
    vars::Vars,
    assignments::Assignments,
    roadway::R,
    )::Float64

    v = log_ptilde(features, θ, vars, assignments, roadway)
    return exp(v)
end
function ptilde{F<:Tuple{Vararg{Function}}, R}(
    features::F,
    θ::Vector{Float64},
    vars::Vars,
    assignments::Assignments,
    scope::Vector{Int},
    roadway::R,
    )::Float64

    v = log_ptilde(features, θ, vars, assignments, scope, roadway)
    return exp(v)
end

"""
    Returns a list of indices for all assignments
    for which scope(fⱼ) ∋ xₖ

    That is, a list of assignments such that xₖ is included.
"""
function scope(
    var_index::Int,
    assignments::Assignments,
    )

    retval = Int[]
    for (assignment_index,(feature_index, assignment)) in enumerate(assignments)
        if var_index ∈ assignment
            push!(retval, assignment_index)
        end
    end
    return retval
end

"""
Returns whether xₖ ∈ scope(fⱼ)

where var_index is the index of the variable (xₖ)
      feature_index is the index of the assigned feature (fⱼ)
"""
function inscope(
    var_index::Int,
    assignment_index::Int,
    assignments::Assignments,
    )

    assignment = assignments[assignment_index][2]
    return var_index ∈ assignment
end

"""
    Compute ptilde_denom

∫ ptilde(τ, other) d τ
"""
function ptilde_denom{F<:Tuple{Vararg{Function}}, R}(
    j::Int, # index of the variable (in vars) we are running this for
    features::F, # shared feature functions
    θ::Vector{Float64}, # weights on the shared features
    vars::Vars, # all variables, set to current assignment
    assignments::Assignments, # assignment of index of shared feature → indeces of input vars
    roadway::R,
    nsamples::Int = 100, # number of quadrature samples
    )

    x₀ = vars.values[j] # store initial value
    bound = vars.bounds[j]
    retval = quadgk(τ->begin
                    vars.values[j] = x₀ + τ # set value
                    return ptilde(features, θ, vars, assignments, roadway)
                end, bound.Δlo, bound.Δhi, maxevals=nsamples)[1]
    vars.values[j] = x₀ # reset initial value

    return retval
end

"""
    Compute E[f(x ∣ other)]

    where xⱼ ~ P(⋅ | other)

Where f is the feature function,
vars are the inputs to the feature function set to their present value,
and i is the index of x, the variable we are running over.

This calculation is performed using Monte Carlo integration:
(actually, now with gaussian quadrature)

    E[f(x ∣ other )] ≈ [∑ f(xⱼ ∣ other) * Wⱼ ] / [∑ Wⱼ]

    where xⱼ ~ Uniform
      and Wⱼ = ptilde(xⱼ, other) / U(xⱼ)
"""
function calc_expectation_x_given_other{F<:Tuple{Vararg{Function}}, R}(
    i::Int, # index of the feature (in assignments) we are running this for
    j::Int, # index of the variable (in vars) we are running this for
    features::F, # shared feature functions
    θ::Vector{Float64}, # weights on the shared features
    vars::Vars, # all variables, set to current assignment
    assignments::Assignments, # assignment of index of shared feature → indeces of input vars
    roadway::R,
    nsamples::Int = 100, # number of Monte Carlo samples
    )::Float64

    feature_index, assignment = assignments[i]
    x₀ = vars.values[j] # store initial value
    f = features[feature_index]

    # integrate f * pdf
    bound = vars.bounds[j]
    pdenom = ptilde_denom(j, features, θ, vars, assignments, roadway, nsamples)
    retval = quadgk(
            Δx->begin
                vars.values[j] = x₀ + Δx # set value
                fval = f(vars, assignment, roadway)
                pdfval = ptilde(features, θ, vars, assignments, roadway) / pdenom
                return fval*pdfval
            end,
            bound.Δlo, bound.Δhi, maxevals=nsamples
        )[1]

    vars.values[j] = x₀ # reset initial value

    return retval
end
function calc_expectation_x_given_other{F<:Tuple{Vararg{Function}}, R}(
    i::Int, # index of the feature (in assignments) we are running this for
    j::Int, # index of the variable (in vars) we are running this for
    features::F, # shared feature functions
    θ::Vector{Float64}, # weights on the shared features
    vars::Vars, # all variables, set to current assignment
    assignments::Assignments, # assignment of index of shared feature → indeces of input vars
    scopes::Vector{Vector{Int}},
    roadway::R,
    nsamples::Int = 100, # number of Monte Carlo samples
    )::Float64

    feature_index, assignment = assignments[i]
    x₀ = vars.values[j] # store initial value
    # U = Uniform(vars.bounds[j])
    f = features[feature_index]
    scope = scopes[j] # scope of j

    warn("scope not used, must implement this properly!")

    # # integrate f * pdf
    # bound = vars.bounds[j]
    # pdenom = ptilde_denom(j, features, θ, vars, assignments, roadway, nsamples)
    # retval = quadgk(
    #         Δx->begin
    #             vars.values[j] = x₀ + Δx # set value
    #             fval = f(vars, assignment, roadway)
    #             pdfval = ptilde(features, θ, vars, assignments, roadway) / pdenom
    #             return fval*pdfval
    #         end,
    #         bound.Δlo, bound.Δhi, maxevals=nsamples
    #     )[1]

    # numerator = 0.0
    # denominator = 0.0
    # for k in 1 : nsamples
    #     Δx = rand(U)
    #     vars.values[j] = x₀ + Δx # set value
    #     W = ptilde(features, θ, vars, assignments, roadway) / pdf(U, Δx)
    #     numerator += W*f(vars, assignment, roadway) # unfortunately, this is computed twice
    #     denominator += W
    # end

    vars.values[j] = x₀ # reset initial value

    return numerator/denominator
end