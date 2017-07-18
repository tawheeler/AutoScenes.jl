
"""
Compute the log of the pseudolikelihood for a single datum or dataset
where the pseudolikelihood is ∏ p(x ∣ oth)
"""
function log_pseudolikelihood{F<:Tuple{Vararg{Function}}, R}(
    features::F,
    θ::Vector{Float64},
    vars::Vars,
    assignments::Assignments,
    scopes::Vector{Vector{Int}}, # var_index -> scope
    roadway::R,
    nsamples::Int = 100, # number of Monte Carlo samples
    )::Float64

    retval = 0.0
    # loop through variables
    for (var_index, scope) in enumerate(scopes)
        if !isempty(scope)

            # the positive term
            for assignment_index in scope
                feature_index, assignment = assignments[assignment_index]
                f = features[feature_index]
                retval += θ[feature_index] * f(vars, assignment, roadway)
            end

            # the negative term, computed via gaussian quadrature
            neg_term = log(ptilde_denom(var_index, features, θ, vars, assignments, scope, roadway))
            retval -= neg_term
        end
    end

    return retval
end
function log_pseudolikelihood{F<:Tuple{Vararg{Function}}, R}(
    features::F,
    θ::Vector{Float64},
    factorgraph::FactorGraph{R},
    nsamples::Int = 100, # number of Monte Carlo samples
    )

    return log_pseudolikelihood(features, θ, factorgraph.vars,
            factorgraph.assignments, factorgraph.scopes, factorgraph.roadway, nsamples)
end
function log_pseudolikelihood{F<:Tuple{Vararg{Function}}, R}(
    features::F,
    θ::Vector{Float64},
    factorgraphs::Vector{FactorGraph{R}},
    nsamples::Int = 100, # number of Monte Carlo samples
    )::Float64

    retval = 0.0
    # TODO: parallelize
    for factorgraph in factorgraphs
        retval += log_pseudolikelihood(features, θ, factorgraph, nsamples)
    end
    return retval
end

"""
Compute the derivative of the log pseudolikelihood with respect to a single instance of a (θ, f) pair.
"""
function log_pseudolikelihood_derivative_single{F<:Tuple{Vararg{Function}}, R}(
    assignment_index::Int, # index of the (θ,f) pair in assignments
    features::F,
    θ::Vector{Float64},
    vars::Vars,
    assignments::Assignments,
    scopes::Vector{Vector{Int}}, # var_index -> scope
    roadway::R,
    nsamples::Int = 100, # number of Monte Carlo samples
    )

    feature_index, assignment = assignments[assignment_index]
    f = features[feature_index]
    pos_term = f(vars, assignment, roadway) # the positive term
    retval = 0.0
    for var_index in assignment
        # the negative term
        neg_term = calc_expectation_x_given_other(assignment_index, var_index, features, θ, vars, assignments, scopes, roadway, nsamples)
        retval += pos_term - neg_term
    end

    return retval
end
function log_pseudolikelihood_derivative_single{F<:Tuple{Vararg{Function}}, R}(
    assignment_index::Int, # index of the (θ,f) pair in assignments
    features::F,
    θ::Vector{Float64},
    factorgraph::FactorGraph{R},
    nsamples::Int = 100, # number of Monte Carlo samples
    )

    return log_pseudolikelihood_derivative_single(assignment_index, features, θ, factorgraph.vars,
            factorgraph.assignments, factorgraph.scopes, factorgraph.roadway, nsamples)
end

"""
Compute the derivative of the log pseudolikelihood with respect to a shared θ value.
"""
function log_pseudolikelihood_derivative_complete{F<:Tuple{Vararg{Function}}, R}(
    feature_index::Int,
    features::F,
    θ::Vector{Float64},
    vars::Vars,
    assignments::Assignments,
    scopes::Vector{Vector{Int}}, # var_index -> scope
    roadway::R,
    nsamples::Int = 100, # number of Monte Carlo samples
    )

    retval = 0.0
    for assignment_index in 1 : length(assignments)
        # for some reason doing it this way does not allocate memory
        if feature_index == assignments[assignment_index][1]
            Δ = log_pseudolikelihood_derivative_single(assignment_index, features, θ, vars, assignments, scopes, roadway, nsamples)
            retval += Δ
        end
    end
    return retval
end
function log_pseudolikelihood_derivative_complete{F<:Tuple{Vararg{Function}}, R}(
    feature_index::Int,
    features::F,
    θ::Vector{Float64},
    factorgraph::FactorGraph{R},
    nsamples::Int = 100, # number of Monte Carlo samples
    )

    return log_pseudolikelihood_derivative_complete(feature_index, features, θ, factorgraph.vars,
        factorgraph.assignments, factorgraph.scopes, factorgraph.roadway, nsamples)
end
function log_pseudolikelihood_derivative_complete{F<:Tuple{Vararg{Function}}, R}(
    feature_index::Int,
    features::F,
    θ::Vector{Float64},
    factorgraphs::Vector{FactorGraph{R}},
    nsamples::Int = 100, # number of Monte Carlo samples
    )::Float64

    retval = 0.0
    # TODO: parallelize note that we may need multiple copies since we mutate vars
    for factorgraph in factorgraphs
        Δ = log_pseudolikelihood_derivative_complete(feature_index, features, θ, factorgraph, nsamples)
        retval += Δ
    end
    # retval = @parallel (+) for factorgraph in factorgraphs
    #     retval += log_pseudolikelihood_derivative_complete(feature_index, features, θ, factorgraph, nsamples)
    # end
    return retval / length(factorgraphs)
end


"""
Compute the gradient of the log pseudolikelihood with respect to a shared θ
Fills ∇, which must be at least as long as `features`
"""
function log_pseudolikelihood_gradient!{F<:Tuple{Vararg{Function}}, R}(
    ∇::Vector{Float64},
    features::F,
    θ::Vector{Float64},
    factorgraphs::Vector{FactorGraph{R}},
    nsamples::Int = 100, # max number of quadrature samples
    )

    # TODO: parallelize?
    for feature_index in 1 : length(features)
        ∇[feature_index] = log_pseudolikelihood_derivative_complete(feature_index,
            features, θ, factorgraphs, nsamples)
    end
    return ∇
end

# #####################

# function reset_weights!(factors::Vector{SharedFactor}, σ::Float64=1.0)
#     for ϕ in factors
#         copy!(ϕ.weights, σ*randn(Float64, length(ϕ.weights)))
#     end
#     factors
# end