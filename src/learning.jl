
"""
Compute the log of the pseudolikelihood for a single datum or dataset
where the pseudolikelihood is ∏ p(x ∣ oth)
"""
function log_pseudolikelihood{F<:Tuple{Vararg{Function}}, R}(
    features::F,
    θ::Vector{Float64},
    vars::Vars,
    assignments::Vector{Tuple{Int, Tuple{Vararg{Int}}}},
    scopes::Vector{Vector{Int}}, # var_index -> scope
    roadway::R,
    nsamples::Int = 100, # number of Monte Carlo samples
    )::Float64

    retval = 0.0
    # loop through variables
    for (var_index, scope) in enumerate(scopes)

        # the positive term
        for assignment_index in scope
            feature_index, assignment = assignments[assignment_index]
            f = features[feature_index]
            retval += θ[feature_index] * f(vars, assignment, roadway)
        end

        # the negative term, computed via Monte Carlo integration
        neg_term = 0.0
        x₀ = vars.values[var_index] # store initial value
        U = Uniform(vars.bounds[var_index])
        # for k in 1 : nsamples
        #     Δx = rand(U)
        #     vars.values[var_index] = x₀ + Δx # set value
        #     subvalue = 0.0
        #     for assignment_index in scope
        #         feature_index, assignment = assignments[assignment_index]
        #         f = features[feature_index]
        #         for i in assignment
        #             subvalue += θ[feature_index] * f(vars, assignment, roadway)
        #         end
        #     end
        #     neg_term += exp(subvalue)
        # end
        # vars.values[var_index] = x₀ # reset initial value
        # neg_term = log(neg_term/nsamples)

        g = Δx -> begin
            vars.values[var_index] = x₀ + Δx # set value
            subvalue = 0.0
            for assignment_index in scope
                feature_index, assignment = assignments[assignment_index]
                f = features[feature_index]
                for i in assignment
                    subvalue += θ[feature_index] * f(vars, assignment, roadway)
                end
            end
            return exp(subvalue)
        end
        bound = vars.bounds[var_index]
        neg_term = log(quadgk(g, bound.Δlo, bound.Δhi, maxevals=nsamples)[1]) # note: only take estimated integral, not the err
        vars.values[var_index] = x₀ # reset initial value

        retval -= neg_term
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
    return retval / length(factorgraphs)
end

"""
Compute the derivative of the log pseudolikelihood with respect to a single instance of a (θ, f) pair.
"""
function log_pseudolikelihood_derivative_single{F<:Tuple{Vararg{Function}}, R}(
    assignment_index::Int, # index of the (θ,f) pair in assignments
    features::F,
    θ::Vector{Float64},
    vars::Vars,
    assignments::Vector{Tuple{Int, Tuple{Vararg{Int}}}},
    scopes::Vector{Vector{Int}}, # var_index -> scope
    roadway::R,
    nsamples::Int = 100, # number of Monte Carlo samples
    )

    feature_index, assignment = assignments[assignment_index]
    f = features[feature_index]
    retval = f(vars, assignment, roadway)
    for var_index in assignment
        # the negative term, computed via Monte Carlo integration
        retval -= calc_expectation_x_given_other(feature_index, var_index, features, θ, vars, assignments, scopes, roadway, nsamples)
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
    assignments::Vector{Tuple{Int, Tuple{Vararg{Int}}}},
    scopes::Vector{Vector{Int}}, # var_index -> scope
    roadway::R,
    nsamples::Int = 100, # number of Monte Carlo samples
    )

    retval = 0.0
    for (assignment_index, (feature_index2, assignment)) in enumerate(assignments)
        if feature_index == feature_index2
            retval += log_pseudolikelihood_derivative_single(assignment_index, features, θ, vars, assignments, scopes, roadway, nsamples)
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
    # TODO: parallelize
    for factorgraph in factorgraphs
        retval += log_pseudolikelihood_derivative_complete(feature_index, features, θ, factorgraph, nsamples)
    end
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
    nsamples::Int = 100, # number of Monte Carlo samples
    )

    # TODO: parallelize?
    for feature_index in 1 : length(features)
        ∇[feature_index] = log_pseudolikelihood_derivative_complete(feature_index,
            features, θ, factorgraphs, nsamples)
    end
    return ∇
end

# #####################

# type BatchSampler
#     dset::SceneStructureDataset
#     perm::Array{Int} # permutation over all samples in the dataset
#     perm_index::Int # location in the perm
#     epoch::Int # the current epoch (ie if this is 2 we have already been through dset once and are on our 2nd time through)
# end
# BatchSampler(dset::SceneStructureDataset) = BatchSampler(dset, randperm(length(dset)), 0, 1)

# epoch_size(sampler::BatchSampler) = length(sampler.dset)
# function get_n_samples_used(sampler::BatchSampler)
#     n_epochs = epoch_size(sampler)
#     n_epochs*(sampler.epoch - 1) + sampler.perm_index
# end
# function restart!(sampler::BatchSampler)
#     sampler.perm_index = 0
#     sampler.epoch = 1
#     sampler.perm = randperm(length(sampler.perm))
#     sampler
# end
# function next_index!(sampler::BatchSampler)
#     sampler.perm_index += 1
#     if sampler.perm_index > length(sampler.perm)
#         sampler.perm_index = 1
#         sampler.epoch += 1
#     end
#     sampler.perm[sampler.perm_index]
# end

# immutable SceneStructureRoadway
#     scene::Scene
#     structure::SceneStructure
#     roadway::Roadway
# end
# function next!(sampler::BatchSampler)
#     index = next_index!(sampler)
#     scene, structure, roadway = get_scene_structure_and_roadway!(Scene(), sampler.dset, index)
#     SceneStructureRoadway(scene, structure, roadway)
# end

# #####################

# function calc_pseudolikelihood_gradient(
#     form::Int,
#     feature_index::Int,
#     sampler::BatchSampler,
#     batch_size::Int,
#     n_samples_monte_carlo_integration::Int,
#     regularization::Float64,
#     rng::AbstractRNG=Base.GLOBAL_RNG,
#     scene::Scene = Scene(),
#     rec::SceneRecord = SceneRecord(2, 0.1),
#     factors::Vector{SharedFactor} = sampler.dset.factors,
#     )

#     retval = 0.0
#     ϕ = factors[form]
#     target_instance = ϕ.instances[feature_index]

#     for m in 1 : batch_size

#         structure_index = next_index!(sampler) # sample uniformly at random
#         scene, structure, roadway = get_scene_structure_and_roadway!(scene, sampler.dset, structure_index)

#         for fa in structure.factor_assignments
#             if fa.form == form # is the correct factor

#                 # first component
#                 extract!(ϕ, scene, roadway, fa.vehicle_indeces)
#                 eval = evaluate(ϕ.template, target_instance)

#                 for vehicle_index in fa.vehicle_indeces

#                     # only calc if the vehicle is active
#                     if vehicle_index in structure.active_vehicles

#                         if uses_s(target_instance)
#                             retval += eval
#                             retval += calc_pseudolikelihood_gradient_component_s(ϕ, factors, scene, structure, roadway, vehicle_index, fa.vehicle_indeces,
#                                                                                  target_instance,n_samples_monte_carlo_integration, rng, rec)
#                         end
#                         if uses_t(target_instance)
#                             retval += eval
#                             retval += calc_pseudolikelihood_gradient_component_t(ϕ, factors, scene, structure, roadway, vehicle_index, fa.vehicle_indeces,
#                                                                                  target_instance,n_samples_monte_carlo_integration, rng, rec)
#                         end
#                         if uses_v(target_instance)
#                             retval += eval
#                             retval += calc_pseudolikelihood_gradient_component_v(ϕ, factors, scene, structure, roadway, vehicle_index, fa.vehicle_indeces,
#                                                                                  target_instance,n_samples_monte_carlo_integration, rng, rec)
#                         end
#                         if uses_ϕ(target_instance)
#                             retval += eval
#                             retval += calc_pseudolikelihood_gradient_component_ϕ(ϕ, factors, scene, structure, roadway, vehicle_index, fa.vehicle_indeces,
#                                                                                  target_instance,n_samples_monte_carlo_integration, rng, rec)
#                         end
#                     end
#                 end
#             end
#         end
#     end

#     retval /= batch_size

#     # add regularization
#     retval -= 2*regularization*ϕ.weights[feature_index]

#     retval
# end
# function calc_pseudolikelihood_gradient(
#     form::Int,
#     feature_index::Int,
#     factors::Vector{SharedFactor},
#     samples::Vector{SceneStructureRoadway}, # of length batch_size
#     n_samples_monte_carlo_integration::Int,
#     regularization::Float64,
#     rng::AbstractRNG=Base.GLOBAL_RNG,
#     scene::Scene = Scene(),
#     rec::SceneRecord = SceneRecord(2, 0.1),
#     )

#     retval = 0.0
#     ϕ = factors[form]
#     target_instance = ϕ.instances[feature_index]

#     for ssr in samples

#         scene, structure, roadway = ssr.scene, ssr.structure, ssr.roadway

#         for fa in structure.factor_assignments
#             if fa.form == form # is the correct factor

#                 # first component
#                 extract!(ϕ, scene, roadway, fa.vehicle_indeces)
#                 eval = evaluate(ϕ.template, target_instance)

#                 for vehicle_index in fa.vehicle_indeces

#                     # only calc if the vehicle is active
#                     if vehicle_index in structure.active_vehicles

#                         if uses_s(target_instance)
#                             retval += eval
#                             retval += calc_pseudolikelihood_gradient_component_s(ϕ, factors, scene, structure, roadway, vehicle_index, fa.vehicle_indeces,
#                                                                                  target_instance,n_samples_monte_carlo_integration, rng, rec)
#                         end
#                         if uses_t(target_instance)
#                             retval += eval
#                             retval += calc_pseudolikelihood_gradient_component_t(ϕ, factors, scene, structure, roadway, vehicle_index, fa.vehicle_indeces,
#                                                                                  target_instance,n_samples_monte_carlo_integration, rng, rec)
#                         end
#                         if uses_v(target_instance)
#                             retval += eval
#                             retval += calc_pseudolikelihood_gradient_component_v(ϕ, factors, scene, structure, roadway, vehicle_index, fa.vehicle_indeces,
#                                                                                  target_instance,n_samples_monte_carlo_integration, rng, rec)
#                         end
#                         if uses_ϕ(target_instance)
#                             retval += eval
#                             retval += calc_pseudolikelihood_gradient_component_ϕ(ϕ, factors, scene, structure, roadway, vehicle_index, fa.vehicle_indeces,
#                                                                                  target_instance,n_samples_monte_carlo_integration, rng, rec)
#                         end
#                     end
#                 end
#             end
#         end
#     end

#     retval /= length(samples)

#     # add regularization
#     retval -= 2*regularization*ϕ.weights[feature_index]

#     retval
# end

# #####################

# function reset_weights!(factors::Vector{SharedFactor}, σ::Float64=1.0)
#     for ϕ in factors
#         copy!(ϕ.weights, σ*randn(Float64, length(ϕ.weights)))
#     end
#     factors
# end