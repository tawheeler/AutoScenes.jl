__precompile__(true)

module AutoScenes

using AutomotiveDrivingModels
using AutoViz
using Distributions
using Vec
import QuadGK: quadgk

export

    #####################################
    LeadFollowRelationships,

    StateBounds,
    domain_size,

    Vars,

    Assignment,
    Assignments,

    #####################################

    FactorGraph,

    #####################################
    assign_feature,
    assign_features,

    scope,
    inscope,

    ptilde_denom,
    prob_x_given_other,
    calc_expectation_x_given_other,
    log_ptilde,
    ptilde,


    #####################################
    # subscene_extraction

    extract_subscene!,
    get_num_vehicles_upstream_in_and_downstream,

    #####################################
    # learning

    log_pseudolikelihood,
    log_pseudolikelihood_derivative_single,
    log_pseudolikelihood_derivative_complete,
    log_pseudolikelihood_gradient!,

    #####################################
    # Batch Iterator

    BatchIterator,

    epoch_size,
    samples_so_far,
    current_epoch,
    get_sample,
    next_sample!,
    pull_batch!

    # #####################################
    # # SGA

    # StochasticGradientAscentParams,
    # GradientStepParams,
    # GradientParams,
    # PrintParams,

    # alloc_grad_velocities,
    # stochastic_gradient_ascent!,
    # step!,
    # parallel_step!,

    # #####################################
    # # sampling

    # SceneGenerator,

    # adheres_to_structure,
    # calc_acceptance_probability,
    # metropolis_hastings_step!,
    # metropolis_hastings!,
    # sample

include("utils.jl")
include("factor_graphs.jl")
include("features.jl")
include("subscene_extraction.jl")
include("learning.jl")
include("batch_iterator.jl")
# include("stochastic_gradient_ascent_params.jl")
# include("sampling.jl")
# include("io.jl")

# include("viz/viz_scene_structures.jl")

end # module
