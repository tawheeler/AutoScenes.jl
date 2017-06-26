__precompile__(true)

module AutoScenes

using AutomotiveDrivingModels
using AutoViz
using Distributions
using Vec

export

    #####################################
    LeadFollowRelationships,

    StateBounds,
    domain_size,

    Vars,
    FactorGraph,

    #####################################
    assign_feature,
    assign_features,

    scope,
    inscope,

    calc_expectation_x_given_other,
    log_ptilde,
    ptilde,


    #####################################
    # SceneStructure


    #####################################
    # subscene_extraction

    extract_subscene!,
    get_num_vehicles_upstream_in_and_downstream,

    # is_in_bounds,
    # is_there_longitudinal_room,
    # is_scene_well_behaved,
    # pull_subscene,

    # #####################################
    # # learning

    log_pseudolikelihood,
    log_pseudolikelihood_derivative_single,
    log_pseudolikelihood_derivative_complete,
    log_pseudolikelihood_gradient!

    # #####################################
    # # Batch Sampler

    # BatchSampler,

    # restart!,
    # next_index!,
    # epoch_size,
    # get_n_samples_used,

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
# include("scene_structures.jl")
include("subscene_extraction.jl")
# include("structure_dataset.jl")
# include("scenedataset.jl")
include("learning.jl")
# include("stochastic_gradient_ascent_params.jl")
# include("sampling.jl")
# include("io.jl")

# include("viz/viz_scene_structures.jl")

end # module
