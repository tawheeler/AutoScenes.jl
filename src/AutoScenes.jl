__precompile__(true)

module AutoScenes

using AutomotiveDrivingModels
using AutoViz
using Distributions
using Vec
using Discretizers
import QuadGK: quadgk

export

    #####################################
    FactorGraph,
    FactorModel,
    FactorGraphSceneGenerator,

    LeadFollowRelationships,

    StateBounds,
    domain_size,

    ZERO_BOUND,

    Vars,

    Assignment,
    Assignments,

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
    pull_batch!,

    #####################################
    # sampling

    metropolis_hastings_step!,
    metropolis_hastings!,

    #####################################
    # metrics

    KLDivMetric,
    assign_metric,
    get_counts,
    kullbeck_leibler_divergence


include("utils.jl")
include("factor_graphs.jl")
include("features.jl")
include("factormodel.jl")
include("subscene_extraction.jl")
include("learning.jl")
include("batch_iterator.jl")
include("sampling.jl")
include("metrics.jl")

# include("viz_scene_structures.jl")

end # module
