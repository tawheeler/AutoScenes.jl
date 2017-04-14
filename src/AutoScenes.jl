__precompile__(true)

module AutoScenes

using AutomotiveDrivingModels
using AutoViz

export

    #####################################
    uses_s,
    uses_t,
    uses_v,
    uses_ϕ,
    assign_feature,

    #####################################
    LeadFollowRelationships,
    SceneStructure,

    get_active_vehicles,

    # #####################################
    # # subscene_extraction
    # SubSceneExtractParams,

    # is_in_bounds,
    # is_there_longitudinal_room,
    # is_scene_well_behaved,
    # pull_subscene,

    # #####################################
    # # structure_dataset
    # SceneStructureDataset,
    # SceneSource,

    # get_scene_structure_and_roadway!,
    # pull_scene_dataset,

    # #####################################
    # # scenedataset

    # SceneDataset,

    # get_scene_and_roadway!,

    # #####################################
    # # learning
    # PseudolikelihoodPrealloc,

    VehicleBounds,
    domain_size_s,
    domain_size_t,
    domain_size_v,
    domain_size_ϕ

    # reset_weights!,
    # calc_pseudolikelihood
    # calc_pseudolikelihood_gradient

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
include("features.jl")
include("scene_structures.jl")
# include("subscene_extraction.jl")
# include("structure_dataset.jl")
# include("scenedataset.jl")
include("learning.jl")
# include("stochastic_gradient_ascent_params.jl")
# include("sampling.jl")
# include("io.jl")

# include("viz/viz_scene_structures.jl")

end # module
