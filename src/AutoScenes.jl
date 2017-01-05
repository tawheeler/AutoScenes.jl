VERSION >= v"0.4.0-dev+6521" && __precompile__(true)

module AutoScenes

using AutomotiveDrivingModels
using AutoViz

export
    #####################################
    # features
    FeatureForms,
    GraphFeatureTemplate,
    GraphFeatureInstance,

    FEATURE_TEMPLATE_ROAD,
    FEATURE_TEMPLATE_FOLLOW,
    FEATURE_TEMPLATE_NEIGHBOR,

    evaluate,

    #####################################
    # factors
    SharedFactor,

    evaluate_dot,
    evaluate_exp,
    create_shared_factors,

    save_factors,
    load_factors,

    #####################################
    # scene_structures
    LeadFollowRelationships,
    FactorAssignment,
    SceneStructure,

    gen_scene_structure,
    evaluate_dot!,
    get_vehicle_indeces,

    #####################################
    # subscene_extraction
    SubSceneExtractParams,

    is_in_bounds,
    is_there_longitudinal_room,
    is_scene_well_behaved,
    pull_subscene,

    #####################################
    # structure_dataset
    SceneStructureDataset,
    SceneSource,

    get_scene_structure_and_roadway!,
    pull_scene_dataset,

    #####################################
    # scenedataset

    SceneDataset,

    get_scene_and_roadway!,

    #####################################
    # learning
    PseudolikelihoodPrealloc,

    reset_weights!,
    calc_pseudolikelihood,
    calc_pseudolikelihood_gradient,

    #####################################
    # Batch Sampler

    BatchSampler,

    restart!,
    next_index!,
    epoch_size,
    get_n_samples_used,

    #####################################
    # SGA

    StochasticGradientAscentParams,
    GradientStepParams,
    GradientParams,
    PrintParams,

    alloc_grad_velocities,
    stochastic_gradient_ascent!,
    step!,
    parallel_step!,

    #####################################
    # sampling

    SceneGenerator,

    adheres_to_structure,
    calc_acceptance_probability,
    metropolis_hastings_step!,
    metropolis_hastings!,
    sample

pkgdir = joinpath(dirname(@__FILE__), "..")
include(joinpath(pkgdir, "src", "features.jl"))
include(joinpath(pkgdir, "src", "factors.jl"))
include(joinpath(pkgdir, "src", "features", "core.jl"))
include(joinpath(pkgdir, "src", "scene_structures.jl"))
include(joinpath(pkgdir, "src", "subscene_extraction.jl"))
include(joinpath(pkgdir, "src", "structure_dataset.jl"))
include(joinpath(pkgdir, "src", "scenedataset.jl"))
include(joinpath(pkgdir, "src", "learning.jl"))
include(joinpath(pkgdir, "src", "stochastic_gradient_ascent_params.jl"))
include(joinpath(pkgdir, "src", "sampling.jl"))
include(joinpath(pkgdir, "src", "io.jl"))

include(joinpath(pkgdir, "src", "viz", "viz_scene_structures.jl"))

end # module
