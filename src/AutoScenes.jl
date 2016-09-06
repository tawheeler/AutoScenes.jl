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

    extract!,
    evaluate,

    #####################################
    # factors
    SharedFactor,

    evaluate_dot,
    evaluate_exp,
    create_shared_factors,

    #####################################
    # scene_structures
    LeadFollowRelationships,
    FactorAssignment,
    SceneStructure,

    gen_scene_structure,

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

    get_scene_structure_and_roadway!,
    pull_scene_dataset,

    #####################################
    # learning
    PseudolikelihoodPrealloc,

    calc_pseudolikelihood,
    calc_pseudolikelihood_gradient,

    #####################################
    # SGA

    StochasticGradientAscentParams,
    PrintParams,

    alloc_grad_velocities,
    stochastic_gradient_ascent!,
    step!

include(Pkg.dir("AutoScenes", "src", "features.jl"))
include(Pkg.dir("AutoScenes", "src", "factors.jl"))
include(Pkg.dir("AutoScenes", "src", "scene_structures.jl"))
include(Pkg.dir("AutoScenes", "src", "subscene_extraction.jl"))
include(Pkg.dir("AutoScenes", "src", "structure_dataset.jl"))
include(Pkg.dir("AutoScenes", "src", "learning.jl"))
include(Pkg.dir("AutoScenes", "src", "stochastic_gradient_ascent_params.jl"))

include(Pkg.dir("AutoScenes", "src", "viz", "viz_scene_structures.jl"))

end # module
