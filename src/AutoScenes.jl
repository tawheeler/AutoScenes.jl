VERSION >= v"0.4.0-dev+6521" && __precompile__(true)

module AutoScenes

using AutomotiveDrivingModels
using AutoViz

export
    FeatureForms,
    GraphFeatureTemplate,
    GraphFeatureInstance,

    FEATURE_TEMPLATE_ROAD,
    FEATURE_TEMPLATE_FOLLOW,
    FEATURE_TEMPLATE_NEIGHBOR,

    extract!,
    evaluate,

    SharedFactor,
    evaluate_dot,
    evaluate_exp,
    create_shared_factors,

    LeadFollowRelationships,
    FactorAssignment,
    SceneStructure,

    gen_scene_structure,

    SubSceneExtractParams,

    is_in_bounds,
    is_there_longitudinal_room,
    is_scene_well_behaved,
    pull_subscene,

    SceneStructureDataset,

    get_scene_and_structure!,
    pull_scene_dataset

include(Pkg.dir("AutoScenes", "src", "features.jl"))
include(Pkg.dir("AutoScenes", "src", "factors.jl"))
include(Pkg.dir("AutoScenes", "src", "scene_structures.jl"))
include(Pkg.dir("AutoScenes", "src", "subscene_extraction.jl"))
include(Pkg.dir("AutoScenes", "src", "structure_dataset.jl"))

include(Pkg.dir("AutoScenes", "src", "viz", "viz_scene_structures.jl"))

end # module
