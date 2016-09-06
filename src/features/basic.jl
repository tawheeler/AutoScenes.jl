const BOUNDS_V = (0.0, 2.0) # m/s
const BOUNDS_ϕ = (-0.82, 0.82) # rad

const USE_S = false
const USE_T = false
const USE_V = true
const USE_ϕ = false

baremodule FeatureForms
    const ROAD     = 1
    const FOLLOW   = 2
end

function AutomotiveDrivingModels.extract!(
    template::GraphFeatureTemplate,
    scene::Scene,
    roadway::Roadway,
    vehicle_indeces::Vector{Int},
    )

    if template.form == FeatureForms.ROAD

        vehicle_index = vehicle_indeces[1]
        _set_and_standardize!(template, scene[vehicle_index].state.v,      1)
    else #if template.form == FeatureForms.FOLLOW

        veh_rear = scene[vehicle_indeces[1]]
        veh_fore = scene[vehicle_indeces[2]]
        relpos = get_frenet_relative_position(veh_fore, veh_rear, roadway)

        Δv = veh_fore.state.v - veh_rear.state.v

        _set_and_standardize!(template, Δv, 1)
    end

    template
end
function evaluate(template::GraphFeatureTemplate, instance::GraphFeatureInstance)

    # NOTE: template must have already been extracted
    @assert(template.form == instance.form)

    retval = 0.0

    for i in 1 : length(template.values)
        v = template.values[i]
        e = instance.exponents[i]
        retval += v^e
    end

    retval::Float64
end

uses_s(instance::GraphFeatureInstance) = false
uses_t(instance::GraphFeatureInstance) = false
uses_v(instance::GraphFeatureInstance) = true
uses_ϕ(instance::GraphFeatureInstance) = false

const FEATURE_TEMPLATE_ROAD = GraphFeatureTemplate(FeatureForms.ROAD, [Normal(0.0, 1.0)])
const FEATURE_TEMPLATE_FOLLOW = GraphFeatureTemplate(FeatureForms.FOLLOW, [Normal( 0.0,  1.0)])

function create_shared_factors()

    retval = Array(SharedFactor, 2)

    # Road
    road_instances = GraphFeatureInstance[]
    push!(road_instances, GraphFeatureInstance(FeatureForms.ROAD, [1.0])) # v
    push!(road_instances, GraphFeatureInstance(FeatureForms.ROAD, [2.0])) # v²
    retval[FeatureForms.ROAD] = SharedFactor(FEATURE_TEMPLATE_ROAD, road_instances, [1.0, 0.1])

    # Follow
    follow_instances = GraphFeatureInstance[]
    push!(follow_instances, GraphFeatureInstance(FeatureForms.FOLLOW, [1.0])) # Δv
    retval[FeatureForms.FOLLOW] = SharedFactor(FEATURE_TEMPLATE_FOLLOW, follow_instances, [1.0])

    retval
end