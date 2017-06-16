#=
ϕ: ∈ (-0.82, 0.82)
    Normal(0.0,  0.82)
	f = [  ϕ²,   ϕ⁴]
	w = [-1.0, -8.6]

t: ∈ (-1.7, 1.7)
    Normal(0.0,  1.7)
    f = [  t²,   t⁴]
    w = [-1.0, -8.6]

v: ∈ (-1, 32)
    Normal(13,  32.0-13.0)
    f = [   v,   v²,  v³,    v⁴]
    w = [-1.5, -10, -2.5, -10.0]

Δv: ∈ (-33, 33)
    Normal(0,  15.0)
    f = [ Δv², Δv⁴]
    w = [-9, 4.5]

Δs: ∈ (0, 100)
    Normal(30,  100.0)
    f = [ Δs¹,  Δs², Δs³, Δs⁴]
    w = [  -5, -7.5, 0.5, 2.0]
=#

const BOUNDS_V = (-1.0, 32.0) # m/s
const BOUNDS_ϕ = (-0.82, 0.82) # rad

const USE_S = true
const USE_T = true
const USE_V = true
const USE_ϕ = true

baremodule FeatureForms
    const ROAD     = 1
    const FOLLOW   = 2
    const NEIGHBOR = 3
end

function AutomotiveDrivingModels.extract!(
    template::GraphFeatureTemplate,
    scene::Scene,
    roadway::Roadway,
    vehicle_indeces::Vector{Int},
    )

    if template.form == FeatureForms.ROAD

        vehicle_index = vehicle_indeces[1]
        _set_standardize_and_clamp!(template, scene[vehicle_index].state.posF.t, 1)
        _set_standardize_and_clamp!(template, scene[vehicle_index].state.v,      2)
        _set_standardize_and_clamp!(template, scene[vehicle_index].state.posF.ϕ, 3)
    elseif template.form == FeatureForms.FOLLOW

        veh_rear = scene[vehicle_indeces[1]]
        veh_fore = scene[vehicle_indeces[2]]
        relpos = get_frenet_relative_position(veh_fore, veh_rear, roadway)

        Δs = relpos.Δs - veh_rear.def.length/2 -  veh_fore.def.length/2
        Δv = veh_fore.state.v - veh_rear.state.v

        _set_standardize_and_clamp!(template, Δs, 1)
        _set_standardize_and_clamp!(template, Δv, 2)
    else #if template.form == FeatureForms.NEIGHBOR

        vehA = scene[vehicle_indeces[1]]
        vehB = scene[vehicle_indeces[2]]

        t_CPA, d_CPA = get_time_and_dist_of_closest_approach(vehA, vehB, template.mem)

        template.values[1] = t_CPA
        template.values[2] = d_CPA
    end

    template
end
function evaluate(template::GraphFeatureTemplate, instance::GraphFeatureInstance)

    # NOTE: template must have already been extracted
    @assert(template.form == instance.form)

    retval = 0.0

    if template.form == FeatureForms.ROAD || template.form == FeatureForms.FOLLOW
        retval = 1.0
        for i in 1 : length(template.values)
            v = template.values[i]
            e = instance.exponents[i]
            retval *= v^e
        end
    else # template.form == FeatureForms.NEIGHBOR

        t_CPA = template.values[1]
        d_CPA = template.values[2]

        if instance.index == 1
            retval = convert(Float64, t_CPA == 0.0 && d_CPA == 0.0)
        elseif instance.index == 2
            retval = convert(Float64, 0.0 < t_CPA ≤ 1.0 && d_CPA ≤ 0.5)
        elseif instance.index == 3
            retval = convert(Float64, 1.0 < t_CPA ≤ 4.0 && d_CPA ≤ 0.5)
        elseif instance.index == 4
            retval = convert(Float64, 4.0 < t_CPA ≤ 10.0 && d_CPA ≤ 0.5)
        elseif instance.index == 5
            retval = convert(Float64, 10.0 < t_CPA || d_CPA > 0.5)
        else
            error("instance.index of $(instance.index) not valid!")
        end
    end

    retval::Float64
end

function uses_s(instance::GraphFeatureInstance)
    if instance.form == FeatureForms.FOLLOW
        return instance.exponents[1] > 0
    elseif instance.form == FeatureForms.NEIGHBOR
        return true
    end
    false
end
function uses_t(instance::GraphFeatureInstance)
    if instance.form == FeatureForms.ROAD
        return instance.exponents[1] > 0
    elseif instance.form == FeatureForms.NEIGHBOR
        return true
    end
    false
end
function uses_v(instance::GraphFeatureInstance)
    if instance.form == FeatureForms.ROAD
        return instance.exponents[2] > 0
    elseif instance.form == FeatureForms.FOLLOW
        return instance.exponents[2] > 0
    elseif instance.form == FeatureForms.NEIGHBOR
        return true
    end
    false
end
function uses_ϕ(instance::GraphFeatureInstance)
    if instance.form == FeatureForms.ROAD
        return instance.exponents[3] > 0
    elseif instance.form == FeatureForms.NEIGHBOR
        return true
    end
    false
end

const FEATURE_TEMPLATE_ROAD = GraphFeatureTemplate(FeatureForms.ROAD,
        [Normal( 0.0,  1.7), # t
         Normal(13.0, 32.0-13.0), # v
         Normal( 0.0,  0.82), # ϕ
        ]
    )
const FEATURE_TEMPLATE_FOLLOW = GraphFeatureTemplate(FeatureForms.FOLLOW,
        [Normal(30.0, 100.0), # Δs
         Normal( 0.0,  18.0), # Δv
        ]
    )
const FEATURE_TEMPLATE_NEIGHBOR = GraphFeatureTemplate(FeatureForms.NEIGHBOR, Array(Normal{Float64}, 2))

function create_shared_factors()


    retval = Array(SharedFactor, 3)

    # Road
    road_instances = GraphFeatureInstance[]
    push!(road_instances, GraphFeatureInstance(FeatureForms.ROAD, [2.0, 0.0, 0.0])) # t
    push!(road_instances, GraphFeatureInstance(FeatureForms.ROAD, [4.0, 0.0, 0.0]))
    push!(road_instances, GraphFeatureInstance(FeatureForms.ROAD, [0.0, 1.0, 0.0])) # v
    push!(road_instances, GraphFeatureInstance(FeatureForms.ROAD, [0.0, 2.0, 0.0]))
    push!(road_instances, GraphFeatureInstance(FeatureForms.ROAD, [0.0, 3.0, 0.0]))
    push!(road_instances, GraphFeatureInstance(FeatureForms.ROAD, [0.0, 4.0, 0.0]))
    push!(road_instances, GraphFeatureInstance(FeatureForms.ROAD, [0.0, 0.0, 2.0])) # ϕ
    push!(road_instances, GraphFeatureInstance(FeatureForms.ROAD, [0.0, 0.0, 4.0]))
    retval[FeatureForms.ROAD] = SharedFactor(FEATURE_TEMPLATE_ROAD, road_instances, [-1.0, -8.6,
                                                                                     -1.5, -10, -2.5, -10.0,
                                                                                     -1.0, -8.6])

    # Follow
    follow_instances = GraphFeatureInstance[]
    push!(follow_instances, GraphFeatureInstance(FeatureForms.FOLLOW, [2.0,0.0])) # Δv
    push!(follow_instances, GraphFeatureInstance(FeatureForms.FOLLOW, [4.0,0.0]))
    push!(follow_instances, GraphFeatureInstance(FeatureForms.FOLLOW, [0.0,1.0])) # Δs
    push!(follow_instances, GraphFeatureInstance(FeatureForms.FOLLOW, [0.0,2.0]))
    push!(follow_instances, GraphFeatureInstance(FeatureForms.FOLLOW, [0.0,3.0]))
    push!(follow_instances, GraphFeatureInstance(FeatureForms.FOLLOW, [0.0,4.0]))
    retval[FeatureForms.FOLLOW] = SharedFactor(FEATURE_TEMPLATE_FOLLOW, follow_instances, [-9, 4.5,
                                                                                           -5, -7.5, 0.5, 2.0])

    # Neighbor
    neighbor_instances = GraphFeatureInstance[]
    retval[FeatureForms.NEIGHBOR] = SharedFactor(FEATURE_TEMPLATE_NEIGHBOR, neighbor_instances, Float64[])

    retval
end

function get_start_scene_and_roadway(ncars::Int)
    roadway = gen_straight_roadway(1, 100000.0, lane_width=1.7)

    vehicles = Array(Vehicle, ncars)
    for i in 1 : ncars
        vehicles[i] = Vehicle(VehicleState(VecSE2(100.0 + (i-1)*46.0,0.0,0.0), roadway, 13.0), BoundingBoxDef(i, AgentClass.CAR, 4.0, 2.0))
    end

    (Scene(vehicles), roadway)
end