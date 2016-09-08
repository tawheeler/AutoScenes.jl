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
        _set_and_standardize!(template, scene[vehicle_index].state.posF.t, 1)
        _set_and_standardize!(template, scene[vehicle_index].state.v,      2)
        _set_and_standardize!(template, scene[vehicle_index].state.posF.ϕ, 3)
    elseif template.form == FeatureForms.FOLLOW

        veh_rear = scene[vehicle_indeces[1]]
        veh_fore = scene[vehicle_indeces[2]]
        relpos = get_frenet_relative_position(veh_fore, veh_rear, roadway)

        Δs = relpos.Δs - veh_rear.def.length/2 -  veh_fore.def.length/2
        Δv = veh_fore.state.v - veh_rear.state.v

        _set_and_standardize!(template, Δs, 1)
        _set_and_standardize!(template, Δv, 2)
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
        for i in 1 : length(template.values)
            v = template.values[i]
            e = instance.exponents[i]
            retval += v^e
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
    elseif instance.form == FeatureForms.ROAD
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
        [Normal(-0.332,  1.629), # t
         Normal(29.883, 13.480), # v
         Normal(   0.0,  0.021*5), # ϕ
        ]
    )
const FEATURE_TEMPLATE_FOLLOW = GraphFeatureTemplate(FeatureForms.FOLLOW,
        [Normal(49.443, 30.552), # Δs
         Normal( 0.184,  4.203*10), # Δv
        ]
    )
const FEATURE_TEMPLATE_NEIGHBOR = GraphFeatureTemplate(FeatureForms.NEIGHBOR, Array(Normal{Float64}, 2))

function create_shared_factors()


    retval = Array(SharedFactor, 3)

    # Road
    road_instances = GraphFeatureInstance[]
    max_pow = 4
    for i in 0:max_pow
        for j in 0:max_pow-i
            for k in 0:max_pow-i-j
                if !(i == j == k == 0)
                    push!(road_instances, GraphFeatureInstance(FeatureForms.ROAD, [i*1.0, j*1.0, k*1.0]))
                end
            end
        end
    end
    retval[FeatureForms.ROAD] = SharedFactor(FEATURE_TEMPLATE_ROAD, road_instances, [0.0,-0.49096518762906405,-0.5187643101726188,-0.9624087682421549,-0.03457329970247343,-0.00016786356657403912,-0.0002480864638823717,-0.005506786817788355,-0.12259277311631592,-0.0030086316664208303,-0.4720617599324351,-0.3447188659113809,-0.026515539351620272,-0.16671326172780884,0.0,0.0,-0.22067775128101277,-0.5005877494332738,-0.044074917358080806,-0.027783349659240883,0.0,-0.10504939904183086,0.0,-0.0038248639806790414,-0.3708798329756009,0.0,-0.4295505877731457,0.0,-0.012666798785022644,-0.4148497103372788,-0.016490006737574472,-0.013080627296054397,-0.32627330387105347,-0.07471814548956576])

    # Follow
    follow_instances = GraphFeatureInstance[]
    max_pow = 3
    for i in 0 : max_pow
        for j in 0 : max_pow-i
            for k in 0 : max_pow-i-j
                if !(i == j == k == 0)
                    push!(follow_instances, GraphFeatureInstance(FeatureForms.FOLLOW, [i*1.0,j*1.0,k*1.0]))
                end
            end
        end
    end
    retval[FeatureForms.FOLLOW] = SharedFactor(FEATURE_TEMPLATE_FOLLOW, follow_instances, [0.0,-0.3318662735476311,0.0,-0.008531161402762881,-0.45392159633304113,-0.09503682461545739,-0.6318291021627882,0.0,-0.04447312908754733,-0.0004463126311876637,-0.5865699204323666,-0.1801965925023794,0.0,0.0,0.0,-0.12357674122687932,0.0,0.0,0.0])

    # Neighbor
    neighbor_instances = GraphFeatureInstance[]
    for i in 1 : 5
        push!(neighbor_instances, GraphFeatureInstance(FeatureForms.NEIGHBOR, i))
    end
    retval[FeatureForms.NEIGHBOR] = SharedFactor(FEATURE_TEMPLATE_NEIGHBOR, neighbor_instances, [-0.1678477166521841,-0.12415396288286973,-0.21435352517602127,-0.08887264406320598,0.0])

    retval
end