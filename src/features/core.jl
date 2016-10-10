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
    max_pow = 3
    for i in 0:max_pow
        for j in 0:max_pow-i
            for k in 0:max_pow-i-j
                if !(i == j == k == 0)
                    push!(road_instances, GraphFeatureInstance(FeatureForms.ROAD, [i, j, k]))
                end
            end
        end
    end
    retval[FeatureForms.ROAD] = SharedFactor(FEATURE_TEMPLATE_ROAD, road_instances, [-7.44583,-16.3951,5.16565,-30.0,-17.0643,15.9508,20.0,-2.00284,-30.0,-30.0,17.1201,-5.07435,20.0,20.0,-27.1647,20.0,-15.887,-30.0,-30.0])

    # Follow
    follow_instances = GraphFeatureInstance[]
    max_pow = 3
    for i in 0 : max_pow
        for j in 0 : max_pow-i
            if !(i == j == 0)
                push!(follow_instances, GraphFeatureInstance(FeatureForms.FOLLOW, [i,j]))
            end
        end
    end
    retval[FeatureForms.FOLLOW] = SharedFactor(FEATURE_TEMPLATE_FOLLOW, follow_instances, [1.5447,20.0,-22.0025,-30.0,-15.9631,-17.2449,20.0,19.832,-29.9064])

    # Neighbor
    neighbor_instances = GraphFeatureInstance[]
    for i in 1 : 5
        push!(neighbor_instances, GraphFeatureInstance(FeatureForms.NEIGHBOR, i))
    end
    retval[FeatureForms.NEIGHBOR] = SharedFactor(FEATURE_TEMPLATE_NEIGHBOR, neighbor_instances, [-21.9485,-21.1018,-13.5304,20.0,20.0])

    retval
end

#=
iter: 28
time: 9788612 milliseconds
learning rate: 0.3811713571735518
batch_size:    56
n_samples:
weights:
1  [5.89984,-18.9764,-28.9311,-30.0,-21.2736,19.8708,20.0,-25.8472,-30.0,-30.0,-7.55589,4.79533,19.5066,-18.9996,-30.0,20.0,-30.0,-30.0,-30.0]
2  [-24.958,-15.6688,-5.7417,-30.0,-22.6837,-25.7289,20.0,-14.3953,-23.1918]
3  [-23.3536,-25.0045,-23.795,20.0,20.0]

iter: 68
time: 45098135 milliseconds
learning rate: 0.25499287312478264
batch_size:    136
n_samples:
weights:
1  [-7.60763,4.92289,19.963,-30.0,-23.4948,0.583121,20.0,1.64827,-30.0,-30.0,5.0555,-3.35636,20.0,20.0,-26.8478,20.0,-20.5293,-30.0,-29.556]
2  [-22.3228,20.0,-5.54496,-30.0,-30.0,-18.9566,20.0,18.0309,-28.9458]
3  [-11.3257,-10.8472,-29.0428,-7.00286,20.0]

iter: 87
time: 70130997 milliseconds
learning rate: 0.21066711107738406
batch_size:    174
n_samples:
weights:
1  [5.06477,-10.6898,1.41138,-30.0,-23.5192,-0.0243049,20.0,1.13066,-30.0,-30.0,10.7815,-11.902,20.0,18.8176,-30.0,20.0,-26.8264,-30.0,-30.0]
2  [-10.3414,19.9659,-22.2042,-30.0,-11.6497,-11.8009,20.0,15.7166,-29.3574]
3  [-29.6686,-5.92442,-20.8276,8.13927,20.0]

iter: 102
time: 94210659 milliseconds
learning rate: 0.18118600893024847
batch_size:    204
n_samples:
weights:
1  [-7.44583,-16.3951,5.16565,-30.0,-17.0643,15.9508,20.0,-2.00284,-30.0,-30.0,17.1201,-5.07435,20.0,20.0,-27.1647,20.0,-15.887,-30.0,-30.0]
2  [1.5447,20.0,-22.0025,-30.0,-15.9631,-17.2449,20.0,19.832,-29.9064]
3  [-21.9485,-21.1018,-13.5304,20.0,20.0]
=#