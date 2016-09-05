# const VEHICLE_FEATURE_SIZE = 4 # {s, t, v, ϕ}

baremodule FeatureForms
    const ROAD     = 1
    const FOLLOW   = 2
    const NEIGHBOR = 3
end

immutable GraphFeatureTemplate
    form::Int # feature form
    values::Vector{Float64} # [t,v,ϕ] for road, [Δs, Δv] for follow, etc.
    normals::Vector{Normal} # mean and stdevs for standardization
    mem::CPAMemory
end
function GraphFeatureTemplate(form::Int, normals::Vector{Normal})
    values = Array(Float64, length(normals))
    mem = CPAMemory()
    GraphFeatureTemplate(form, values, normals, mem)
end


immutable GraphFeatureInstance
    form::Int
    index::Int # for discrete values
    exponents::Vector{Float64} # exponents for each value
end
GraphFeatureInstance(form::Int, index::Int) = GraphFeatureInstance(form, index, Float64[])
GraphFeatureInstance(form::Int, exponents::Vector{Float64}) = GraphFeatureInstance(form, 0, exponents)

_standardize(v::Float64, μ::Float64, σ::Float64) = (v-μ)/σ
_standardize(v::Float64, normal::Normal) = (v-normal.μ)/normal.σ
function _set_and_standardize!(template::GraphFeatureTemplate, v::Float64, i::Int)
    template.values[i] = _standardize(v, template.normals[i])
    template
end
function extract!(
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
    else
    false
end
function uses_t(instance::GraphFeatureInstance)
    if instance.form == FeatureForms.ROAD
        return instance.exponents[1] > 0
    elseif instance.form == FeatureForms.NEIGHBOR
        return true
    else
    false
end
function uses_v(instance::GraphFeatureInstance)
    if instance.form == FeatureForms.ROAD
        return instance.exponents[2] > 0
    elseif instance.form == FeatureForms.ROAD
        return instance.exponents[2] > 0
    elseif instance.form == FeatureForms.NEIGHBOR
        return true
    else
    false
end
function uses_ϕ(instance::GraphFeatureInstance)
    if instance.form == FeatureForms.ROAD
        return instance.exponents[3] > 0
    elseif instance.form == FeatureForms.NEIGHBOR
        return true
    else
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
const FEATURE_TEMPLATE_NEIGHBOR = GraphFeatureTemplate(FeatureForms.NEIGHBOR, Normal[])