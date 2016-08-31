type SharedFactor
    template::GraphFeatureTemplate
    instances::Vector{GraphFeatureInstance} # must all share the same form
    weights::Vector{Float64}                # one weight for each instance

    function SharedFactor(
        template::GraphFeatureTemplate,
        instances::Vector{GraphFeatureInstance},
        weights::Vector{Float64} = rand(Float64, length(instances)),
        )

        new(template, instances, weights)
    end
end

function extract!(
    factor::SharedFactor,
    scene::Scene,
    roadway::Roadway,
    vehicle_indeces::Vector{Int},
    )

    extract!(factor.template, scene, roadway, vehicle_indeces)
end
function evaluate_dot(factor::SharedFactor)
    # NOTE: extract! must have already been called
    retval = 0.0
    for i in 1 : length(instances)
        v = evaluate(factor.template, factor.instance)
        retval += factor.weights[i]*v
    end
    retval
end
evaluate_exp(factor::SharedFactor) = exp(evaluate_dot(factor))

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
    retval[FeatureForms.ROAD] = SharedFactor(FEATURE_TEMPLATE_ROAD, road_instances)

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
    retval[FeatureForms.FOLLOW] = SharedFactor(FEATURE_TEMPLATE_FOLLOW, follow_instances)

    # Neighbor
    neighbor_instances = GraphFeatureInstance[]
    for i in 1 : 5
        push!(neighbor_instances, GraphFeatureInstance(FeatureForms.NEIGHBOR, i))
    end
    retval[FeatureForms.NEIGHBOR] = SharedFactor(FEATURE_TEMPLATE_NEIGHBOR, neighbor_instances)

    retval
end