type SharedFactor
    template::GraphFeatureTemplate
    instances::Vector{GraphFeatureInstance} # must all share the same form
    weights::Vector{Float64}                # one weight for each instance

    function SharedFactor(
        template::GraphFeatureTemplate,
        instances::Vector{GraphFeatureInstance},
        weights::Vector{Float64} = 0.001*rand(Float64, length(instances)),
        )

        new(template, instances, weights)
    end
end

function AutomotiveDrivingModels.extract!(
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
    for i in 1 : length(factor.instances)
        v = evaluate(factor.template, factor.instances[i])
        retval += factor.weights[i]*v
    end
    retval
end
evaluate_exp(factor::SharedFactor) = exp(evaluate_dot(factor))