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

function Base.write(io::IO, factors::Vector{SharedFactor})
    println(io, "SHARED FACTORS")

    println(io, length(factors))
    for factor in factors
        # GraphFeatureTemplate
        template = factor.template
        println(io, template.form)
        println(io, length(template.normals))
        for N in template.normals
            @printf(io, "%.8e %.8e\n", N.μ, N.σ)
        end

        # GraphFeatureInstance
        println(io, length(factor.instances))
        for instance in factor.instances
            print(io, instance.index)
            for v in instance.exponents
                print(io, " ", v)
            end
            print(io, "\n")
        end

        # weights
        for (i,v) in enumerate(factor.weights)
            @printf(io, "%.8e", v)
            if i < length(factor.weights)
                print(io, " ")
            end
        end
        print(io, "\n")
    end
end
function Base.read(io::IO, ::Type{Vector{SharedFactor}})
    lines = readlines(io)
    line_index = 1
    if contains(lines[line_index], "SHARED FACTORS")
        line_index += 1
    end

    function advance!()
        line = strip(lines[line_index])
        line_index += 1
        line
    end

    N = parse(Int, advance!())
    factors = Array(SharedFactor, N)
    for i in 1 : N
        # GraphFeatureTemplate
        form = parse(Int, advance!())
        n_normals = parse(Int, advance!())
        normals = Array(Normal{Float64}, n_normals)
        for j in 1 : n_normals
            tokens = split(advance!(), ' ')
            μ = parse(Float64, tokens[1])
            σ = parse(Float64, tokens[2])
            normals[j] = Normal(μ, σ)
        end
        template = GraphFeatureTemplate(form, normals)

        # GraphFeatureInstance
        n_instances = parse(Int, advance!())
        instances = Array(GraphFeatureInstance, n_instances)
        for j in 1 : n_instances
            vals = Int[parse(Int, token) for token in split(advance!(), ' ')]
            index = vals[1]
            exponents = length(vals) > 1 ? vals[2:end] : Int[]
            instances[j] = GraphFeatureInstance(form, index, exponents)
        end

        # weights
        weights = Float64[parse(Float64, token) for token in split(advance!(), ' ')]
        @assert(length(weights) == n_instances)

        factors[i] = SharedFactor(template, instances, weights)
    end

    factors
end
save_factors(factors::Vector{SharedFactor}, filepath::String) = open(io->write(io, factors), filepath, "w")
load_factors(filepath::String) = open(io->read(io, Vector{SharedFactor}), filepath, "r")