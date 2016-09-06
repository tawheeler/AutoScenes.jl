type PrintParams
    to_stdout::Bool      # whether to print to STDOUT
    to_stdout_every::Int # [batches]
    to_file::AbstractString
    to_file_every::Int   # [batches]

    function PrintParams(;
        to_stdout::Bool=false,
        to_stdout_every::Int=5,
        to_file::AbstractString = "",
        to_file_every::Int=5,
        )

        retval = new()
        retval.to_stdout = to_stdout
        retval.to_stdout_every = to_stdout_every
        retval.to_file = to_file
        retval.to_file_every = to_file_every
        retval
    end
end
function pprint(params::PrintParams, tup::Tuple)
    if params.to_stdout
        print(tup...)
    end
    if !isempty(params.to_file)
        open(params.to_file, "a") do fout
            print(fout, tup...)
        end
    end
    nothing
end
function pprintln(params::PrintParams, tup::Tuple)
    if params.print_to_stdout
        println(tup...)
    end
    if !isempty(params.print_to_file)
        open(params.print_to_file, "a") do fout
            println(fout, tup...)
        end
    end
    nothing
end

type StochasticGradientAscentParams

    batch_size::Int
    batch_size_multiplier::Float64
    niter::Int

    learning_rate::Float64
    learning_rate_multiplier::Float64
    momentum_param::Float64      # ∈ [0,1), v ← γv + α∇ₜ(batch), γ typically starts at 0.5 and then is set to 0.9 later
    regularization::Float64

    batch_rng::AbstractRNG
    monte_carlo_pseudolikelihood_rng::AbstractRNG
    n_samples_monte_carlo_integration::Int
    n_samples_monte_carlo_pseudolikelihood::Int

    factor_weight_min::Float64
    factor_weight_max::Float64
    gradient_min::Float64
    gradient_max::Float64

    save_every::Int            # [batches], set to -1 to never save
    save_dir::AbstractString   # directory in which to store checkpointed models
    same_name::AbstractString  # name for saved models -> ex 'model' -> model_001.jld

    print_params::PrintParams

    function StochasticGradientAscentParams(;
        batch_size::Int = 10,
        batch_size_multiplier::Float64 = 1.1,
        niter::Int = 5,
        learning_rate::Float64 = 1.0,
        learning_rate_multiplier::Float64 = 0.97,
        momentum_param::Float64 = 0.5, # ∈ [0,1), v ← γv + α∇ₜ(batch), γ typically starts at 0.5 and then is set to 0.9 later
        regularization::Float64 = 0.001,
        batch_rng::AbstractRNG = Base.GLOBAL_RNG,
        monte_carlo_pseudolikelihood_rng::AbstractRNG = Base.GLOBAL_RNG,
        n_samples_monte_carlo_integration::Int = 10,
        n_samples_monte_carlo_pseudolikelihood::Int = 10,
        factor_weight_min::Float64 = -8.0,
        factor_weight_max::Float64 =  1.0,
        gradient_min::Float64 = -0.1,
        gradient_max::Float64 = 0.1,
        save_every::Int = -1,           # [batches], set to -1 to never save
        save_dir::AbstractString = "",   # directory in which to store checkpointed models
        same_name::AbstractString = "full_scene", # name for saved models -> ex 'model' -> model_001.jld
        print_params::PrintParams = PrintParams(),
        )

        retval = new()
        retval.batch_size = batch_size
        retval.batch_size_multiplier = batch_size_multiplier
        retval.niter = niter
        retval.learning_rate = learning_rate
        retval.learning_rate_multiplier = learning_rate_multiplier
        retval.momentum_param = momentum_param
        retval.regularization = regularization
        retval.batch_rng = batch_rng
        retval.monte_carlo_pseudolikelihood_rng = monte_carlo_pseudolikelihood_rng
        retval.n_samples_monte_carlo_integration = n_samples_monte_carlo_integration
        retval.n_samples_monte_carlo_pseudolikelihood = n_samples_monte_carlo_pseudolikelihood
        retval.factor_weight_min = factor_weight_min
        retval.factor_weight_max = factor_weight_max
        retval.gradient_min = gradient_min
        retval.gradient_max = gradient_max
        retval.save_every = save_every
        retval.save_dir = save_dir
        retval.same_name = same_name
        retval.print_params = print_params
        retval
    end
end
function Base.show(io::IO, params::StochasticGradientAscentParams)
    println(io, "StochasticGradientAscentParams")
    println(io, "\tbatch_size: ", params.batch_size)
    println(io, "\tbatch_size_multiplier: ", params.batch_size_multiplier)
    println(io, "\niter: ", params.niter)
    println(io, "\tlearning_rate: ", params.learning_rate)
    println(io, "\tlearning_rate_multiplier: ", params.learning_rate_multiplier)
    println(io, "\tmomentum_param: ", params.momentum_param)
    println(io, "\tregularization: ", params.regularization)
    println(io, "\tn_samples_monte_carlo_integration: ", params.n_samples_monte_carlo_integration)
    println(io, "\tn_samples_monte_carlo_pseudolikelihood: ", params.n_samples_monte_carlo_pseudolikelihood)
    println(io, "\tfactor_weight_min: ", params.factor_weight_min)
    println(io, "\tfactor_weight_max: ", params.factor_weight_max)
    println(io, "\tgradient_min: ", params.gradient_min)
    println(io, "\tgradient_max: ", params.gradient_max)
    println(io, "\tsave_every: ", params.save_every)
    println(io, "\tsave_dir: ", params.save_dir)
    println(io, "\tsame_name: ", params.same_name)
    println(io, "\tprint_params: ", params.print_params)
end

function alloc_grad_velocities(dset::SceneStructureDataset)
    retval = Array(Vector{Float64}, length(dset.factors))
    for (i, ϕ) in enumerate(dset.factors)
        retval[i] = Array(Float64, length(ϕ.instances))
    end
    retval
end
function step!(
    dset::SceneStructureDataset,
    params::StochasticGradientAscentParams,
    grad_velocitities::Vector{Vector{Float64}},
    learning_rate::Float64,
    batch_size::Int,
    scene::Scene,
    rec::SceneRecord,
    )

    α = learning_rate
    γ = params.momentum_param

    # apply the momentum speed update
    for (factor_index, ϕ) in enumerate(dset.factors)
        grad_vel_arr = grad_velocitities[factor_index]

        for feature_index in 1 : length(grad_vel_arr)
            gradient = calc_pseudolikelihood_gradient(factor_index, feature_index, dset, batch_size,
                                                      params.n_samples_monte_carlo_integration, params.regularization,
                                                      params.monte_carlo_pseudolikelihood_rng, scene, rec)
            @assert(!isnan(gradient))
            @assert(!isinf(gradient))
            grad_vel = grad_vel_arr[feature_index]
            @assert(!isnan(grad_vel))
            @assert(!isinf(grad_vel))
            grad_vel_arr[feature_index] = γ*grad_vel + α*gradient
        end
    end

    # apply gradient update
    for (factor_index, ϕ) in enumerate(dset.factors)
        grad_vel_arr = grad_velocitities[factor_index]

        for i in 1 : length(ϕ.weights)
            gradient = clamp(grad_vel_arr[i], params.gradient_min, params.gradient_max)
            ϕ.weights[i] = clamp(ϕ.weights[i] + gradient, params.factor_weight_min, params.factor_weight_max)
            @assert(!isnan(ϕ.weights[i]))
        end
    end

    dset
end

function stochastic_gradient_ascent!(dset::SceneStructureDataset, params::StochasticGradientAscentParams;
    grad_velocities::Vector{Vector{Float64}} = alloc_grad_velocities(dset),
    scene::Scene = Scene(),
    rec::SceneRecord = SceneRecord(1, 0.1),
    )

    α = params.learning_rate
    batch_size = params.batch_size

    # run gradient ascent
    iter = 0
    while iter < params.niter
        iter += 1

        # gradient step
        step!(dset, params, grad_velocities, α, batch_size, scene, rec)

        # learning rate mult
        α *= params.learning_rate_multiplier

        # batch size mult
        batch_size = round(Int, batch_size * params.batch_size_multiplier)
    end

    dset
end