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

##########################

type GradientParams
    batch_sampler::BatchSampler
    monte_carlo_pseudolikelihood_rng::AbstractRNG
    n_samples_monte_carlo_integration::Int
    n_samples_monte_carlo_pseudolikelihood::Int
    regularization::Float64

    scene::Scene
    rec::SceneRecord

    function GradientParams(batch_sampler::BatchSampler;
        monte_carlo_pseudolikelihood_rng::AbstractRNG = Base.GLOBAL_RNG,
        n_samples_monte_carlo_integration::Int = 10,
        n_samples_monte_carlo_pseudolikelihood::Int = 10,
        regularization::Float64 = 0.001,
        scene::Scene = Scene(),
        rec::SceneRecord = SceneRecord(1,NaN),
        )

        retval = new()
        retval.batch_sampler = batch_sampler
        retval.monte_carlo_pseudolikelihood_rng = monte_carlo_pseudolikelihood_rng
        retval.n_samples_monte_carlo_integration = n_samples_monte_carlo_integration
        retval.n_samples_monte_carlo_pseudolikelihood = n_samples_monte_carlo_pseudolikelihood
        retval.regularization = regularization
        retval.scene = scene
        retval.rec = rec
        retval
    end
end
function calc_pseudolikelihood_gradient(form::Int, feature_index::Int, batch_size::Int, params::GradientParams)

    calc_pseudolikelihood_gradient(form, feature_index, params.batch_sampler, batch_size,
                                   params.n_samples_monte_carlo_integration, params.regularization,
                                   params.monte_carlo_pseudolikelihood_rng, params.scene, params.rec)
end

function alloc_grad_velocities(dset::SceneStructureDataset)
    retval = Array(Vector{Float64}, length(dset.factors))
    for (i, ϕ) in enumerate(dset.factors)
        retval[i] = zeros(Float64, length(ϕ.instances))
    end
    retval
end
type GradientStepParams
    batch_size::Int
    learning_rate::Float64
    momentum_param::Float64 # ∈ [0,1), v ← γv + α∇ₜ(batch), γ typically starts at 0.5 and then is set to 0.9 later
    grad_params::GradientParams
    grad_velocitities::Vector{Vector{Float64}}

    gradient_min::Float64
    gradient_max::Float64
    factor_weight_min::Float64
    factor_weight_max::Float64

    function GradientStepParams(batch_sampler::BatchSampler;
        batch_size::Int = 10,
        learning_rate::Float64 = 0.1,
        momentum_param::Float64 = 0.5, # ∈ [0,1), v ← γv + α∇ₜ(batch), γ typically starts at 0.5 and then is set to 0.9 later
        grad_params::GradientParams = GradientParams(batch_sampler),
        gradient_min::Float64 = -1.0,
        gradient_max::Float64 = 1.0,
        factor_weight_min::Float64 = -8.0,
        factor_weight_max::Float64 =  1.0, # large factors lead to exploding exp() values
        )

        retval = new()
        retval.batch_size = batch_size
        retval.learning_rate = learning_rate
        retval.momentum_param = momentum_param
        retval.grad_params = grad_params
        retval.grad_velocitities = alloc_grad_velocities(grad_params.batch_sampler.dset)
        retval.gradient_min = gradient_min
        retval.gradient_max = gradient_max
        retval.factor_weight_min = factor_weight_min
        retval.factor_weight_max = factor_weight_max
        retval
    end
end

type StochasticGradientAscentParams

    n_epochs::Int # quit once this many epochs have been done
    n_samples::Int # or quit once this many samples have been done

    batch_size_multiplier::Float64
    learning_rate_multiplier::Float64
    momentum_param_multiplier::Float64

    step_params::GradientStepParams

    # function StochasticGradientAscentParams(;
    #     n_epochs::Int = 1,
    #     n_samples::Int = 1000,
    #     batch_size_multiplier::Float64 = 1.0,
    #     learning_rate_multiplier::Float64 = 1.0,
    #     momentum_param_multiplier::Float64 = 1.0,
    #     step_params::GradientStepParams = GradientStepParams(),
    #     )

    #     retval = new()
    #     retval.n_epochs = n_epochs
    #     retval.n_samples = n_samples
    #     retval.batch_size_multiplier = batch_size_multiplier
    #     retval.learning_rate_multiplier = learning_rate_multiplier
    #     retval.momentum_param_multiplier = momentum_param_multiplier
    #     retval.step_params = step_params
    #     retval
    # end
end


function step!(params::GradientStepParams)

    dset = params.grad_params.batch_sampler.dset
    α = params.learning_rate
    γ = params.momentum_param

    # apply the momentum speed update
    for (factor_index, ϕ) in enumerate(dset.factors)
        grad_vel_arr = params.grad_velocitities[factor_index]

        for feature_index in 1 : length(grad_vel_arr)
            gradient = calc_pseudolikelihood_gradient(factor_index, feature_index, params.batch_size, params.grad_params)
            grad_vel = grad_vel_arr[feature_index]
            grad_vel_arr[feature_index] = γ*grad_vel + α*gradient
        end
    end

    # apply gradient update
    for (factor_index, ϕ) in enumerate(dset.factors)
        grad_vel_arr = params.grad_velocitities[factor_index]

        print(factor_index, ": ")
        for i in 1 : length(ϕ.weights)
            @printf("%8.3f  ", grad_vel_arr[i])
            gradient = clamp(grad_vel_arr[i], params.gradient_min, params.gradient_max)
            ϕ.weights[i] = clamp(ϕ.weights[i] + gradient, params.factor_weight_min, params.factor_weight_max)
        end
        println("")
    end

    dset
end

function stochastic_gradient_ascent!(params::StochasticGradientAscentParams)

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