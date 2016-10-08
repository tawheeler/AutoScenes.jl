addprocs(4)

using AutomotiveDrivingModels
@everywhere using AutoScenes
using AutoViz
using NGSIM

trajdata = load_trajdata(4)
scene = get!(Scene(), trajdata, 4000)

const REGIONS = Dict{String, SubSceneExtractParams}(
  "101A" => SubSceneExtractParams(VecSE2(1966395.000, 570900.000, deg2rad(138)), 100.0, 41.0),
  "101B" => SubSceneExtractParams(VecSE2(1966501.608, 570804.793, 2.425), 100.0, 25.0),
  "80A"  => SubSceneExtractParams(VecSE2(1841794.208, 650650.640, deg2rad(-81)), 100.0, 32.0),
  "80B"  => SubSceneExtractParams(VecSE2(1841849.208, 650320.640, deg2rad(-81)), 100.0, 38.0),
  )

factors = create_shared_factors()
dset = SceneStructureDataset(factors)

for trajdata_index in 1 : 6
    trajdata = load_trajdata(trajdata_index)
    frames = 1:40:nframes(trajdata)
    if startswith(splitdir(NGSIM.TRAJDATA_PATHS[trajdata_index])[2], "trajdata_i101")
        append!(dset, pull_scene_dataset(trajdata, REGIONS["101A"], frames=frames))
        append!(dset, pull_scene_dataset(trajdata, REGIONS["101B"], frames=frames))
    else
        append!(dset, pull_scene_dataset(trajdata, REGIONS["80A"], frames=frames))
        append!(dset, pull_scene_dataset(trajdata, REGIONS["80B"], frames=frames))
    end
end

dat = PseudolikelihoodPrealloc(50)
params = GradientStepParams(BatchSampler(dset))
params.grad_params.n_samples_monte_carlo_integration = 20
params.grad_params.n_samples_monte_carlo_pseudolikelihood = 20
params.factor_weight_min = -30.0
params.factor_weight_max =  20.0
params.gradient_min = -5.0
params.gradient_max = 5.0

learning_rate = 0.5
learning_rate_decay = 0.99
batch_size = 2
batch_size_increase = 2
t_start = now()

iter = 0
while iter < typemax(Int)
    iter += 1

    params.learning_rate = learning_rate
    params.batch_size = batch_size
    parallel_step!(params)

    println("iter: ", iter)
    println("time: ", now() - t_start)
    if mod(iter, 5) == 1
        println("plogl: ", calc_pseudolikelihood(dset, dat=dat, scene=params.grad_params.scene, rec=params.grad_params.rec))
    end
    println("learning rate: ", learning_rate)
    println("batch_size:    ", batch_size)
    println("n_samples:     ", )
    println("weights: ")
    for ϕ in dset.factors
        println(ϕ.template.form, "  ", ϕ.weights)
    end
    println("")

    learning_rate *= learning_rate_decay
    batch_size = min(length(dset), batch_size + batch_size_increase)
end