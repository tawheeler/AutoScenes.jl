using Base.Test
using AutomotiveDrivingModels


# find the package and use it with basic features

line_orig = ""
line_index_orig = 0
filepath = Pkg.dir("AutoScenes", "src", "AutoScenes.jl")
lines = open(readlines, filepath)
for (line_index, line) in enumerate(lines)
    if contains(line, "include(Pkg.dir(\"AutoScenes\", \"src\", \"features\", ")
        line_orig = line
        lines[line_index] = "include(Pkg.dir(\"AutoScenes\", \"src\", \"features\", \"basic.jl\"))\n" # put in basic
        line_index_orig = line_index
        break
    end
end

@assert line_index_orig != 0

io = open(filepath, "w")
for line in lines
    print(io, line)
end
close(io)

###############

using AutoScenes

roadway = gen_straight_roadway(1)
scene = Scene([
    Vehicle(VehicleState(VecSE2(100.0,0.0,0.0), roadway, 1.0), VehicleDef(1, AgentClass.CAR, 4.0, 2.0)),
    Vehicle(VehicleState(VecSE2(108.0,0.0,0.0), roadway, 2.0), VehicleDef(2, AgentClass.CAR, 4.0, 2.0)),
    Vehicle(VehicleState(VecSE2(116.0,0.0,0.0), roadway, 0.0), VehicleDef(3, AgentClass.CAR, 4.0, 2.0)),
    Vehicle(VehicleState(VecSE2( 92.0,0.0,0.0), roadway, 0.0), VehicleDef(4, AgentClass.CAR, 4.0, 2.0)),
])
structure = SceneStructure([
    FactorAssignment(FeatureForms.ROAD, [1]),
    FactorAssignment(FeatureForms.ROAD, [2]),
    FactorAssignment(FeatureForms.FOLLOW, [1,2]),
    ], Set{Int}([1,2]), LeadFollowRelationships([2,3,0,1], [4,1,2,0]))

vehdefs = Dict{Int, VehicleDef}()
vehdefs[1] = scene[1].def
vehdefs[2] = scene[2].def
vehdefs[3] = scene[3].def
vehdefs[4] = scene[4].def

states = [TrajdataState(1, scene[1].state),
          TrajdataState(2, scene[2].state),
          TrajdataState(3, scene[3].state),
          TrajdataState(4, scene[4].state),
]

frames = [TrajdataFrame(1,4,0.0)]

dset = SceneStructureDataset(
    [Trajdata(roadway, vehdefs, states, frames)],
    [AutoScenes.SceneSource(1, 1)],
    [structure],
    create_shared_factors(),
)

ϕ_road = dset.factors[1] # shared factor
vehicle_indeces = [1]
extract!(ϕ_road, scene, roadway, vehicle_indeces)
@test ϕ_road.template.values == [1.0]
@test evaluate(ϕ_road.template, ϕ_road.instances[1]) == 1.0
@test evaluate(ϕ_road.template, ϕ_road.instances[2]) == 1.0^2
@test evaluate_dot(ϕ_road) == 1.1

vehicle_indeces = [2]
extract!(ϕ_road, scene, roadway, vehicle_indeces)
@test ϕ_road.template.values == [2.0]
@test evaluate(ϕ_road.template, ϕ_road.instances[1]) == 2.0
@test evaluate(ϕ_road.template, ϕ_road.instances[2]) == 2.0^2
@test evaluate_dot(ϕ_road) == 2.4

ϕ_follow = dset.factors[2] # shared factor
vehicle_indeces = [1,2]
extract!(ϕ_follow, scene, roadway, vehicle_indeces)
@test ϕ_follow.template.values == [1.0]
@test evaluate(ϕ_follow.template, ϕ_follow.instances[1]) == 1.0
@test evaluate_dot(ϕ_follow) == 1.0

@test evaluate_dot!(structure, dset.factors, scene, roadway, SceneRecord(1,0.1)) == 4.5

plog = calc_pseudolikelihood(dset, dat = PseudolikelihoodPrealloc(100000))
@test isapprox(plog, 0.1184, atol=0.001)

sampler = BatchSampler(dset)
grad = calc_pseudolikelihood_gradient(FeatureForms.ROAD, 1, sampler, 1, 100000, 0.0)
@test isapprox(grad, 0.35208, atol=0.005)

params = GradientStepParams(sampler, batch_size=1)
params.grad_params.n_samples_monte_carlo_integration=10000
params.grad_params.n_samples_monte_carlo_pseudolikelihood=10000
step!(params)

plog2 = calc_pseudolikelihood(dset, dat = PseudolikelihoodPrealloc(100000))
@test plog2 > plog  # it increases!

###############

lines[line_index_orig] = line_orig

io = open(filepath, "w")
for line in lines
    print(io, line)
end
close(io)