using Base.Test
using AutomotiveDrivingModels
using AutoScenes

roadway = gen_straight_roadway(1)
scene = Scene([
    Vehicle(VehicleState(VecSE2(100.0,0.0,0.0), roadway, 1.0), VehicleDef(1, AgentClass.CAR, 4.0, 2.0)),
    Vehicle(VehicleState(VecSE2(108.0,0.0,0.0), roadway, 2.0), VehicleDef(2, AgentClass.CAR, 4.0, 2.0)),
    Vehicle(VehicleState(VecSE2(116.0,0.0,0.0), roadway, 0.0), VehicleDef(3, AgentClass.CAR, 4.0, 2.0)),
    Vehicle(VehicleState(VecSE2( 92.0,0.0,0.0), roadway, 0.0), VehicleDef(4, AgentClass.CAR, 4.0, 2.0)),
])

lead_follow = LeadFollowRelationships(scene, roadway)
@test lead_follow.index_fore == [2,3,0,1]
@test lead_follow.index_rear == [4,1,2,0]

active = get_active_vehicles(lead_follow)
@test 1 ∈ active
@test 2 ∈ active
@test 3 ∉ active
@test 4 ∉ active

#####

function extract_basic_road_feature!(
    v::Vector{Float64},
    scene::Scene,
    roadway::Roadway,
    vehicle_indices::Tuple{Vararg{Int}},
    )

    vehicle_index = vehicle_indices[1]
    v[1] = scene[vehicle_index].state.v
    nothing
end
AutoScenes.uses_s{F<:typeof(extract_basic_road_feature!)}(ϕ::LogLinearSharedFactor{F}) = false
AutoScenes.uses_t{F<:typeof(extract_basic_road_feature!)}(ϕ::LogLinearSharedFactor{F}) = false
AutoScenes.uses_v{F<:typeof(extract_basic_road_feature!)}(ϕ::LogLinearSharedFactor{F}) = true
AutoScenes.uses_ϕ{F<:typeof(extract_basic_road_feature!)}(ϕ::LogLinearSharedFactor{F}) = false
function AutoScenes.assign_factors{F <: typeof(extract_basic_road_feature!)}(
    ϕ::LogLinearSharedFactor{F},
    scene::Scene,
    roadway::Roadway,
    active_vehicles::Set{Int},
    lead_follow::LeadFollowRelationships,
    )

    retval = Array(Tuple{Vararg{Int}}, length(active_vehicles))
    for (i,vehicle_index) in enumerate(active_vehicles)
        retval[i] = (vehicle_index,)
    end
    return retval
end

ϕ_road = LogLinearSharedFactor(extract_basic_road_feature!, [1.0])
@test !uses_s(ϕ_road)
@test !uses_t(ϕ_road)
@test  uses_v(ϕ_road)
@test !uses_ϕ(ϕ_road)

factor_assignments = assign_factors(ϕ_road, scene, roadway, get_active_vehicles(lead_follow), lead_follow)
@test length(factor_assignments) == 2
@test (1,) ∈ factor_assignments
@test (2,) ∈ factor_assignments

structure = SceneStructure(scene, roadway, (ϕ_road,))
@test structure.lead_follow == lead_follow
@test 1 ∈ structure.active_vehicles
@test 2 ∈ structure.active_vehicles
@test 3 ∉ structure.active_vehicles
@test 4 ∉ structure.active_vehicles
@test structure.factor_assignments[ϕ_road] == factor_assignments

extract!(ϕ_road, scene, roadway, (1,)); @test isapprox(ϕ_road.v[1], 1.0)
extract!(ϕ_road, scene, roadway, (2,)); @test isapprox(ϕ_road.v[1], 2.0)
@test isapprox(dot(ϕ_road), 2.0*1.0)
@test isapprox(exp(ϕ_road), exp(2.0*1.0))

###

function extract_basic_follow_feature!(
    v::Vector{Float64},
    scene::Scene,
    roadway::Roadway,
    vehicle_indices::Tuple{Vararg{Int}},
    )

    veh_rear = scene[vehicle_indices[1]]
    veh_fore = scene[vehicle_indices[2]]

    println()

    v[1] = veh_fore.state.v - veh_rear.state.v
    nothing
end
AutoScenes.uses_s{F<:typeof(extract_basic_follow_feature!)}(ϕ::LogLinearSharedFactor{F}) = false
AutoScenes.uses_t{F<:typeof(extract_basic_follow_feature!)}(ϕ::LogLinearSharedFactor{F}) = false
AutoScenes.uses_v{F<:typeof(extract_basic_follow_feature!)}(ϕ::LogLinearSharedFactor{F}) = true
AutoScenes.uses_ϕ{F<:typeof(extract_basic_follow_feature!)}(ϕ::LogLinearSharedFactor{F}) = false
function AutoScenes.assign_factors{F <: typeof(extract_basic_follow_feature!)}(
    ϕ::LogLinearSharedFactor{F},
    scene::Scene,
    roadway::Roadway,
    active_vehicles::Set{Int},
    lead_follow::LeadFollowRelationships,
    )

    retval = Tuple{Vararg{Int}}[]

    for (vehicle_index, index_fore) in enumerate(lead_follow.index_fore)
        if index_fore != 0 && (vehicle_index ∈ active_vehicles || index_fore ∈ active_vehicles)

            @assert index_fore != vehicle_index
            push!(retval, (vehicle_index, index_fore))
        end
    end

    return retval
end

ϕ_follow = LogLinearSharedFactor(extract_basic_follow_feature!, [0.5])
@test !uses_s(ϕ_follow)
@test !uses_t(ϕ_follow)
@test  uses_v(ϕ_follow)
@test !uses_ϕ(ϕ_follow)

factor_assignments_follow = assign_factors(ϕ_follow, scene, roadway, get_active_vehicles(lead_follow), lead_follow)
@test length(factor_assignments_follow) == 3
@test (1,2) ∈ factor_assignments_follow
@test (2,3) ∈ factor_assignments_follow
@test (4,1) ∈ factor_assignments_follow

structure = SceneStructure(scene, roadway, (ϕ_road,ϕ_follow))
@test structure.lead_follow == lead_follow
@test 1 ∈ structure.active_vehicles
@test 2 ∈ structure.active_vehicles
@test 3 ∉ structure.active_vehicles
@test 4 ∉ structure.active_vehicles
@test structure.factor_assignments[ϕ_road] == factor_assignments
@test structure.factor_assignments[ϕ_follow] == factor_assignments_follow

extract!(ϕ_follow, scene, roadway, (1,2)); @test isapprox(ϕ_follow.v[1], 2.0 - 1.0)
@test isapprox(dot(ϕ_follow), (2.0-1.0)*0.5)
@test isapprox(exp(ϕ_follow), exp((2.0-1.0)*0.5))

# structure = SceneStructure([
#     FactorAssignment(FeatureForms.ROAD, [1]),
#     FactorAssignment(FeatureForms.ROAD, [2]),
#     FactorAssignment(FeatureForms.FOLLOW, [1,2]),
#     ], Set{Int}([1,2]), LeadFollowRelationships([2,3,0,1], [4,1,2,0]))

# vehdefs = Dict{Int, VehicleDef}()
# vehdefs[1] = scene[1].def
# vehdefs[2] = scene[2].def
# vehdefs[3] = scene[3].def
# vehdefs[4] = scene[4].def

# states = [TrajdataState(1, scene[1].state),
#           TrajdataState(2, scene[2].state),
#           TrajdataState(3, scene[3].state),
#           TrajdataState(4, scene[4].state),
# ]

# frames = [TrajdataFrame(1,4,0.0)]

# dset = SceneStructureDataset(
#     [Trajdata(roadway, vehdefs, states, frames)],
#     [AutoScenes.SceneSource(1, 1)],
#     [structure],
#     create_shared_factors(),
# )

# ϕ_road = dset.factors[1] # shared factor
# vehicle_indeces = [1]
# extract!(ϕ_road, scene, roadway, vehicle_indeces)
# @test ϕ_road.template.values == [1.0]
# @test evaluate(ϕ_road.template, ϕ_road.instances[1]) == 1.0
# @test evaluate(ϕ_road.template, ϕ_road.instances[2]) == 1.0^2
# @test evaluate_dot(ϕ_road) == 1.1

# vehicle_indeces = [2]
# extract!(ϕ_road, scene, roadway, vehicle_indeces)
# @test ϕ_road.template.values == [2.0]
# @test evaluate(ϕ_road.template, ϕ_road.instances[1]) == 2.0
# @test evaluate(ϕ_road.template, ϕ_road.instances[2]) == 2.0^2
# @test evaluate_dot(ϕ_road) == 2.4

# ϕ_follow = dset.factors[2] # shared factor
# vehicle_indeces = [1,2]
# extract!(ϕ_follow, scene, roadway, vehicle_indeces)
# @test ϕ_follow.template.values == [1.0]
# @test evaluate(ϕ_follow.template, ϕ_follow.instances[1]) == 1.0
# @test evaluate_dot(ϕ_follow) == 1.0

# @test evaluate_dot!(structure, dset.factors, scene, roadway, SceneRecord(1,0.1)) == 4.5

# plog = calc_pseudolikelihood(dset, dat = PseudolikelihoodPrealloc(100000))
# @test isapprox(plog, 0.1184, atol=0.001)

# sampler = BatchSampler(dset)
# grad = calc_pseudolikelihood_gradient(FeatureForms.ROAD, 1, sampler, 1, 100000, 0.0)
# @test isapprox(grad, 0.35208, atol=0.005)

# params = GradientStepParams(sampler, batch_size=1)
# params.grad_params.n_samples_monte_carlo_integration=10000
# params.grad_params.n_samples_monte_carlo_pseudolikelihood=10000
# step!(params)

# plog2 = calc_pseudolikelihood(dset, dat = PseudolikelihoodPrealloc(100000))
# @test plog2 > plog  # it increases!