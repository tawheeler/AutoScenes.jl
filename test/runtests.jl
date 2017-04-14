using Base.Test
using AutomotiveDrivingModels
using AutoScenes

def = VehicleDef(AgentClass.CAR, 4.0, 2.0)
roadway = StraightRoadway(200.0)
scene = MobiusScene([
    MobiusVehicle(State1D(10.0,10.0), def, 1),
    MobiusVehicle(State1D(18.0,12.0), def, 2),
    MobiusVehicle(State1D(26.0,10.0), def, 3),
    MobiusVehicle(State1D(34.0, 8.0), def, 4),
])

lead_follow = LeadFollowRelationships(scene, roadway)
@test lead_follow.index_fore == [2,3,4,1]
@test lead_follow.index_rear == [4,1,2,3]

active = get_active_vehicles(lead_follow)
@test 1 ∈ active
@test 2 ∈ active
@test 3 ∈ active
@test 4 ∈ active

#####

function speed{R}(
    scene::Union{Scene, MobiusScene},
    roadway::R,
    vehicle_indices::Tuple{Int},
    )

    vehicle_index = vehicle_indices[1]
    return scene[vehicle_index].state.v
end
AutoScenes.uses_s{F<:typeof(speed)}(f::F) = false
AutoScenes.uses_t{F<:typeof(speed)}(f::F) = false
AutoScenes.uses_v{F<:typeof(speed)}(f::F) = true
AutoScenes.uses_ϕ{F<:typeof(speed)}(f::F) = false
function AutoScenes.assign_feature{F <: typeof(speed), R}(
    f::F,
    scene::Union{Scene, MobiusScene},
    roadway::R,
    active_vehicles::Set{Int},
    lead_follow::LeadFollowRelationships,
    )

    retval = Array(Tuple{Vararg{Int}}, length(active_vehicles))
    for (i,vehicle_index) in enumerate(active_vehicles)
        retval[i] = (vehicle_index,)
    end
    return retval
end

f₁ = speed
@test !uses_s(f₁)
@test !uses_t(f₁)
@test  uses_v(f₁)
@test !uses_ϕ(f₁)

assignments₁ = assign_feature(f₁, scene, roadway, get_active_vehicles(lead_follow), lead_follow)
@test length(assignments₁) == 4
@test (1,) ∈ assignments₁
@test (2,) ∈ assignments₁
@test (3,) ∈ assignments₁
@test (4,) ∈ assignments₁

structure = SceneStructure(scene, roadway, (f₁,))
@test structure.lead_follow == lead_follow
@test 1 ∈ structure.active_vehicles
@test 2 ∈ structure.active_vehicles
@test 3 ∈ structure.active_vehicles
@test 4 ∈ structure.active_vehicles
@test structure.assignments[f₁] == assignments₁

@test isapprox(f₁(scene, roadway, (1,)), 10.0)
@test isapprox(f₁(scene, roadway, (2,)), 12.0)

###

function delta_speed{R}(
    scene::Union{Scene, MobiusScene},
    roadway::R,
    vehicle_indices::Tuple{Int, Int},
    )

    veh_rear = scene[vehicle_indices[1]]
    veh_fore = scene[vehicle_indices[2]]

    return veh_fore.state.v - veh_rear.state.v
end
AutoScenes.uses_s{F<:typeof(delta_speed)}(ϕ::F) = false
AutoScenes.uses_t{F<:typeof(delta_speed)}(ϕ::F) = false
AutoScenes.uses_v{F<:typeof(delta_speed)}(ϕ::F) = true
AutoScenes.uses_ϕ{F<:typeof(delta_speed)}(ϕ::F) = false
function AutoScenes.assign_feature{F <: typeof(delta_speed), R}(
    f::F,
    scene::Union{Scene, MobiusScene},
    roadway::R,
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

f₂ = delta_speed
@test !uses_s(f₂)
@test !uses_t(f₂)
@test  uses_v(f₂)
@test !uses_ϕ(f₂)

assignments₂ = assign_feature(f₂, scene, roadway, get_active_vehicles(lead_follow), lead_follow)
@test length(assignments₂) == 4
@test (1,2) ∈ assignments₂
@test (2,3) ∈ assignments₂
@test (3,4) ∈ assignments₂
@test (4,1) ∈ assignments₂

shared_features = (f₁,f₂)
structure = SceneStructure(scene, roadway, shared_features)
@test structure.lead_follow == lead_follow
@test 1 ∈ structure.active_vehicles
@test 2 ∈ structure.active_vehicles
@test 3 ∈ structure.active_vehicles
@test 4 ∈ structure.active_vehicles
@test structure.assignments[f₁] == assignments₁
@test structure.assignments[f₂] == assignments₂

@test isapprox(f₂(scene, roadway, (1,2)), 12.0 - 10.0)

#####

vehicle_index = 1
bounds = VehicleBounds(scene, roadway, structure.lead_follow, vehicle_index)

pseudo_expectation(f, scene, roadway, vehicle_indices, target)

pseudo_expectation(f, vars, j, roadway)
vars:
- variable instance
- variable bounds

speed, vars -> :v
delta speed, vars -> :v1, :v2


abstract Variable
type Speed <: Variable
end

function extract()

# Variable instance for a particular scene
type VarInstance
    value::Float64 # value for this particular car
    bounds::StateBounds # value for this particular car
end



###

function speed{R}(
    vars::Tuple{Speed},
    roadway::R,
    )

    return vars[1].value
end
function AutoScenes.assign_feature{F <: typeof(speed), R}(
    f::F,
    scene::Union{Scene, MobiusScene},
    roadway::R,
    active_vehicles::Set{Int},
    lead_follow::LeadFollowRelationships,
    )

    assignments = Tuple{Speed}[]

    for (vehicle_index, index_fore) in enumerate(lead_follow.index_fore)
        if index_fore != 0 && (vehicle_index ∈ active_vehicles || index_fore ∈ active_vehicles)

            @assert index_fore != vehicle_index
            push!(retval, (VarInstance(Speed, scene[vehicle_index]),))
        end
    end

    return retval
end
# scope{F<:typeof(speed)}(f::F) = {Speed}

function delta_speed{R}(
    vars::Tuple{Speed,Speed},
    roadway::R,
    )

    v_rear = vars[1].value
    v_fore = vars[2].value
    return v_fore - v_rear
end





# calc_pseudolikelihood_gradient(ϕ_road, scene, structure, roadway)

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