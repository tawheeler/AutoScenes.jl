using Base.Test
using AutomotiveDrivingModels
using AutoScenes

def = VehicleDef(AgentClass.CAR, 4.0, 2.0)
roadway = StraightRoadway(200.0)
scene = Scene1D([
    Vehicle1D(State1D(10.0,10.0), def, 1),
    Vehicle1D(State1D(18.0,12.0), def, 2),
    Vehicle1D(State1D(26.0,10.0), def, 3),
    Vehicle1D(State1D(34.0, 8.0), def, 4),
])

lead_follow = LeadFollowRelationships(scene, roadway)
@test lead_follow.index_fore == [2,3,4,1]
@test lead_follow.index_rear == [4,1,2,3]

#####

"""
    Vars(scene, roadway)

Construct a Vars based on the scene.
Each scene type must implement this function.
"""
function AutoScenes.Vars(scene::Scene1D, roadway::StraightRoadway)

    n = length(scene)
    vars = Vars(Array(Float64, 2n),
                Array(StateBounds, 2n),
                Array(Symbol, 2n),
                Array(Int, 2n))

    j = 0
    for (vehicle_index, veh) in enumerate(scene)

        rear = scene[lead_follow.index_rear[vehicle_index]]
        fore = scene[lead_follow.index_fore[vehicle_index]]

        Δs_fore = get_headway(veh, fore, roadway)
        Δs_rear = get_headway(rear, veh, roadway)

        # position
        j += 1
        vars.values[j] = veh.state.s
        vars.bounds[j] = StateBounds(-Δs_rear, Δs_fore)
        vars.symbols[j] = :s
        vars.vehicle_indices[j] = vehicle_index
        @assert vars.bounds[j].Δlo ≤ 0.0
        @assert vars.bounds[j].Δhi ≥ 0.0

        # speed
        j += 1
        vars.values[j] = veh.state.v
        vars.bounds[j] = StateBounds(0.0 - veh.state.v, 32.0 - veh.state.v)
        vars.symbols[j] = :v
        vars.vehicle_indices[j] = vehicle_index
        @assert vars.bounds[j].Δlo ≤ 0.0
        @assert vars.bounds[j].Δhi ≥ 0.0
    end

    return vars
end

vars = Vars(scene, roadway)
@test vars.values ≈ [10.0, 10.0, 18.0, 12.0, 26.0, 10.0, 34.0, 8.0]
@test vars.symbols == [:s, :v, :s, :v, :s, :v, :s, :v]
@test vars.vehicle_indices == [1, 1, 2, 2, 3, 3, 4, 4]
@test vars.bounds[1] == StateBounds(-(200-34+10-4.0),4.0)
@test vars.bounds[2] == StateBounds(-10.0, 22.0)
@test vars.bounds[3] == StateBounds(-4.0, 4.0)

function speed{R}(
    vars::Vars,
    assignment::Tuple{Int}, # indeces of variables in vars
    roadway::R,
    )

    return vars.values[assignment[1]]
end
function AutoScenes.assign_feature{F <: typeof(speed), R}(
    f::F,
    scene::Union{Scene, Scene1D},
    roadway::R,
    vars::Vars,
    )

    assignments = Tuple{Int}[]
    for (i, sym) in enumerate(vars.symbols)
        if sym == :v
            push!(assignments, (i,))
        end
    end

    return assignments
end

assignments_speed = assign_feature(speed, scene, roadway, vars)
@test assignments_speed == [(2,), (4,), (6,), (8,)]
@test speed(vars, assignments_speed[1], roadway) ≈ 10.0
@test speed(vars, assignments_speed[2], roadway) ≈ 12.0
@test speed(vars, assignments_speed[3], roadway) ≈ 10.0
@test speed(vars, assignments_speed[4], roadway) ≈  8.0

function delta_speed{R}(
    vars::Vars, # all variables
    assignment::Tuple{Int,Int}, # indeces of variables in vars
    roadway::R,
    )

    v_rear = vars.values[assignment[1]]
    v_fore = vars.values[assignment[2]]
    return v_fore - v_rear
end
function AutoScenes.assign_feature{F <: typeof(delta_speed)}(
    f::F,
    scene::Scene1D,
    roadway::StraightRoadway,
    vars::Vars,
    )

    lead_follow = LeadFollowRelationships(scene, roadway)

    assignments = Tuple{Int,Int}[]
    for (vehicle_index, index_fore) in enumerate(lead_follow.index_fore)
        j_rear = findfirst(vars, vehicle_index, :v)
        j_fore = findfirst(vars, index_fore, :v)
        push!(assignments, (j_rear, j_fore))
    end

    return assignments
end

assignments_delta_speed = assign_feature(delta_speed, scene, roadway, vars)
@test assignments_delta_speed == [(2,4), (4,6), (6,8), (8,2)]
@test delta_speed(vars, assignments_delta_speed[1], roadway) ≈ 12.0 - 10.0
@test delta_speed(vars, assignments_delta_speed[2], roadway) ≈ 10.0 - 12.0
@test delta_speed(vars, assignments_delta_speed[3], roadway) ≈  8.0 - 10.0
@test delta_speed(vars, assignments_delta_speed[4], roadway) ≈ 10.0 -  8.0

features = (speed, delta_speed)
assignments = assign_features(features, scene, roadway, vars)
@test assignments[1] == (1, (2,))
@test assignments[2] == (1, (4,))
@test assignments[3] == (1, (6,))
@test assignments[4] == (1, (8,))
@test assignments[5] == (2, (2,4))

"""
    Compute ptilde(assignment) = exp(θᵀf)
"""
function ptilde{F<:Tuple{Vararg{Function}}, R}(
    features::F,
    θ::Vector{Float64},
    vars::Vars,
    assignments::Vector{Tuple{Int, Tuple{Vararg{Int}}}},
    roadway::R,
    )

    v = 0.0
    for (feature_index, assignment) in assignments
        f = features[feature_index]
        w = θ[feature_index]
        v += f(vars, assignment, roadway)
    end

    return exp(v)
end

θ = ones(length(vars))
@test ptilde(features, θ, vars, assignments, roadway) ≈ exp(
        θ[1] * speed(vars, assignments_speed[1], roadway) +
        θ[2] * speed(vars, assignments_speed[2], roadway) +
        θ[3] * speed(vars, assignments_speed[3], roadway) +
        θ[4] * speed(vars, assignments_speed[4], roadway) +
        θ[5] * delta_speed(vars, assignments_delta_speed[1], roadway) +
        θ[6] * delta_speed(vars, assignments_delta_speed[2], roadway) +
        θ[7] * delta_speed(vars, assignments_delta_speed[3], roadway) +
        θ[8] * delta_speed(vars, assignments_delta_speed[4], roadway)
    )

# """
#     Compute E[f(x ∣ other)]

#     where xⱼ ~ P(⋅ | other)

# Where f is the feature function,
# vars are the inputs to the feature function set to their present value,
# and i is the index of x, the variable we are running over.

# This calculation is performed using Monte Carlo integration:

#     E[f(x ∣ other )] ≈ [∑ f(xⱼ ∣ other) * Wⱼ ] / [∑ Wⱼ]

#     where xⱼ ~ Uniform
#       and Wⱼ = ptilde(xⱼ, other) / U(xⱼ)

# Assignments is typically the subset of vectors that
# """
# function calc_expectation_x_given_other{R}(
#     i::Int, # index of the feature (in assignments) we are running this for
#     j::Int, # index of the variable (in vars) we are running this for
#     shared_features::SharedFeatures, # shared feature functions
#     θ::Vector{Float64}, # weights on the shared features
#     vars::Vars, # all variables, set to current assignment
#     assignments::Vector{Tuple{Int, Tuple{Vararg{Int}}}}, # assignment of index of shared feature → indeces of input vars
#     roadway::R,
#     nsamples::Int = 100, # number of Monte Carlo samples
#     )::Float64

#     feature_index, assignment = assignments[i]
#     x₀ = vars.values[j] # store initial value
#     U = Uniform(val₀.bounds)
#     f = shared_features[feature_index]

#     numerator = 0.0
#     denominator = 0.0
#     for k in 1 : nsamples
#         Δx = rand(U)
#         vars.values[k] = VarInstance(val₀.value + Δx, val₀.bounds) # set value
#         W = ptilde(shared_features, θ, vars, assignments, roadway) / pdf(U, Δx)
#         numerator += W*f(vars, assignment, roadway) # unfortunately, this is computed twice
#         denominator += W
#     end

#     vars.values[k] = x₀ # reset initial value

#     return numerator/denominator
# end

###

# function speed{R}(
#     scene::Union{Scene, Scene1D},
#     roadway::R,
#     vehicle_indices::Tuple{Int},
#     )

#     vehicle_index = vehicle_indices[1]
#     return scene[vehicle_index].state.v
# end
# AutoScenes.uses_s{F<:typeof(speed)}(f::F) = false
# AutoScenes.uses_t{F<:typeof(speed)}(f::F) = false
# AutoScenes.uses_v{F<:typeof(speed)}(f::F) = true
# AutoScenes.uses_ϕ{F<:typeof(speed)}(f::F) = false
# function AutoScenes.assign_feature{F <: typeof(speed), R}(
#     f::F,
#     scene::Union{Scene, Scene1D},
#     roadway::R,
#     active_vehicles::Set{Int},
#     lead_follow::LeadFollowRelationships,
#     )

#     retval = Array(Tuple{Vararg{Int}}, length(active_vehicles))
#     for (i,vehicle_index) in enumerate(active_vehicles)
#         retval[i] = (vehicle_index,)
#     end
#     return retval
# end

# f₁ = speed
# @test !uses_s(f₁)
# @test !uses_t(f₁)
# @test  uses_v(f₁)
# @test !uses_ϕ(f₁)

# assignments₁ = assign_feature(f₁, scene, roadway, get_active_vehicles(lead_follow), lead_follow)
# @test length(assignments₁) == 4
# @test (1,) ∈ assignments₁
# @test (2,) ∈ assignments₁
# @test (3,) ∈ assignments₁
# @test (4,) ∈ assignments₁

# structure = SceneStructure(scene, roadway, (f₁,))
# @test structure.lead_follow == lead_follow
# @test 1 ∈ structure.active_vehicles
# @test 2 ∈ structure.active_vehicles
# @test 3 ∈ structure.active_vehicles
# @test 4 ∈ structure.active_vehicles
# @test structure.assignments[f₁] == assignments₁

# @test isapprox(f₁(scene, roadway, (1,)), 10.0)
# @test isapprox(f₁(scene, roadway, (2,)), 12.0)

# ###

# function delta_speed{R}(
#     scene::Union{Scene, Scene1D},
#     roadway::R,
#     vehicle_indices::Tuple{Int, Int},
#     )

#     veh_rear = scene[vehicle_indices[1]]
#     veh_fore = scene[vehicle_indices[2]]

#     return veh_fore.state.v - veh_rear.state.v
# end
# AutoScenes.uses_s{F<:typeof(delta_speed)}(ϕ::F) = false
# AutoScenes.uses_t{F<:typeof(delta_speed)}(ϕ::F) = false
# AutoScenes.uses_v{F<:typeof(delta_speed)}(ϕ::F) = true
# AutoScenes.uses_ϕ{F<:typeof(delta_speed)}(ϕ::F) = false
# function AutoScenes.assign_feature{F <: typeof(delta_speed), R}(
#     f::F,
#     scene::Union{Scene, Scene1D},
#     roadway::R,
#     active_vehicles::Set{Int},
#     lead_follow::LeadFollowRelationships,
#     )

#     retval = Tuple{Vararg{Int}}[]

#     for (vehicle_index, index_fore) in enumerate(lead_follow.index_fore)
#         if index_fore != 0 && (vehicle_index ∈ active_vehicles || index_fore ∈ active_vehicles)

#             @assert index_fore != vehicle_index
#             push!(retval, (vehicle_index, index_fore))
#         end
#     end

#     return retval
# end

# f₂ = delta_speed
# @test !uses_s(f₂)
# @test !uses_t(f₂)
# @test  uses_v(f₂)
# @test !uses_ϕ(f₂)

# assignments₂ = assign_feature(f₂, scene, roadway, get_active_vehicles(lead_follow), lead_follow)
# @test length(assignments₂) == 4
# @test (1,2) ∈ assignments₂
# @test (2,3) ∈ assignments₂
# @test (3,4) ∈ assignments₂
# @test (4,1) ∈ assignments₂

# shared_features = (f₁,f₂)
# structure = SceneStructure(scene, roadway, shared_features)
# @test structure.lead_follow == lead_follow
# @test 1 ∈ structure.active_vehicles
# @test 2 ∈ structure.active_vehicles
# @test 3 ∈ structure.active_vehicles
# @test 4 ∈ structure.active_vehicles
# @test structure.assignments[f₁] == assignments₁
# @test structure.assignments[f₂] == assignments₂

# @test isapprox(f₂(scene, roadway, (1,2)), 12.0 - 10.0)

#####

# vehicle_index = 1
# bounds = VehicleBounds(scene, roadway, structure.lead_follow, vehicle_index)

# pseudo_expectation(f, scene, roadway, vehicle_indices, target)

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