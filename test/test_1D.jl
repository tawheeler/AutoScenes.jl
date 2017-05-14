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
@test assignments[6] == (2, (4,6))
@test assignments[7] == (2, (6,8))
@test assignments[8] == (2, (8,2))

@test scope(1, assignments) == Int[]
@test scope(2, assignments) == [1,5,8]
@test scope(3, assignments) == Int[]
@test scope(4, assignments) == [2,5,6]
scopes = [scope(var_index, assignments) for var_index in 1 : length(vars)]

θ = ones(length(features))
@test ptilde(features, θ, vars, assignments, roadway) ≈ exp(
        θ[1] * speed(vars, assignments_speed[1], roadway) +
        θ[1] * speed(vars, assignments_speed[2], roadway) +
        θ[1] * speed(vars, assignments_speed[3], roadway) +
        θ[1] * speed(vars, assignments_speed[4], roadway) +
        θ[2] * delta_speed(vars, assignments_delta_speed[1], roadway) +
        θ[2] * delta_speed(vars, assignments_delta_speed[2], roadway) +
        θ[2] * delta_speed(vars, assignments_delta_speed[3], roadway) +
        θ[2] * delta_speed(vars, assignments_delta_speed[4], roadway)
    )

# E[speed(v)]
srand(0)
@test isapprox(calc_expectation_x_given_other(1, 2, features, θ, vars, assignments, roadway), 31.0, atol=1e-5)
srand(0)
@test isapprox(calc_expectation_x_given_other(1, 2, features, θ, vars, assignments, scopes, roadway), 30.61164, atol=1e-5)

# E[delta_speed(v_rear | v_fore)]
srand(0)
@test isapprox(calc_expectation_x_given_other(2, 5, features, θ, vars, assignments, roadway), 12.00000, atol=1e-5)
srand(0)
@test isapprox(calc_expectation_x_given_other(2, 5, features, θ, vars, assignments, scopes, roadway), 12.00000, atol=1e-5)

srand(0)
@test isapprox(log_pseudolikelihood(features, θ, vars, assignments, scopes, roadway), -102.49985, atol=1e-4)

srand(0)
@test isapprox(log_pseudolikelihood_derivative_single(1, features, θ, vars, assignments, scopes, roadway), -20.61164, atol=1e-4)

srand(0)
@test isapprox(log_pseudolikelihood_derivative_complete(1, features, θ, vars, assignments, scopes, roadway), -20.61164, atol=1e-4)

srand(0)
@test isapprox(log_pseudolikelihood_derivative_complete(2, features, θ, vars, assignments, scopes, roadway), -133.84615, atol=1e-4)

factorgraph = FactorGraph(vars, assignments, roadway)
factorgraph = FactorGraph(features, scene, roadway)

srand(0)
logPL_init = -102.49985
@test isapprox(log_pseudolikelihood(features, θ, factorgraph), logPL_init, atol=1e-4)
srand(0)
@test isapprox(log_pseudolikelihood_derivative_single(1, features, θ, factorgraph), -20.61164, atol=1e-4)
srand(0)
@test isapprox(log_pseudolikelihood_derivative_complete(1, features, θ, factorgraph), -20.61164, atol=1e-4)
srand(0)
@test isapprox(log_pseudolikelihood_derivative_complete(2, features, θ, factorgraph), -133.84615, atol=1e-4)

factorgraphs = [factorgraph]
srand(0)
@test isapprox(log_pseudolikelihood(features, θ, factorgraphs), -102.49985, atol=1e-4)
srand(0)
@test isapprox(log_pseudolikelihood_derivative_complete(1, features, θ, factorgraphs), -20.61164, atol=1e-4)
srand(0)
@test isapprox(log_pseudolikelihood_derivative_complete(2, features, θ, factorgraphs), -133.84615, atol=1e-4)

∇ = Array(Float64, length(θ))
srand(0)
log_pseudolikelihood_gradient!(∇, features, θ, factorgraphs)
@test isapprox(∇[1],  -20.61164, atol=1e-4)
@test isapprox(∇[2], -133.84615, atol=0.1)

θ += 0.001*∇
srand(0)
@test log_pseudolikelihood(features, θ, factorgraphs) > logPL_init # it increases!