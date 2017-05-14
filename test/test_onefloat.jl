typealias FloatScene Frame{Entity{Float64, Void, Int}}
function AutoScenes.Vars(scene::FloatScene, roadway::Void)
    x = scene[1].state
    return Vars(Float64[x],
                [StateBounds(-x, 1-x)],
                [:x],
                [1])
end

# f(x) = (x < 0.5)
function custom1(
    vars::Vars,
    assignment::Tuple{Int}, # indeces of variables in vars
    roadway::Void,
    )::Float64

    return vars.values[assignment[1]] < 0.5
end
function AutoScenes.assign_feature{F <: typeof(custom1), Void}(
    f::F,
    scene::FloatScene,
    roadway::Void,
    vars::Vars,
    )

    return Tuple{Int}[(1,)]
end

# f(x) = (x ≥ 0.5)
function custom2(
    vars::Vars,
    assignment::Tuple{Int}, # indeces of variables in vars
    roadway::Void,
    )::Float64

    return vars.values[assignment[1]] ≥ 0.5
end
function AutoScenes.assign_feature{F <: typeof(custom2), Void}(
    f::F,
    scene::FloatScene,
    roadway::Void,
    vars::Vars,
    )

    return Tuple{Int}[(1,)]
end

roadway = nothing
scenes = [
    FloatScene([Entity{Float64,Void,Int}(0.0,roadway,1)], 1),
    FloatScene([Entity{Float64,Void,Int}(1/4,roadway,1)], 1),
    FloatScene([Entity{Float64,Void,Int}(3/4,roadway,1)], 1),
    FloatScene([Entity{Float64,Void,Int}(3/4,roadway,1)], 1),
    FloatScene([Entity{Float64,Void,Int}(3/4,roadway,1)], 1),
    FloatScene([Entity{Float64,Void,Int}(1.0,roadway,1)], 1),
]
features = (custom1, custom2)
factorgraphs = [FactorGraph(features, scene, roadway) for scene in scenes]

# E[f1(v)]
srand(0)
θ = Float64[1,1]
for factorgraph in factorgraphs
    v = calc_expectation_x_given_other(1, 1, features, θ, factorgraph.vars, factorgraph.assignments, roadway)
    @test isapprox(v, 0.5)
end

srand(0)
@test isapprox(log_pseudolikelihood(features, θ, factorgraphs[1]), 0.0, atol=1e-8)

srand(0)
@test isapprox(log_pseudolikelihood(features, θ, factorgraphs), 0.0, atol=1e-8)

srand(0)
@test isapprox(log_pseudolikelihood(features, Float64[2,2], factorgraphs), 0.0, atol=1e-8) # should be invariant to scaling

srand(0)
θ = [1/3,2/3]
@test isapprox(log_pseudolikelihood(features, θ, factorgraphs), 0.0417, atol=1e-4) # better!

srand(0)
θ = Float64[1,1]
@test isapprox(log_pseudolikelihood_derivative_complete(1, features, θ, factorgraphs[1]), 0.5, atol=0.025)

∇ = Array(Float64, length(θ))
θ = Float64[1,1]
srand(0)
log_pseudolikelihood_gradient!(∇, features, θ, factorgraphs)
@test isapprox(∇[1], -0.1666667, atol=1e-2)
@test isapprox(∇[2],  0.1666667, atol=1e-2)

θ += ∇
@test log_pseudolikelihood(features, θ, factorgraphs) > 0.0 # should improve