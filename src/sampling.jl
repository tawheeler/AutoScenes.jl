struct FactorGraphSceneGenerator{F}
    model::FactorModel{F}
    Ts::Dict{Symbol, Normal{Float64}} # normal transition distributions for each variable type
    burnin::Int # number of burn-in steps
end
Base.show(io::IO, sg::FactorGraphSceneGenerator) = print(io, "FactorGraphSceneGenerator(burnin=", sg.burnin, ")")

"""
Sample a transition for all variables from the transition proposal distribution,
and then accept according to the acceptance probability.
"""
function metropolis_hastings_step!{F,R}(
    gen::FactorGraphSceneGenerator{F},
    factorgraph::FactorGraph{R},
    a::Vector{Float64}, # a set of deviations from factorgraph.vars.values
    b::Vector{Float64}, # candidate deviations from factorgraph.vars.values
    logPtilde_a::Float64, # precomputed
    )

    vars = factorgraph.vars

    # compute logP(a→b) and logP(b→a)
    # use TruncatedNormals to help enforce bounds
    logP_a2b = 0.0
    logP_b2a = 0.0
    for i in 1 : length(vars)
        sym = vars.symbols[i]
        bounds = vars.bounds[i]

        Pa2b = Truncated(gen.Ts[sym], bounds.Δlo - a[i], bounds.Δhi - a[i])
        Δ = rand(Pa2b) # proposed transition for this variable
        logP_a2b += logpdf(Pa2b, Δ)
        b[i] = a[i] + Δ
        Pb2a = Truncated(gen.Ts[sym], bounds.Δlo - b[i], bounds.Δhi - b[i])
        logP_b2a += logpdf(Pb2a, -Δ)
        @assert !isinf(logP_a2b)
        @assert !isinf(logP_b2a)
    end

    # TODO: ensure that new state is acceptable to the graph

    # calc acceptance probability
    # - you need to add b into vars.values and then recover
    vars.values .+= b
    logPtilde_b = log_ptilde(gen.model.features, gen.model.weights, vars,
                             factorgraph.assignments, factorgraph.roadway)
    vars.values .-= b

    logA = logPtilde_b - logPtilde_a + logP_b2a - logP_a2b
    A = exp(logA)

    # see whether we accept
    if rand() ≤ A
        copy!(a, b)
        logPtilde_a = logPtilde_b
    end

    return (a, logPtilde_a)
end
function metropolis_hastings!{F,R}(
    gen::FactorGraphSceneGenerator{F},
    factorgraph::FactorGraph{R};

    n_steps::Int = gen.burnin,
    a::Vector{Float64} = zeros(length(factorgraph.vars)),
    b::Vector{Float64} = similar(a),
    logPtilde_a::Float64 = begin
        factorgraph.vars.values .+= a
        logPtilde_a = log_ptilde(gen.model.features, gen.model.weights, factorgraph.vars,
            factorgraph.assignments, factorgraph.roadway)
        factorgraph.vars.values .-= a
        logPtilde_a
    end,
    )

    for i in 1 : n_steps
        (a, logPtilde_a) = metropolis_hastings_step!(gen, factorgraph, a, b, logPtilde_a)
    end

    return a
end