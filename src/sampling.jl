typealias MHTransition Tuple{Normal, Normal, Normal, Normal} # {Δs,Δt,Δv,Δϕ}

type SceneGenerator
    factors::Vector{SharedFactor}
    propsal_distribution::MHTransition
    burnin::Int # number of burn-in steps

    Δ_propose::Vector{Float64}
    mem::CPAMemory
    rec::SceneRecord

    function SceneGenerator(
        factors::Vector{SharedFactor},
        propsal_distribution::MHTransition,
        burnin::Int;
        Δ_propose::Vector{Float64}=Array(Float64, 4),
        mem::CPAMemory=CPAMemory(),
        )

        retval = new()
        retval.factors = factors
        retval.propsal_distribution = propsal_distribution
        retval.burnin = burnin
        retval.Δ_propose = Δ_propose
        retval.mem = mem
        retval.rec = SceneRecord(1, NaN)
        retval
    end
end
Base.show(io::IO, sg::SceneGenerator) = print(io, "SceneGenerator(burnin=", sg.burnin, ")")

function get_shifted_state(
    scene::Scene,
    structure::SceneStructure,
    roadway::Roadway,
    vehicle_index::Int,
    Δ_propose::Vector{Float64}, # Δs,Δt,Δv,Δϕ
    )

    Δs = Δ_propose[1]
    Δt = Δ_propose[2]
    Δv = Δ_propose[3]
    Δϕ = Δ_propose[4]

    state = scene[vehicle_index].state
    move_along(state, roadway, Δs,
               ϕ₂=state.posF.ϕ + Δϕ,
               t₂=state.posF.t + Δt,
               v₂=state.v + Δv)
end
function draw_active_vehicle_index(scene::Scene, structure::SceneStructure)
    active_veh_index = rand(1:length(structure.active_vehicles))
    for i in 1 : length(scene.vehicles)
        if in(i, structure.active_vehicles)
            active_veh_index -= 1
            if active_veh_index == 0
                return i
            end
        end
    end
    error("active_veh_index invalid")
    0
end

function evaluate_dot!(
    structure::SceneStructure,
    vehicle_index::Int,
    factors::Vector{SharedFactor},
    scene::Scene,
    roadway::Roadway,
    rec::SceneRecord,
    )

    # Only evaluate the factors of which vehicle_index is a member
    # NOTE: this will call extract!

    retval = 0.0
    for i in 1 : length(structure.factor_assignments)

        fa = structure.factor_assignments[i]
        if vehicle_index in fa.vehicle_indeces
            ϕ = factors[fa.form]
            extract!(ϕ, scene, roadway, fa.vehicle_indeces)
            retval += evaluate_dot(ϕ)
        end
    end
    retval
end

function calc_acceptance_probability(
    state_propose::VehicleState,
    vehicle_index::Int,
    factors::Vector{SharedFactor},
    scene::Scene,
    rec::SceneRecord,
    structure::SceneStructure,
    roadway::Roadway,
    mem::CPAMemory,
    MH_transition::MHTransition,
    logprob_trans::Float64,
    Δ_propose::Vector{Float64},
    )

    state_orig = scene[vehicle_index].state
    scene[vehicle_index].state = state_propose

    logprob_rev_trans = 0.0
    Δlo, Δhi = get_relative_variable_bounds_s(scene, structure, roadway, vehicle_index)
    logprob_rev_trans += logpdf(TruncatedNormal(MH_transition[1], Δlo, Δhi), Δ_propose[1])
    if Δlo > 0.0 || Δhi < 0.0
        scene[vehicle_index].state = state_orig
        return 0.0
    end

    update!(rec, scene)
    Δlo, Δhi = get_relative_variable_bounds_t(rec, roadway, vehicle_index)
    logprob_rev_trans += logpdf(TruncatedNormal(MH_transition[2], Δlo, Δhi), Δ_propose[2])
    if Δlo > 0.0 || Δhi < 0.0
        scene[vehicle_index].state = state_orig
        return 0.0
    end

    Δlo, Δhi = get_relative_variable_bounds_v(scene, vehicle_index)
    logprob_rev_trans += logpdf(TruncatedNormal(MH_transition[3], Δlo, Δhi), Δ_propose[3])
    if Δlo > 0.0 || Δhi < 0.0
        scene[vehicle_index].state = state_orig
        return 0.0
    end

    Δlo, Δhi = get_relative_variable_bounds_ϕ(scene, vehicle_index)
    logprob_rev_trans += logpdf(TruncatedNormal(MH_transition[4], Δlo, Δhi), Δ_propose[4])
    if Δlo > 0.0 || Δhi < 0.0
        scene[vehicle_index].state = state_orig
        return 0.0
    end

    if get_first_collision(scene, vehicle_index, mem).is_colliding
        scene[vehicle_index].state = state_orig
        return 0.0
    end
    scene[vehicle_index].state = state_orig

    veh = scene[vehicle_index]
    state_current = veh.state
    log_p_current = evaluate_dot!(structure, vehicle_index, factors, scene, roadway, rec)
    veh.state = state_propose
    log_p_propose = evaluate_dot!(structure, vehicle_index, factors, scene, roadway, rec)
    veh.state = state_orig

    # min(1.0, p_propose / p_current)
    min(1.0, exp(logprob_rev_trans - logprob_trans + log_p_propose - log_p_current))
end
function metropolis_hastings_step!(
    scene::Scene,
    structure::SceneStructure,
    roadway::Roadway,
    factors::Vector{SharedFactor},
    MH_transition::MHTransition,
    Δ_propose::Vector{Float64} = Array(Float64, 4), # preallocated memory
    mem::CPAMemory = CPAMemory(),
    rec::SceneRecord = SceneRecord(1, NaN),
    )

    #=
    pick a random vehicle and shift it
    =#

    update!(rec, scene)
    vehicle_index = draw_active_vehicle_index(scene, structure)
    Δlo_s, Δhi_s = get_relative_variable_bounds_s(scene, structure, roadway, vehicle_index)
    Δlo_t, Δhi_t = get_relative_variable_bounds_t(rec, roadway, vehicle_index)
    Δlo_v, Δhi_v = get_relative_variable_bounds_v(scene, vehicle_index)
    Δlo_ϕ, Δhi_ϕ = get_relative_variable_bounds_ϕ(scene, vehicle_index)

    # use TruncatedNormals to enforce bounds
    logprob_trans = 0.0
    trunc_s = TruncatedNormal(MH_transition[1], Δlo_s, Δhi_s)
    trunc_t = TruncatedNormal(MH_transition[2], Δlo_t, Δhi_t)
    trunc_v = TruncatedNormal(MH_transition[3], Δlo_v, Δhi_v)
    trunc_ϕ = TruncatedNormal(MH_transition[4], Δlo_ϕ, Δhi_ϕ)
    Δ_propose[1] = rand(trunc_s)
    Δ_propose[2] = rand(trunc_t)
    Δ_propose[3] = rand(trunc_v)
    Δ_propose[4] = rand(trunc_ϕ)
    logprob_trans += logpdf(trunc_s, Δ_propose[1])
    logprob_trans += logpdf(trunc_t, Δ_propose[2])
    logprob_trans += logpdf(trunc_v, Δ_propose[3])
    logprob_trans += logpdf(trunc_ϕ, Δ_propose[4])

    state_propose = get_shifted_state(scene, structure, roadway, vehicle_index, Δ_propose)
    if rand() ≤ calc_acceptance_probability(state_propose, vehicle_index, factors,
                                            scene, rec, structure, roadway, mem, MH_transition, logprob_trans, Δ_propose)
        scene[vehicle_index].state = state_propose
    end

    scene
end
function metropolis_hastings!(
    scene::Scene,
    structure::SceneStructure,
    roadway::Roadway,
    factors::Vector{SharedFactor},
    propsal_distribution::MHTransition,
    n_steps::Int,
    Δ_propose::Vector{Float64}=Array(Float64, 4), # preallocated memory
    mem::CPAMemory=CPAMemory(),
    rec::SceneRecord=SceneRecord(1, NaN),
    )

    #=
    Runs Metropolis Hastings for n steps
    =#

    for i in 1 : n_steps
        metropolis_hastings_step!(scene, structure, roadway, factors,
                                  propsal_distribution, Δ_propose, mem)
    end

    scene
end

function Distributions.sample(sg::SceneGenerator, dset::SceneStructureDataset)

    starting_scene_index = rand(1:length(dset))
    source = dset.sources[starting_scene_index]
    scene, structure, roadway = get_scene_structure_and_roadway!(Scene(), dset, starting_scene_index)

    metropolis_hastings!(scene, structure, roadway, sg.factors,
                         sg.propsal_distribution, sg.burnin, sg.Δ_propose, sg.mem, sg.rec)

    (scene, source, structure, roadway)
end

function Distributions.sample(N::Int, sg::SceneGenerator, dset::SceneStructureDataset)
    sdset = SceneDataset()
    for i in 1 : N
        scene, source, structure, roadway = sample(sg, dset)
        push!(sdset, scene, source, structure)
    end
    sdset
end