immutable PseudolikelihoodPrealloc

    lnP_tilde_denom_arr_s::Vector{Float64}
    lnP_tilde_denom_arr_t::Vector{Float64}
    lnP_tilde_denom_arr_v::Vector{Float64}
    lnP_tilde_denom_arr_ϕ::Vector{Float64}

    function PseudolikelihoodPrealloc(n_samples_monte_carlo_integration::Int)
        new(Array(Float64, n_samples_monte_carlo_integration),
            Array(Float64, n_samples_monte_carlo_integration),
            Array(Float64, n_samples_monte_carlo_integration),
            Array(Float64, n_samples_monte_carlo_integration))
    end
end

function posF_matches_posG(state::VehicleState, roadway::Roadway)
    posG = state.posG
    posF = get_posG(state.posF, roadway)
    abs(posG - posF) < 0.15
end

function get_relative_variable_bounds_s(scene::Scene, structure::SceneStructure, roadway::Roadway, vehicle_index::Int)
    veh_rear = scene[vehicle_index]
    veh_fore = scene[structure.lead_follow.index_fore[vehicle_index]]
    relpos = get_frenet_relative_position(veh_fore, veh_rear, roadway)
    Δs_fore = relpos.Δs - veh_rear.def.length/2 -  veh_fore.def.length/2

    veh_rear = scene[structure.lead_follow.index_rear[vehicle_index]]
    veh_fore = scene[vehicle_index]
    relpos = get_frenet_relative_position(veh_fore, veh_rear, roadway)
    @assert(relpos.Δs ≥ 0.0)
    Δs_rear = relpos.Δs - veh_rear.def.length/2 -  veh_fore.def.length/2
    (-Δs_rear, Δs_fore) # NOTE: these are relative to the current s
end
function get_relative_variable_bounds_t(rec::SceneRecord, roadway::Roadway, vehicle_index::Int)
    d_left = convert(Float64, get(MARKERDIST_LEFT, rec, roadway, vehicle_index))
    d_right = convert(Float64, get(MARKERDIST_RIGHT, rec, roadway, vehicle_index))
    (-d_right, d_left)  # NOTE: these are relative to the current t
end
function get_relative_variable_bounds_v(scene::Scene, vehicle_index::Int)
    v = scene[vehicle_index].state.v
    (-1.0-v, 32.0-v) # [m/s]
end
function get_relative_variable_bounds_ϕ(scene::Scene, vehicle_index::Int)
    ϕ = scene[vehicle_index].state.posF.ϕ
    (-0.82-ϕ, 0.82-ϕ) # [rad]
end

function set_s!(veh::Vehicle, state_original::VehicleState, roadway::Roadway, value_delta::Float64)
    veh.state = move_along(state_original, roadway, value_delta)
    veh
end
function set_t!(veh::Vehicle, state_original::VehicleState, roadway::Roadway, value_delta::Float64)
    f = state_original.posF
    new_frenet = Frenet(f.roadind, f.s, f.t+value_delta, f.ϕ)
    veh.state = VehicleState(new_frenet, roadway, state_original.v)
    veh
end
function set_v!(veh::Vehicle, state_original::VehicleState, roadway::Roadway, value_delta::Float64)
    veh.state = VehicleState(state_original.posG, state_original.posF, state_original.v+value_delta)
    veh
end
function set_ϕ!(veh::Vehicle, state_original::VehicleState, roadway::Roadway, value_delta::Float64)
    g = state_original.posG
    new_posG = VecSE2(g.x, g.y, g.θ + value_delta)

    f = state_original.posF
    new_posF = Frenet(f.roadind, f.s, f.t, f.ϕ+value_delta)

    veh.state = VehicleState(new_posG, new_posF, state_original.v)
    veh
end

function _update_lnP_tilde_denom_arr_s!(
    dat::PseudolikelihoodPrealloc,
    ϕ::SharedFactor,
    scene::Scene,
    roadway::Roadway,
    vehicle_index::Int,
    vehicle_indeces::Vector{Int},
    Δlo::Float64,
    Δhi::Float64,
    nsamples::Int,
    )

    ###########################
    # v -> lnP(v | ~)
    #   vary v and compute the denominator term, ∫ᵥ p(v | ~) dv
    #   use Monte Carlo integration to get ∫ₐ P(a , x₋ⱼ) ≈ V/N ∑ₐ P(a , x₋ⱼ) where V is domain width

    # get original state
    veh = scene[vehicle_index]
    state_original = veh.state

    Δv = Δlo
    Δval = (Δhi - Δlo) / (nsamples-1)
    for k in 1 : nsamples
        set_s!(veh, state_original, roadway, Δv)
        extract!(ϕ, scene, roadway, vehicle_indeces)
        dat.lnP_tilde_denom_arr_s[k] += evaluate_dot(ϕ)
        Δv += Δval
    end

    # reset original state
    scene[vehicle_index].state = state_original

    dat
end
function _update_lnP_tilde_denom_arr_t!(
    dat::PseudolikelihoodPrealloc,
    ϕ::SharedFactor,
    scene::Scene,
    roadway::Roadway,
    vehicle_index::Int,
    vehicle_indeces::Vector{Int},
    Δlo::Float64,
    Δhi::Float64,
    nsamples::Int,
    )

    ###########################
    # v -> lnP(v | ~)
    #   vary v and compute the denominator term, ∫ᵥ p(v | ~) dv
    #   use Monte Carlo integration to get ∫ₐ P(a , x₋ⱼ) ≈ V/N ∑ₐ P(a , x₋ⱼ) where V is domain width

    # reset original state
    veh = scene[vehicle_index]
    state_original = veh.state

    Δv = Δlo
    Δval = (Δhi - Δlo) / (nsamples-1)
    for k in 1 : nsamples
        set_t!(veh, state_original, roadway, Δv)
        extract!(ϕ, scene, roadway, vehicle_indeces)
        dat.lnP_tilde_denom_arr_t[k] += evaluate_dot(ϕ)
        Δv += Δval
    end

    # reset original state
    scene[vehicle_index].state = state_original

    dat
end
function _update_lnP_tilde_denom_arr_v!(
    dat::PseudolikelihoodPrealloc,
    ϕ::SharedFactor,
    scene::Scene,
    roadway::Roadway,
    vehicle_index::Int,
    vehicle_indeces::Vector{Int},
    Δlo::Float64,
    Δhi::Float64,
    nsamples::Int,
    )

    ###########################
    # v -> lnP(v | ~)
    #   vary v and compute the denominator term, ∫ᵥ p(v | ~) dv
    #   use Monte Carlo integration to get ∫ₐ P(a , x₋ⱼ) ≈ V/N ∑ₐ P(a , x₋ⱼ) where V is domain width

    # reset original state
    veh = scene[vehicle_index]
    state_original = veh.state

    Δv = Δlo
    Δval = (Δhi - Δlo) / (nsamples-1)
    for k in 1 : nsamples
        set_v!(veh, state_original, roadway, Δv)
        extract!(ϕ, scene, roadway, vehicle_indeces)
        dat.lnP_tilde_denom_arr_v[k] += evaluate_dot(ϕ)
        Δv += Δval
    end

    # reset original state
    scene[vehicle_index].state = state_original

    dat
end
function _update_lnP_tilde_denom_arr_ϕ!(
    dat::PseudolikelihoodPrealloc,
    ϕ::SharedFactor,
    scene::Scene,
    roadway::Roadway,
    vehicle_index::Int,
    vehicle_indeces::Vector{Int},
    Δlo::Float64,
    Δhi::Float64,
    nsamples::Int,
    )

    ###########################
    # v -> lnP(v | ~)
    #   vary v and compute the denominator term, ∫ᵥ p(v | ~) dv
    #   use Monte Carlo integration to get ∫ₐ P(a , x₋ⱼ) ≈ V/N ∑ₐ P(a , x₋ⱼ) where V is domain width

    # reset original state
    veh = scene[vehicle_index]
    state_original = veh.state
    f = state_original.posF

    Δv = Δlo
    Δval = (Δhi - Δlo) / (nsamples-1)
    for k in 1 : nsamples
        set_ϕ!(veh, state_original, roadway, Δv)
        extract!(ϕ, scene, roadway, vehicle_indeces)
        dat.lnP_tilde_denom_arr_ϕ[k] += evaluate_dot(ϕ)
        Δv += Δval
    end

    # reset original state
    scene[vehicle_index].state = state_original

    dat
end

function _get_logP_tilde_denom(lnP_tilde_denom_arr::Vector{Float64}, volume::Float64, n_samples_monte_carlo_integration::Int)
    P_tilde_denom = 0.0
    for v in lnP_tilde_denom_arr
        P_tilde_denom += exp(v)
    end

    # DEBUG
    blah = P_tilde_denom * volume/n_samples_monte_carlo_integration
    if isinf(blah) || isnan(blah) || blah ≤ 0.0
        println("lnP_tilde_denom_arr:               ", lnP_tilde_denom_arr)
        println("P_tilde_denom:                     ", P_tilde_denom)
        println("volume:                            ", volume)
        println("n_samples_monte_carlo_integration: ", n_samples_monte_carlo_integration)
    end
    log(P_tilde_denom * volume/n_samples_monte_carlo_integration)
end

function calc_pseudolikelihood(dset::SceneStructureDataset;
    dat::PseudolikelihoodPrealloc=PseudolikelihoodPrealloc(10),
    scene::Scene = Scene(),
    rec::SceneRecord = SceneRecord(1, NaN),
    )

    retval = 0.0
    M = length(dset)
    n_samples_monte_carlo_integration = length(dat.lnP_tilde_denom_arr_s)

    for m in 1 : M

        scene, structure, roadway = get_scene_structure_and_roadway!(scene, dset, m)
        update!(rec, scene)

        for veh in scene
            @assert(posF_matches_posG(veh.state, roadway))
        end

        for vehicle_index in structure.active_vehicles

            lnP_tilde = 0.0
            fill!(dat.lnP_tilde_denom_arr_s, 0.0)
            fill!(dat.lnP_tilde_denom_arr_t, 0.0)
            fill!(dat.lnP_tilde_denom_arr_v, 0.0)
            fill!(dat.lnP_tilde_denom_arr_ϕ, 0.0)

            Δlo_s, Δhi_s = get_relative_variable_bounds_s(scene, structure, roadway, vehicle_index)
            Δlo_t, Δhi_t = get_relative_variable_bounds_t(rec, roadway, vehicle_index)
            Δlo_v, Δhi_v = get_relative_variable_bounds_v(scene, vehicle_index)
            Δlo_ϕ, Δhi_ϕ = get_relative_variable_bounds_ϕ(scene, vehicle_index)

            if Δhi_s - Δlo_s < 0.0
                println("m: ", m)
                println("vehicle_index: ", vehicle_index)
                println("s: ", Δlo_s, "  ", Δhi_s)
            end
            if Δhi_t - Δlo_t < 0.0
                println("t: ", Δlo_t, "  ", Δhi_t)
            end
            if Δhi_v - Δlo_v < 0.0
                println("v: ", Δlo_v, "  ", Δhi_v)
            end
            if Δhi_ϕ - Δlo_ϕ < 0.0
                println("ϕ: ", Δlo_ϕ, "  ", Δhi_ϕ)
            end

            # println("vehicle_index: ", vehicle_index)
            # println("s: ", Δlo_s, "  ", Δhi_s)
            # println("t: ", Δlo_t, "  ", Δhi_t)
            # println("v: ", Δlo_v, "  ", Δhi_v)
            # println("ϕ: ", Δlo_ϕ, "  ", Δhi_ϕ)

            for fa in structure.factor_assignments

                ϕ = dset.factors[fa.form]
                extract!(ϕ, scene, roadway, fa.vehicle_indeces)
                fd = evaluate_dot(ϕ)
                lnP_tilde += fd

                if vehicle_index in fa.vehicle_indeces # this factor affects the vehicle

                    target_index = findfirst(fa.vehicle_indeces, vehicle_index)
                    @assert(target_index > 0)

                    _update_lnP_tilde_denom_arr_s!(dat, ϕ, scene, roadway, vehicle_index, fa.vehicle_indeces, Δlo_s, Δhi_s, n_samples_monte_carlo_integration)
                    _update_lnP_tilde_denom_arr_t!(dat, ϕ, scene, roadway, vehicle_index, fa.vehicle_indeces, Δlo_t, Δhi_t, n_samples_monte_carlo_integration)
                    _update_lnP_tilde_denom_arr_v!(dat, ϕ, scene, roadway, vehicle_index, fa.vehicle_indeces, Δlo_v, Δhi_v, n_samples_monte_carlo_integration)
                    _update_lnP_tilde_denom_arr_ϕ!(dat, ϕ, scene, roadway, vehicle_index, fa.vehicle_indeces, Δlo_ϕ, Δhi_ϕ, n_samples_monte_carlo_integration)

                else # this factor does not affect the vehicle
                    for k in 1 : n_samples_monte_carlo_integration
                        dat.lnP_tilde_denom_arr_s[k] += fd
                        dat.lnP_tilde_denom_arr_t[k] += fd
                        dat.lnP_tilde_denom_arr_v[k] += fd
                        dat.lnP_tilde_denom_arr_ϕ[k] += fd
                    end
                end
            end

            retval += lnP_tilde - _get_logP_tilde_denom(dat.lnP_tilde_denom_arr_s, Δhi_s - Δlo_s, n_samples_monte_carlo_integration)
            retval += lnP_tilde - _get_logP_tilde_denom(dat.lnP_tilde_denom_arr_t, Δhi_t - Δlo_t, n_samples_monte_carlo_integration)
            retval += lnP_tilde - _get_logP_tilde_denom(dat.lnP_tilde_denom_arr_v, Δhi_v - Δlo_v, n_samples_monte_carlo_integration)
            retval += lnP_tilde - _get_logP_tilde_denom(dat.lnP_tilde_denom_arr_ϕ, Δhi_ϕ - Δlo_ϕ, n_samples_monte_carlo_integration)
        end
    end
    retval /= M

    retval
end

function calc_pseudolikelihood_gradient_component_s(
    ϕ::SharedFactor,
    scene::Scene,
    structure::SceneStructure,
    roadway::Roadway,
    vehicle_index::Int,
    vehicle_indeces::Vector{Int},
    target_instance::GraphFeatureInstance,
    n_samples_monte_carlo_integration::Int,
    rng::AbstractRNG,
    )

    Δlo, Δhi = get_relative_variable_bounds_s(scene, structure, roadway, vehicle_index)
    volume = Δhi - Δlo

    @assert(volume > 0.0)

    # get original state
    veh = scene[vehicle_index]
    state_original = veh.state

    # compute expectation term
    #   evaluated using Importance Sampling
    E_numerator = 0.0
    E_denominator = 0.0
    for r in 1 : n_samples_monte_carlo_integration

        set_s!(veh, state_original, roadway, volume*rand(rng) + Δlo)
        extract!(ϕ, scene, roadway, vehicle_indeces)
        f = evaluate(ϕ.template, target_instance)

        p_true = evaluate_dot(ϕ)
        p_importance = 1.0/volume
        W = p_true / p_importance

        @assert(!isnan(p_true))
        @assert(!isnan(p_importance))
        @assert(!isnan(W))
        @assert(!isinf(W))

        E_numerator += f*W
        E_denominator += W
    end

    # reset original state
    scene[vehicle_index].state = state_original

    if !isapprox(E_denominator, 0.0)
        E = E_numerator / E_denominator
    else
        E = 0.0
    end

    @assert(!isnan(E))
    @assert(!isinf(E))

    -E
end
function calc_pseudolikelihood_gradient_component_t(
    ϕ::SharedFactor,
    scene::Scene,
    structure::SceneStructure,
    roadway::Roadway,
    vehicle_index::Int,
    vehicle_indeces::Vector{Int},
    target_instance::GraphFeatureInstance,
    n_samples_monte_carlo_integration::Int,
    rng::AbstractRNG,
    rec::SceneRecord,
    )

    Δlo, Δhi = get_relative_variable_bounds_t(update!(rec, scene), roadway, vehicle_index)
    volume = Δhi - Δlo

    # get original state
    veh = scene[vehicle_index]
    state_original = veh.state

    # compute expectation term
    #   evaluated using Importance Sampling
    E_numerator = 0.0
    E_denominator = 0.0
    for r in 1 : n_samples_monte_carlo_integration

        set_t!(veh, state_original, roadway, volume*rand(rng) + Δlo)
        extract!(ϕ, scene, roadway, vehicle_indeces)
        f = evaluate(ϕ.template, target_instance)

        p_true = evaluate_dot(ϕ)
        p_importance = 1.0/volume
        W = p_true / p_importance

        @assert(!isnan(p_true))
        @assert(!isnan(p_importance))
        @assert(!isnan(W))
        @assert(!isinf(W))

        E_numerator += f*W
        E_denominator += W
    end

    # reset original state
    scene[vehicle_index].state = state_original

    E = E_numerator / E_denominator

    if isnan(E)
        print("vals: ")
        for i in 1 : length(ϕ.instances)
            v = evaluate(ϕ.template, ϕ.instances[i])
            @printf("%.3f  ", v)
        end
        println("")
        println("weights: ", ϕ.weights)
        println("eval: ", evaluate_dot(ϕ))
        println("n_samples_monte_carlo_integration: ", n_samples_monte_carlo_integration)
        println("E_numerator: ", E_numerator)
        println("E_denominator: ", E_denominator)
    end

    @assert(!isnan(E))
    @assert(!isinf(E))

    -E
end
function calc_pseudolikelihood_gradient_component_v(
    ϕ::SharedFactor,
    scene::Scene,
    structure::SceneStructure,
    roadway::Roadway,
    vehicle_index::Int,
    vehicle_indeces::Vector{Int},
    target_instance::GraphFeatureInstance,
    n_samples_monte_carlo_integration::Int,
    rng::AbstractRNG,
    )

    Δlo, Δhi = get_relative_variable_bounds_v(scene, vehicle_index)
    volume = Δhi - Δlo

    # get original state
    veh = scene[vehicle_index]
    state_original = veh.state

    # compute expectation term
    #   evaluated using Importance Sampling
    E_numerator = 0.0
    E_denominator = 0.0
    for r in 1 : n_samples_monte_carlo_integration

        set_v!(veh, state_original, roadway, volume*rand(rng) + Δlo)
        extract!(ϕ, scene, roadway, vehicle_indeces)
        f = evaluate(ϕ.template, target_instance)

        p_true = evaluate_dot(ϕ)
        p_importance = 1.0/volume
        W = p_true / p_importance

        @assert(!isnan(p_true))
        @assert(!isnan(p_importance))
        @assert(!isnan(W))

        E_numerator += f*W
        E_denominator += W
    end

    # reset original state
    scene[vehicle_index].state = state_original

    E = E_numerator / E_denominator

    @assert(!isnan(E))
    @assert(!isinf(E))

    -E
end
function calc_pseudolikelihood_gradient_component_ϕ(
    ϕ::SharedFactor,
    scene::Scene,
    structure::SceneStructure,
    roadway::Roadway,
    vehicle_index::Int,
    vehicle_indeces::Vector{Int},
    target_instance::GraphFeatureInstance,
    n_samples_monte_carlo_integration::Int,
    rng::AbstractRNG,
    )

    Δlo, Δhi = get_relative_variable_bounds_ϕ(scene, vehicle_index)
    volume = Δhi - Δlo

    # get original state
    veh = scene[vehicle_index]
    state_original = veh.state
    f = state_original.posF

    # compute expectation term
    #   evaluated using Importance Sampling
    E_numerator = 0.0
    E_denominator = 0.0
    for r in 1 : n_samples_monte_carlo_integration

        set_ϕ!(veh, state_original, roadway, volume*rand(rng) + Δlo)
        extract!(ϕ, scene, roadway, vehicle_indeces)
        f = evaluate(ϕ.template, target_instance)

        p_true = evaluate_dot(ϕ)
        p_importance = 1.0/volume
        W = p_true / p_importance

        @assert(!isnan(p_true))
        @assert(!isnan(p_importance))
        @assert(!isnan(W))

        E_numerator += f*W
        E_denominator += W
    end

    # reset original state
    scene[vehicle_index].state = state_original

    E = E_numerator / E_denominator

    @assert(!isnan(E))
    @assert(!isinf(E))

    -E
end

function calc_pseudolikelihood_gradient(
    form::Int,
    feature_index::Int,
    dset::SceneStructureDataset,
    batch_size::Int,
    n_samples_monte_carlo_integration::Int,
    regularization::Float64,
    rng::AbstractRNG=Base.GLOBAL_RNG,
    scene::Scene = Scene(),
    rec::SceneRecord = SceneRecord(2, 0.1),
    )

    retval = 0.0
    ϕ = dset.factors[form]
    target_instance = ϕ.instances[feature_index]

    for m in 1 : batch_size

        structure_index = rand(1:length(dset)) # sample uniformly at random
        scene, structure, roadway = get_scene_structure_and_roadway!(scene, dset, structure_index)

        for fa in structure.factor_assignments
            if fa.form == form # is the correct factor

                # first component
                extract!(ϕ, scene, roadway, fa.vehicle_indeces)
                eval = evaluate(ϕ.template, target_instance)

                @assert(!isnan(retval))
                @assert(!isinf(retval))

                for vehicle_index in fa.vehicle_indeces

                    # only calc if the vehicle is active
                    if vehicle_index in structure.active_vehicles

                        if uses_s(target_instance)
                            retval += eval
                            retval += calc_pseudolikelihood_gradient_component_s(ϕ, scene, structure, roadway, vehicle_index, fa.vehicle_indeces,
                                                                                 target_instance,n_samples_monte_carlo_integration, rng)
                        end
                        if uses_t(target_instance)
                            retval += eval
                            retval += calc_pseudolikelihood_gradient_component_t(ϕ, scene, structure, roadway, vehicle_index, fa.vehicle_indeces,
                                                                                 target_instance,n_samples_monte_carlo_integration, rng, rec)
                        end
                        if uses_v(target_instance)
                            retval += eval
                            retval += calc_pseudolikelihood_gradient_component_v(ϕ, scene, structure, roadway, vehicle_index, fa.vehicle_indeces,
                                                                                 target_instance,n_samples_monte_carlo_integration, rng)
                        end
                        if uses_ϕ(target_instance)
                            retval += eval
                            retval += calc_pseudolikelihood_gradient_component_ϕ(ϕ, scene, structure, roadway, vehicle_index, fa.vehicle_indeces,
                                                                                 target_instance,n_samples_monte_carlo_integration, rng)
                        end
                    end
                end
            end
        end
    end

    retval /= batch_size

    # add regularization
    retval -= 2*regularization*ϕ.weights[feature_index]

    retval
end