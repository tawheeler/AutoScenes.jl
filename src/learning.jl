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

    # reset original state
    veh = scene[vehicle_index]
    state_original = veh.state
    f = state_original.posF

    Δv = Δlo
    Δval = (Δhi - Δlo) / (nsamples-1)
    for k in 1 : nsamples
        veh.state = move_along(state_original, roadway, Δv)
        extract!(ϕ, scene, roadway, vehicle_indeces)

        if evaluate_dot(ϕ) > 1000.0
            println(ϕ)
        end

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
    f = state_original.posF

    Δv = Δlo
    Δval = (Δhi - Δlo) / (nsamples-1)
    for k in 1 : nsamples

        new_frenet = Frenet(f.roadind, f.s, f.t+Δv, f.ϕ)
        veh.state = VehicleState(new_frenet, roadway, state_original.v)

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
    f = state_original.posF

    Δv = Δlo
    Δval = (Δhi - Δlo) / (nsamples-1)
    for k in 1 : nsamples

        veh.state = VehicleState(f, roadway, state_original.v+Δv)

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

        new_frenet = Frenet(f.roadind, f.s, f.t, f.ϕ+Δv)
        veh.state = VehicleState(new_frenet, roadway, state_original.v)

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
    println("lnP_tilde_denom_arr:               ", lnP_tilde_denom_arr)
    println("P_tilde_denom:                     ", P_tilde_denom)
    println("volume:                            ", volume)
    println("n_samples_monte_carlo_integration: ", n_samples_monte_carlo_integration)
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

            println("vehicle_index: ", vehicle_index)
            println("s: ", Δlo_s, "  ", Δhi_s)
            println("t: ", Δlo_t, "  ", Δhi_t)
            println("v: ", Δlo_v, "  ", Δhi_v)
            println("ϕ: ", Δlo_ϕ, "  ", Δhi_ϕ)

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