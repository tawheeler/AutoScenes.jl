# immutable PseudolikelihoodPrealloc

#     lnP_tilde_denom_arr_s::Vector{Float64}
#     lnP_tilde_denom_arr_t::Vector{Float64}
#     lnP_tilde_denom_arr_v::Vector{Float64}
#     lnP_tilde_denom_arr_ϕ::Vector{Float64}

#     function PseudolikelihoodPrealloc(n_samples_monte_carlo_integration::Int)
#         new(Array(Float64, n_samples_monte_carlo_integration),
#             Array(Float64, n_samples_monte_carlo_integration),
#             Array(Float64, n_samples_monte_carlo_integration),
#             Array(Float64, n_samples_monte_carlo_integration))
#     end
# end
# function Base.empty!(dat::PseudolikelihoodPrealloc)
#     fill!(dat.lnP_tilde_denom_arr_s, 0.0)
#     fill!(dat.lnP_tilde_denom_arr_t, 0.0)
#     fill!(dat.lnP_tilde_denom_arr_v, 0.0)
#     fill!(dat.lnP_tilde_denom_arr_ϕ, 0.0)
#     dat
# end

# function posF_matches_posG(state::VehicleState, roadway::Roadway)
#     posG = state.posG
#     posF = get_posG(state.posF, roadway)
#     abs(posG - posF) < 0.15
# end

function get_relative_variable_bounds_s(scene::Scene, structure::SceneStructure, roadway::Roadway, vehicle_index::Int)
    veh_rear = scene[vehicle_index]
    veh_fore = scene[structure.lead_follow.index_fore[vehicle_index]]
    relpos = get_frenet_relative_position(veh_fore, veh_rear, roadway)
    Δs_fore = relpos.Δs - veh_rear.def.length/2 -  veh_fore.def.length/2
    # @assert !isnan(Δs_fore)
    if isnan(Δs_fore)
        Δs_fore = 0.0
    end

    veh_rear = scene[structure.lead_follow.index_rear[vehicle_index]]
    veh_fore = scene[vehicle_index]
    relpos = get_frenet_relative_position(veh_fore, veh_rear, roadway)
    Δs_rear = relpos.Δs - veh_rear.def.length/2 -  veh_fore.def.length/2
    # @assert !isnan(Δs_rear)
    if isnan(Δs_rear)
        Δs_rear = 0.0
    end

    Δs_rear = 1.0 # DEBUG
    Δs_fore = 1.0 # DEBUG
    (-Δs_rear, Δs_fore)
end
function get_relative_variable_bounds_t(scene::Scene, roadway::Roadway, vehicle_index::Int)
    d_left = get_markerdist_left(scene[vehicle_index], roadway)
    d_right = get_markerdist_right(scene[vehicle_index], roadway)
    (-d_right, d_left)  # NOTE: these are relative to the current t
end
function get_relative_variable_bounds_v(scene::Scene, vehicle_index::Int)
    v = scene[vehicle_index].state.v
    (BOUNDS_V[1]-v, BOUNDS_V[2]-v) # [m/s]
end
function get_relative_variable_bounds_ϕ(scene::Scene, vehicle_index::Int)
    ϕ = scene[vehicle_index].state.posF.ϕ
    (BOUNDS_ϕ[1]-ϕ, BOUNDS_ϕ[2]-ϕ) # [rad]
end

# get_s(veh::Vehicle) = veh.state.posF.s
# get_t(veh::Vehicle) = veh.state.posF.t
# get_v(veh::Vehicle) = veh.state.v
# get_ϕ(veh::Vehicle) = veh.state.posF.ϕ
# function set_s!(veh::Vehicle, state_original::VehicleState, roadway::Roadway, value_delta::Float64)
#     veh.state = move_along(state_original, roadway, value_delta)
#     veh
# end
# function set_t!(veh::Vehicle, state_original::VehicleState, roadway::Roadway, value_delta::Float64)
#     f = state_original.posF
#     new_frenet = Frenet(f.roadind, f.s, f.t+value_delta, f.ϕ)
#     veh.state = VehicleState(new_frenet, roadway, state_original.v)
#     veh
# end
function set_v!(veh::Vehicle, state_original::VehicleState, roadway::Roadway, value_delta::Float64)
    veh.state = VehicleState(state_original.posG, state_original.posF, state_original.v+value_delta)
    veh
end
# function set_ϕ!(veh::Vehicle, state_original::VehicleState, roadway::Roadway, value_delta::Float64)
#     g = state_original.posG
#     new_posG = VecSE2(g.x, g.y, g.θ + value_delta)

#     f = state_original.posF
#     new_posF = Frenet(f.roadind, f.s, f.t, f.ϕ+value_delta)

#     veh.state = VehicleState(new_posG, new_posF, state_original.v)
#     veh
# end

# # TODO: generate with a macro?
# function _update_lnP_tilde_denom_arr_s!(
#     dat::PseudolikelihoodPrealloc,
#     ϕ::SharedFactor,
#     scene::Scene,
#     roadway::Roadway,
#     vehicle_index::Int,
#     vehicle_indeces::Vector{Int},
#     Δlo::Float64,
#     Δhi::Float64,
#     nsamples::Int,
#     )

#     ###########################
#     # v -> lnP(v | ~)
#     #   vary v and compute the denominator term, ∫ᵥ p(v | ~) dv
#     #   use Monte Carlo integration to get ∫ₐ P(a , x₋ⱼ) ≈ V/N ∑ₐ P(a , x₋ⱼ) where V is domain width

#     # get original state
#     veh = scene[vehicle_index]
#     state_original = veh.state

#     Δv = Δlo
#     Δval = (Δhi - Δlo) / (nsamples+1)
#     for k in 1 : nsamples
#         set_s!(veh, state_original, roadway, Δv)
#         extract!(ϕ, scene, roadway, vehicle_indeces)
#         dat.lnP_tilde_denom_arr_s[k] += evaluate_dot(ϕ)
#         Δv += Δval
#     end

#     # reset original state
#     scene[vehicle_index].state = state_original

#     dat
# end
# function _update_lnP_tilde_denom_arr_t!(
#     dat::PseudolikelihoodPrealloc,
#     ϕ::SharedFactor,
#     scene::Scene,
#     roadway::Roadway,
#     vehicle_index::Int,
#     vehicle_indeces::Vector{Int},
#     Δlo::Float64,
#     Δhi::Float64,
#     nsamples::Int,
#     )

#     ###########################
#     # v -> lnP(v | ~)
#     #   vary v and compute the denominator term, ∫ᵥ p(v | ~) dv
#     #   use Monte Carlo integration to get ∫ₐ P(a , x₋ⱼ) ≈ V/N ∑ₐ P(a , x₋ⱼ) where V is domain width

#     # reset original state
#     veh = scene[vehicle_index]
#     state_original = veh.state

#     Δv = Δlo
#     Δval = (Δhi - Δlo) / (nsamples+1)
#     for k in 1 : nsamples
#         set_t!(veh, state_original, roadway, Δv)
#         extract!(ϕ, scene, roadway, vehicle_indeces)
#         dat.lnP_tilde_denom_arr_t[k] += evaluate_dot(ϕ)
#         Δv += Δval
#     end

#     # reset original state
#     scene[vehicle_index].state = state_original

#     dat
# end
# function _update_lnP_tilde_denom_arr_v!(
#     dat::PseudolikelihoodPrealloc,
#     ϕ::SharedFactor,
#     scene::Scene,
#     roadway::Roadway,
#     vehicle_index::Int,
#     vehicle_indeces::Vector{Int},
#     Δlo::Float64,
#     Δhi::Float64,
#     nsamples::Int,
#     )

#     ###########################
#     # v -> lnP(v | ~)
#     #   vary v and compute the denominator term, ∫ᵥ p(v | ~) dv
#     #   use Monte Carlo integration to get ∫ₐ P(a , x₋ⱼ) ≈ V/N ∑ₐ P(a , x₋ⱼ) where V is domain width

#     # reset original state
#     veh = scene[vehicle_index]
#     state_original = veh.state

#     Δv = Δlo
#     Δval = (Δhi - Δlo) / (nsamples+1)
#     for k in 1 : nsamples
#         set_v!(veh, state_original, roadway, Δv)
#         extract!(ϕ, scene, roadway, vehicle_indeces)
#         dat.lnP_tilde_denom_arr_v[k] += evaluate_dot(ϕ)
#         Δv += Δval
#     end

#     # reset original state
#     scene[vehicle_index].state = state_original

#     dat
# end
# function _update_lnP_tilde_denom_arr_ϕ!(
#     dat::PseudolikelihoodPrealloc,
#     ϕ::SharedFactor,
#     scene::Scene,
#     roadway::Roadway,
#     vehicle_index::Int,
#     vehicle_indeces::Vector{Int},
#     Δlo::Float64,
#     Δhi::Float64,
#     nsamples::Int,
#     )

#     ###########################
#     # v -> lnP(v | ~)
#     #   vary v and compute the denominator term, ∫ᵥ p(v | ~) dv
#     #   use Monte Carlo integration to get ∫ₐ P(a , x₋ⱼ) ≈ V/N ∑ₐ P(a , x₋ⱼ) where V is domain width

#     # reset original state
#     veh = scene[vehicle_index]
#     state_original = veh.state
#     f = state_original.posF

#     Δv = Δlo
#     Δval = (Δhi - Δlo) / (nsamples+1)
#     for k in 1 : nsamples
#         set_ϕ!(veh, state_original, roadway, Δv)
#         extract!(ϕ, scene, roadway, vehicle_indeces)
#         dat.lnP_tilde_denom_arr_ϕ[k] += evaluate_dot(ϕ)
#         Δv += Δval
#     end

#     # reset original state
#     scene[vehicle_index].state = state_original

#     dat
# end

# # TODO: generate with a macro?
# function _get_logP_tilde_denom_s(
#     dat::PseudolikelihoodPrealloc,
#     scene::Scene,
#     vehicle_index::Int,
#     Δlo::Float64,
#     Δhi::Float64,
#     nsamples::Int,
#     )

#     P_tilde_denom = 0.0
#     volume = Δhi - Δlo
#     for k in 1 : nsamples
#         P_tilde_denom += exp(dat.lnP_tilde_denom_arr_s[k])
#     end

#     log(P_tilde_denom * volume/nsamples)
# end
# function _get_logP_tilde_denom_t(
#     dat::PseudolikelihoodPrealloc,
#     scene::Scene,
#     vehicle_index::Int,
#     Δlo::Float64,
#     Δhi::Float64,
#     nsamples::Int,
#     )

#     P_tilde_denom = 0.0
#     volume = Δhi - Δlo
#     for k in 1 : nsamples
#         P_tilde_denom += exp(dat.lnP_tilde_denom_arr_t[k])
#     end

#     log(P_tilde_denom * volume/nsamples)
# end
# function _get_logP_tilde_denom_v(
#     dat::PseudolikelihoodPrealloc,
#     scene::Scene,
#     vehicle_index::Int,
#     Δlo::Float64,
#     Δhi::Float64,
#     nsamples::Int,
#     )

#     P_tilde_denom = 0.0
#     volume = Δhi - Δlo
#     for k in 1 : nsamples
#         P_tilde_denom += exp(dat.lnP_tilde_denom_arr_v[k])
#     end

#     log(P_tilde_denom * volume/nsamples)
# end
# function _get_logP_tilde_denom_ϕ(
#     dat::PseudolikelihoodPrealloc,
#     scene::Scene,
#     vehicle_index::Int,
#     Δlo::Float64,
#     Δhi::Float64,
#     nsamples::Int,
#     )

#     P_tilde_denom = 0.0
#     volume = Δhi - Δlo
#     for k in 1 : nsamples
#         P_tilde_denom += exp(dat.lnP_tilde_denom_arr_ϕ[k])
#     end

#     log(P_tilde_denom * volume/nsamples)
# end

immutable VehicleBounds
    Δlo_s::Float64
    Δhi_s::Float64
    Δlo_t::Float64
    Δhi_t::Float64
    Δlo_v::Float64
    Δhi_v::Float64
    Δlo_ϕ::Float64
    Δhi_ϕ::Float64
end
domain_size_s(bounds::VehicleBounds) = bounds.Δhi_s - bounds.Δlo_s
domain_size_t(bounds::VehicleBounds) = bounds.Δhi_t - bounds.Δlo_t
domain_size_v(bounds::VehicleBounds) = bounds.Δhi_v - bounds.Δlo_v
domain_size_ϕ(bounds::VehicleBounds) = bounds.Δhi_ϕ - bounds.Δlo_ϕ

# function calc_pseudolikelihood(scene::Scene, structure::SceneStructure, roadway::Roadway, vehicle_index::Int, bounds::VehicleBounds)

#     retval = 0.0
#     lnP_tilde = 0.0

#     for (ϕ, assignments) in structure.factor_assignments

#         for assignment in assignments
#             # TODO: pre-extract?
#             extract!(ϕ, scene, roadway, assignment)
#             fd = dot(ϕ)
#             lnP_tilde += fd

#             #=
#             compute \log \tilde{p}(x_j ∣x_{-j})

#             \tilde{p}(x_j ∣x_{-j}) is equal to \tilde{p}(x) / int_a \tilde{p}(a, x_{-j}) da
#             The denominator is computed using the MC integration

#             \log \tilde{p}(x_j ∣x_{-j}) is equal to

#                 \log \tilde{p}(x) - \log denominator
#             =#

#             if vehicle_index in assignment # this factor affects the vehicle

#                 target_index = findfirst(assignment, vehicle_index)
#                 @assert(target_index > 0)

#                 # TODO: can I use a macro to generate this for specific factors?

#             #     if USE_S; _update_lnP_tilde_denom_arr_s!(dat, ϕ, scene, roadway, vehicle_index, fa.vehicle_indeces, Δlo_s, Δhi_s, nsamples); end
#             #     if USE_T; _update_lnP_tilde_denom_arr_t!(dat, ϕ, scene, roadway, vehicle_index, fa.vehicle_indeces, Δlo_t, Δhi_t, nsamples); end
#                 # if USE_V; _update_lnP_tilde_denom_arr_v!(dat, ϕ, scene, roadway, vehicle_index, fa.vehicle_indeces, Δlo_v, Δhi_v, nsamples); end
#             #     if USE_ϕ; _update_lnP_tilde_denom_arr_ϕ!(dat, ϕ, scene, roadway, vehicle_index, fa.vehicle_indeces, Δlo_ϕ, Δhi_ϕ, nsamples); end

#                 if uses_v(ϕ)

#                 else

#                 end

#             else
#                 #=
#                 If vehicle does not affect the factor, then the denominator is A, the size of the domain of x_j,
#                 and we get \tilde{p}(x_j ∣x_{-j}) = 1/A
#                 so    \log \tilde{p}(x_j ∣x_{-j}) = -log(A)

#                 =#

#                 A = domain_size_v(bounds) # TODO: move to outside of loop
#                 retval -= log(A)
#             end
#         end
#     end

#     # if USE_S; retval += lnP_tilde - _get_logP_tilde_denom_s(dat, scene, vehicle_index, Δlo_s, Δhi_s, nsamples); end
#     # if USE_T; retval += lnP_tilde - _get_logP_tilde_denom_t(dat, scene, vehicle_index, Δlo_t, Δhi_t, nsamples); end
#     # if USE_V; retval += lnP_tilde - _get_logP_tilde_denom_v(dat, scene, vehicle_index, Δlo_v, Δhi_v, nsamples); end
#     # if USE_ϕ; retval += lnP_tilde - _get_logP_tilde_denom_ϕ(dat, scene, vehicle_index, Δlo_ϕ, Δhi_ϕ, nsamples); end

#     return retval
# end

# function calc_pseudolikelihood(dset::SceneStructureDataset;
#     dat::PseudolikelihoodPrealloc=PseudolikelihoodPrealloc(10),
#     scene::Scene = Scene(),
#     rec::SceneRecord = SceneRecord(1, NaN),
#     )

#     retval = 0.0
#     M = length(dset)
#     nsamples = length(dat.lnP_tilde_denom_arr_s) # n_samples_monte_carlo_integration

#     for m in 1 : M

#         scene, structure, roadway = get_scene_structure_and_roadway!(scene, dset, m)
#         update!(rec, scene)

#         for vehicle_index in structure.active_vehicles

#             lnP_tilde = 0.0
#             empty!(dat)


#             Δlo_s, Δhi_s = get_relative_variable_bounds_s(scene, structure, roadway, vehicle_index)
#             Δlo_t, Δhi_t = get_relative_variable_bounds_t(rec, roadway, vehicle_index)
#             Δlo_v, Δhi_v = get_relative_variable_bounds_v(scene, vehicle_index)
#             Δlo_ϕ, Δhi_ϕ = get_relative_variable_bounds_ϕ(scene, vehicle_index)

#             if Δhi_s - Δlo_s < 0.0
#                 println("m: ", m)
#                 println("vehicle_index: ", vehicle_index)
#                 println("s: ", Δlo_s, "  ", Δhi_s)
#             end
#             if Δhi_t - Δlo_t < 0.0
#                 println("t: ", Δlo_t, "  ", Δhi_t)
#             end
#             if Δhi_v - Δlo_v < 0.0
#                 println("v: ", Δlo_v, "  ", Δhi_v)
#             end
#             if Δhi_ϕ - Δlo_ϕ < 0.0
#                 println("ϕ: ", Δlo_ϕ, "  ", Δhi_ϕ)
#             end

#             for fa in structure.factor_assignments

#                 ϕ = dset.factors[fa.form]
#                 extract!(ϕ, scene, roadway, fa.vehicle_indeces)
#                 fd = evaluate_dot(ϕ)
#                 lnP_tilde += fd

#                 if vehicle_index in fa.vehicle_indeces # this factor affects the vehicle

#                     target_index = findfirst(fa.vehicle_indeces, vehicle_index)
#                     @assert(target_index > 0)

#                     if USE_S; _update_lnP_tilde_denom_arr_s!(dat, ϕ, scene, roadway, vehicle_index, fa.vehicle_indeces, Δlo_s, Δhi_s, nsamples); end
#                     if USE_T; _update_lnP_tilde_denom_arr_t!(dat, ϕ, scene, roadway, vehicle_index, fa.vehicle_indeces, Δlo_t, Δhi_t, nsamples); end
#                     if USE_V; _update_lnP_tilde_denom_arr_v!(dat, ϕ, scene, roadway, vehicle_index, fa.vehicle_indeces, Δlo_v, Δhi_v, nsamples); end
#                     if USE_ϕ; _update_lnP_tilde_denom_arr_ϕ!(dat, ϕ, scene, roadway, vehicle_index, fa.vehicle_indeces, Δlo_ϕ, Δhi_ϕ, nsamples); end

#                 else # this factor does not affect the vehicle
#                     for k in 1 : nsamples
#                         dat.lnP_tilde_denom_arr_s[k] += fd
#                         dat.lnP_tilde_denom_arr_t[k] += fd
#                         dat.lnP_tilde_denom_arr_v[k] += fd
#                         dat.lnP_tilde_denom_arr_ϕ[k] += fd
#                     end
#                 end
#             end

#             if USE_S; retval += lnP_tilde - _get_logP_tilde_denom_s(dat, scene, vehicle_index, Δlo_s, Δhi_s, nsamples); end
#             if USE_T; retval += lnP_tilde - _get_logP_tilde_denom_t(dat, scene, vehicle_index, Δlo_t, Δhi_t, nsamples); end
#             if USE_V; retval += lnP_tilde - _get_logP_tilde_denom_v(dat, scene, vehicle_index, Δlo_v, Δhi_v, nsamples); end
#             if USE_ϕ; retval += lnP_tilde - _get_logP_tilde_denom_ϕ(dat, scene, vehicle_index, Δlo_ϕ, Δhi_ϕ, nsamples); end
#         end
#     end
#     retval /= M

#     if isnan(retval) || isinf(retval)
#         retval = 0.0
#     end

#     retval
# end

# # TODO: generate this with a macro?
# function calc_pseudolikelihood_gradient_component_s(
#     ϕ::SharedFactor,
#     factors::Vector{SharedFactor},
#     scene::Scene,
#     structure::SceneStructure,
#     roadway::Roadway,
#     vehicle_index::Int,
#     vehicle_indeces::Vector{Int},
#     target_instance::GraphFeatureInstance,
#     n_samples_monte_carlo_integration::Int,
#     rng::AbstractRNG,
#     rec::SceneRecord,
#     )

#     Δlo, Δhi = get_relative_variable_bounds_s(scene, structure, roadway, vehicle_index)
#     volume = Δhi - Δlo

#     # get original state
#     veh = scene[vehicle_index]
#     state_original = veh.state

#     # compute expectation term
#     #   evaluated using Importance Sampling
#     E_numerator = 0.0
#     E_denominator = 0.0
#     for r in 1 : n_samples_monte_carlo_integration

#         set_s!(veh, state_original, roadway, volume*rand(rng) + Δlo)
#         extract!(ϕ, scene, roadway, vehicle_indeces)
#         f = evaluate(ϕ.template, target_instance)
#         p_true = exp(evaluate_dot!(structure, factors, scene, roadway, rec, vehicle_index))

#         E_numerator += f*p_true
#         E_denominator += p_true
#     end

#     # reset original state
#     scene[vehicle_index].state = state_original

#     E = E_numerator / E_denominator

#     if isnan(E) || isinf(E)
#         E = randn()
#     end

#     -E
# end

function pdf_tilde(factors, # TODO: type
    scene::Scene,
    structure::SceneStructure,
    roadway::Roadway,
    )

    retval = 0.0
    for (ϕ, assignments) in structure.factor_assignments
        for assignment in assignments
            extract!(ϕ, scene, roadway, assignment)
            retval += dot(ϕ)
        end
    end

    return exp(retval)
end
function pdf_tilde_only_for_vehicle(factors, # TODO: type
    scene::Scene,
    structure::SceneStructure,
    roadway::Roadway,
    vehicle_index::Int,
    )

    retval = 0.0
    for (ϕ, assignments) in structure.factor_assignments
        for assignment in assignments
            if vehicle_index ∈ assignment
                extract!(ϕ, scene, roadway, assignment)
                retval += dot(ϕ)
            end
        end
    end

    return exp(retval)
end

function estimate_expectation_of_f_over_speed(
    ϕ::LogLinearSharedFactor,
    scene::Scene,
    structure::SceneStructure,
    roadway::Roadway,
    assignment::Tuple{Vararg{Int}},
    ego_index_in_assignment::Int,
    bounds::VehicleBounds,
    nsamples::Int,
    )

    # TODO: move allocation out
    numerator = zeros(Float64, length(ϕ.θ))
    denominator = zeros(Float64, length(ϕ.θ))

    # proposal distribution
    U = Uniform(bounds.Δlo_v, bounds.Δhi_v)

    veh_index = assignment[ego_index_in_assignment]
    veh = scene[veh_index]
    initial_state = veh.state
    for r in 1 : nsamples
        Δv = rand(U)
        set_v!(veh, initial_state, roadway, Δv)
        extract!(ϕ, scene, roadway, assignment)

        # technically, only need to recompute the ones that depend on this vehicle and its speed
        W = pdf_tilde_only_for_vehicle(factors, scene, structure, roadway)

        numerator += ϕ.v .* W
        denominator += W
    end
    veh.state = initial_state

    return E
end

function calc_pseudolikelihood_gradient(ϕ::Factor, scene::Scene, structure::SceneStructure, roadway::Roadway)

    # TODO: move allocation out
    ∇ = zeros(Float64, length(ϕ.θ))

    for assignment in structure.factor_assignments[ϕ]

        extract!(ϕ, scene, roadway, assignment)

        ∇ += (uses_v(ϕ) + uses_s(ϕ) + uses_t(ϕ) + uses_ϕ(ϕ))*ϕ.v

        if uses_v(ϕ)
            #=
            Subtract expectation term
                Compute it using importance sampling w/ a uniform distribution
            =#
            ∇ -= estimate_expectation_of_f_over_speed(ϕ, scene, structure, roadway, assignment, ...)
        end

        # if uses_s
        # if uses_t
        # if uses_ϕ
    end

    return ∇
end

# function calc_pseudolikelihood_gradient_component_t(
#     ϕ::SharedFactor,
#     factors::Vector{SharedFactor},
#     scene::Scene,
#     structure::SceneStructure,
#     roadway::Roadway,
#     vehicle_index::Int,
#     vehicle_indeces::Vector{Int},
#     target_instance::GraphFeatureInstance,
#     n_samples_monte_carlo_integration::Int,
#     rng::AbstractRNG,
#     rec::SceneRecord,
#     )

#     update!(rec, scene)
#     Δlo, Δhi = get_relative_variable_bounds_t(rec, roadway, vehicle_index)
#     volume = Δhi - Δlo

#     # get original state
#     veh = scene[vehicle_index]
#     state_original = veh.state

#     # compute expectation term
#     #   evaluated using Importance Sampling
#     E_numerator = 0.0
#     E_denominator = 0.0
#     for r in 1 : n_samples_monte_carlo_integration

#         set_t!(veh, state_original, roadway, volume*rand(rng) + Δlo)
#         extract!(ϕ, scene, roadway, vehicle_indeces)
#         f = evaluate(ϕ.template, target_instance)
#         p_true = exp(evaluate_dot!(structure, factors, scene, roadway, rec, vehicle_index))

#         E_numerator += f*p_true
#         E_denominator += p_true
#     end

#     # reset original state
#     scene[vehicle_index].state = state_original

#     E = E_numerator / E_denominator

#     # @assert(!isnan(E))
#     # @assert(!isinf(E))

#     if isnan(E) || isinf(E)
#         E = randn()
#     end

#     -E
# end
# function calc_pseudolikelihood_gradient_component_v(
#     ϕ::SharedFactor,
#     factors::Vector{SharedFactor},
#     scene::Scene,
#     structure::SceneStructure,
#     roadway::Roadway,
#     vehicle_index::Int,
#     vehicle_indeces::Vector{Int},
#     target_instance::GraphFeatureInstance,
#     n_samples_monte_carlo_integration::Int,
#     rng::AbstractRNG,
#     rec::SceneRecord,
#     )

#     Δlo, Δhi = get_relative_variable_bounds_v(scene, vehicle_index)
#     volume = Δhi - Δlo

#     # get original state
#     veh = scene[vehicle_index]
#     state_original = veh.state

#     # compute expectation term
#     #   evaluated using Importance Sampling
#     E_numerator = 0.0
#     E_denominator = 0.0
#     for r in 1 : n_samples_monte_carlo_integration

#         set_v!(veh, state_original, roadway, volume*rand(rng) + Δlo)
#         extract!(ϕ, scene, roadway, vehicle_indeces)
#         f = evaluate(ϕ.template, target_instance)
#         p_true = exp(evaluate_dot!(structure, factors, scene, roadway, rec, vehicle_index)) # TODO: speed this up by only re-evaluating the Markov blanket

#         E_numerator += f*p_true
#         E_denominator += p_true
#     end

#     # reset original state
#     scene[vehicle_index].state = state_original

#     E = E_numerator / E_denominator

#     # @assert(!isnan(E))
#     # @assert(!isinf(E))

#     if isnan(E) || isinf(E)
#         # warn("calc_pseudolikelihood_gradient_component_v is $E; $E_numerator, $E_denominator")
#         E = randn()
#     end

#     -E
# end
# function calc_pseudolikelihood_gradient_component_ϕ(
#     ϕ::SharedFactor,
#     factors::Vector{SharedFactor},
#     scene::Scene,
#     structure::SceneStructure,
#     roadway::Roadway,
#     vehicle_index::Int,
#     vehicle_indeces::Vector{Int},
#     target_instance::GraphFeatureInstance,
#     n_samples_monte_carlo_integration::Int,
#     rng::AbstractRNG,
#     rec::SceneRecord,
#     )

#     Δlo, Δhi = get_relative_variable_bounds_ϕ(scene, vehicle_index)
#     volume = Δhi - Δlo

#     # get original state
#     veh = scene[vehicle_index]
#     state_original = veh.state

#     # compute expectation term
#     #   evaluated using Importance Sampling
#     E_numerator = 0.0
#     E_denominator = 0.0
#     for r in 1 : n_samples_monte_carlo_integration

#         set_ϕ!(veh, state_original, roadway, volume*rand(rng) + Δlo)
#         extract!(ϕ, scene, roadway, vehicle_indeces)
#         f = evaluate(ϕ.template, target_instance)
#         p_true = exp(evaluate_dot!(structure, factors, scene, roadway, rec, vehicle_index))

#         E_numerator += f*p_true
#         E_denominator += p_true
#     end

#     # reset original state
#     scene[vehicle_index].state = state_original

#     E = E_numerator / E_denominator

#     if isnan(E) || isinf(E)
#         warn("E is $E")
#         E = randn()
#     end

#     -E
# end

# #####################

# type BatchSampler
#     dset::SceneStructureDataset
#     perm::Array{Int} # permutation over all samples in the dataset
#     perm_index::Int # location in the perm
#     epoch::Int # the current epoch (ie if this is 2 we have already been through dset once and are on our 2nd time through)
# end
# BatchSampler(dset::SceneStructureDataset) = BatchSampler(dset, randperm(length(dset)), 0, 1)

# epoch_size(sampler::BatchSampler) = length(sampler.dset)
# function get_n_samples_used(sampler::BatchSampler)
#     n_epochs = epoch_size(sampler)
#     n_epochs*(sampler.epoch - 1) + sampler.perm_index
# end
# function restart!(sampler::BatchSampler)
#     sampler.perm_index = 0
#     sampler.epoch = 1
#     sampler.perm = randperm(length(sampler.perm))
#     sampler
# end
# function next_index!(sampler::BatchSampler)
#     sampler.perm_index += 1
#     if sampler.perm_index > length(sampler.perm)
#         sampler.perm_index = 1
#         sampler.epoch += 1
#     end
#     sampler.perm[sampler.perm_index]
# end

# immutable SceneStructureRoadway
#     scene::Scene
#     structure::SceneStructure
#     roadway::Roadway
# end
# function next!(sampler::BatchSampler)
#     index = next_index!(sampler)
#     scene, structure, roadway = get_scene_structure_and_roadway!(Scene(), sampler.dset, index)
#     SceneStructureRoadway(scene, structure, roadway)
# end

# #####################

# function calc_pseudolikelihood_gradient(
#     form::Int,
#     feature_index::Int,
#     sampler::BatchSampler,
#     batch_size::Int,
#     n_samples_monte_carlo_integration::Int,
#     regularization::Float64,
#     rng::AbstractRNG=Base.GLOBAL_RNG,
#     scene::Scene = Scene(),
#     rec::SceneRecord = SceneRecord(2, 0.1),
#     factors::Vector{SharedFactor} = sampler.dset.factors,
#     )

#     retval = 0.0
#     ϕ = factors[form]
#     target_instance = ϕ.instances[feature_index]

#     for m in 1 : batch_size

#         structure_index = next_index!(sampler) # sample uniformly at random
#         scene, structure, roadway = get_scene_structure_and_roadway!(scene, sampler.dset, structure_index)

#         for fa in structure.factor_assignments
#             if fa.form == form # is the correct factor

#                 # first component
#                 extract!(ϕ, scene, roadway, fa.vehicle_indeces)
#                 eval = evaluate(ϕ.template, target_instance)

#                 for vehicle_index in fa.vehicle_indeces

#                     # only calc if the vehicle is active
#                     if vehicle_index in structure.active_vehicles

#                         if uses_s(target_instance)
#                             retval += eval
#                             retval += calc_pseudolikelihood_gradient_component_s(ϕ, factors, scene, structure, roadway, vehicle_index, fa.vehicle_indeces,
#                                                                                  target_instance,n_samples_monte_carlo_integration, rng, rec)
#                         end
#                         if uses_t(target_instance)
#                             retval += eval
#                             retval += calc_pseudolikelihood_gradient_component_t(ϕ, factors, scene, structure, roadway, vehicle_index, fa.vehicle_indeces,
#                                                                                  target_instance,n_samples_monte_carlo_integration, rng, rec)
#                         end
#                         if uses_v(target_instance)
#                             retval += eval
#                             retval += calc_pseudolikelihood_gradient_component_v(ϕ, factors, scene, structure, roadway, vehicle_index, fa.vehicle_indeces,
#                                                                                  target_instance,n_samples_monte_carlo_integration, rng, rec)
#                         end
#                         if uses_ϕ(target_instance)
#                             retval += eval
#                             retval += calc_pseudolikelihood_gradient_component_ϕ(ϕ, factors, scene, structure, roadway, vehicle_index, fa.vehicle_indeces,
#                                                                                  target_instance,n_samples_monte_carlo_integration, rng, rec)
#                         end
#                     end
#                 end
#             end
#         end
#     end

#     retval /= batch_size

#     # add regularization
#     retval -= 2*regularization*ϕ.weights[feature_index]

#     retval
# end
# function calc_pseudolikelihood_gradient(
#     form::Int,
#     feature_index::Int,
#     factors::Vector{SharedFactor},
#     samples::Vector{SceneStructureRoadway}, # of length batch_size
#     n_samples_monte_carlo_integration::Int,
#     regularization::Float64,
#     rng::AbstractRNG=Base.GLOBAL_RNG,
#     scene::Scene = Scene(),
#     rec::SceneRecord = SceneRecord(2, 0.1),
#     )

#     retval = 0.0
#     ϕ = factors[form]
#     target_instance = ϕ.instances[feature_index]

#     for ssr in samples

#         scene, structure, roadway = ssr.scene, ssr.structure, ssr.roadway

#         for fa in structure.factor_assignments
#             if fa.form == form # is the correct factor

#                 # first component
#                 extract!(ϕ, scene, roadway, fa.vehicle_indeces)
#                 eval = evaluate(ϕ.template, target_instance)

#                 for vehicle_index in fa.vehicle_indeces

#                     # only calc if the vehicle is active
#                     if vehicle_index in structure.active_vehicles

#                         if uses_s(target_instance)
#                             retval += eval
#                             retval += calc_pseudolikelihood_gradient_component_s(ϕ, factors, scene, structure, roadway, vehicle_index, fa.vehicle_indeces,
#                                                                                  target_instance,n_samples_monte_carlo_integration, rng, rec)
#                         end
#                         if uses_t(target_instance)
#                             retval += eval
#                             retval += calc_pseudolikelihood_gradient_component_t(ϕ, factors, scene, structure, roadway, vehicle_index, fa.vehicle_indeces,
#                                                                                  target_instance,n_samples_monte_carlo_integration, rng, rec)
#                         end
#                         if uses_v(target_instance)
#                             retval += eval
#                             retval += calc_pseudolikelihood_gradient_component_v(ϕ, factors, scene, structure, roadway, vehicle_index, fa.vehicle_indeces,
#                                                                                  target_instance,n_samples_monte_carlo_integration, rng, rec)
#                         end
#                         if uses_ϕ(target_instance)
#                             retval += eval
#                             retval += calc_pseudolikelihood_gradient_component_ϕ(ϕ, factors, scene, structure, roadway, vehicle_index, fa.vehicle_indeces,
#                                                                                  target_instance,n_samples_monte_carlo_integration, rng, rec)
#                         end
#                     end
#                 end
#             end
#         end
#     end

#     retval /= length(samples)

#     # add regularization
#     retval -= 2*regularization*ϕ.weights[feature_index]

#     retval
# end

# #####################

# function reset_weights!(factors::Vector{SharedFactor}, σ::Float64=1.0)
#     for ϕ in factors
#         copy!(ϕ.weights, σ*randn(Float64, length(ϕ.weights)))
#     end
#     factors
# end