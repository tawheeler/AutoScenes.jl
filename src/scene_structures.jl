
type LeadFollowRelationships # needed for bounds
    index_fore::Vector{Int}
    index_rear::Vector{Int}
end

immutable FactorAssignment
    form::Int
    vehicle_indeces::Vector{Int}
end
type SceneStructure # a Factor Graph
    factor_assignments::Vector{FactorAssignment}
    active_vehicles::Set{Int} # set of vehicles that can be manipulated (vehicle index)
    lead_follow::LeadFollowRelationships
end

function get_vehicle_indeces(structure::SceneStructure)
    vehicle_indeces = deepcopy(structure.active_vehicles)
    for i in structure.lead_follow.index_fore
        if i != 0
            push!(vehicle_indeces, i)
        end
    end
    for i in structure.lead_follow.index_rear
        if i != 0
            push!(vehicle_indeces, i)
        end
    end
    sort!(collect(vehicle_indeces))
end

_ordered_tup_pair(a::Int, b::Int) = a < b ? (a,b) : (b,a)
function _try_to_add_factor(vindA::Int, vindB::Int, form, factor_assignments, pair_factors_added)
    if vindA != 0 && vindB != 0
        tup = _ordered_tup_pair(vindA, vindB)
        if !in(tup, pair_factors_added)
            push!(factor_assignments, FactorAssignment(form, Int[vindA, vindB]))
            push!(pair_factors_added, tup)
        end
    end
    nothing
end
function gen_scene_structure(
    scene::Scene,
    roadway::Roadway,
    factors::Vector{SharedFactor},
    vehicle_indeces::AbstractVector{Int}=1:length(scene),
    )

    nvehicles = length(scene)
    factor_assignments = FactorAssignment[]
    active_vehicles = Set{Int}()
    lead_follow = LeadFollowRelationships(zeros(Int, nvehicles), zeros(Int, nvehicles))

    F = VehicleTargetPointFront()
    R = VehicleTargetPointRear()

    # to check for pairs already added
    pair_factors_added = Set{Tuple{Int, Int}}() # (a,b) s.t. a < b

    # add all road factors
    for vehicle_index in vehicle_indeces

        veh_fore_index = get_neighbor_fore_along_lane(scene, vehicle_index, roadway, F, R, F).ind
        veh_rear_index = get_neighbor_rear_along_lane(scene, vehicle_index, roadway, R, F, R).ind

        lead_follow.index_fore[vehicle_index] = veh_fore_index
        lead_follow.index_rear[vehicle_index] = veh_rear_index

        if veh_fore_index != 0 && veh_rear_index != 0
            push!(factor_assignments, FactorAssignment(FeatureForms.ROAD, Int[vehicle_index]))
            push!(active_vehicles, vehicle_index)

            _try_to_add_factor(vehicle_index, veh_fore_index, FeatureForms.FOLLOW, factor_assignments, pair_factors_added)
            _try_to_add_factor(veh_rear_index, vehicle_index, FeatureForms.FOLLOW, factor_assignments, pair_factors_added)
        end
    end

    # add all neighbor features
    for vehicle_index in active_vehicles
        _try_to_add_factor(vehicle_index, get_neighbor_fore_along_right_lane(scene, vehicle_index, roadway, F, R, F).ind, FeatureForms.NEIGHBOR, factor_assignments, pair_factors_added)
        _try_to_add_factor(vehicle_index, get_neighbor_rear_along_right_lane(scene, vehicle_index, roadway, R, F, R).ind, FeatureForms.NEIGHBOR, factor_assignments, pair_factors_added)
        _try_to_add_factor(vehicle_index, get_neighbor_fore_along_left_lane( scene, vehicle_index, roadway, F, R, F).ind, FeatureForms.NEIGHBOR, factor_assignments, pair_factors_added)
        _try_to_add_factor(vehicle_index, get_neighbor_rear_along_left_lane( scene, vehicle_index, roadway, R, F, R).ind, FeatureForms.NEIGHBOR, factor_assignments, pair_factors_added)
    end

    SceneStructure(factor_assignments, active_vehicles, lead_follow)
end

function evaluate_dot!(
    structure::SceneStructure,
    factors::Vector{SharedFactor},
    scene::Scene,
    roadway::Roadway,
    rec::SceneRecord,
    )

    # NOTE: this will call extract!

    retval = 0.0
    for i in 1 : length(structure.factor_assignments)

        fa = structure.factor_assignments[i]
        ϕ = factors[fa.form]

        extract!(ϕ, scene, roadway, fa.vehicle_indeces)
        retval += evaluate_dot(ϕ)
    end
    retval
end