type LeadFollowRelationships # needed for bounds
    index_fore::Vector{Int}
    index_rear::Vector{Int}
end
function LeadFollowRelationships(scene::Scene, roadway::Roadway, vehicle_indices::AbstractVector{Int} = 1:length(scene))

    nvehicles = length(scene)
    index_fore = zeros(Int, nvehicles)
    index_rear = zeros(Int, nvehicles)

    F = VehicleTargetPointFront()
    R = VehicleTargetPointRear()

    for vehicle_index in vehicle_indices
        index_fore[vehicle_index] = get_neighbor_fore_along_lane(scene, vehicle_index, roadway, F, R, F).ind
        index_rear[vehicle_index] = get_neighbor_rear_along_lane(scene, vehicle_index, roadway, R, F, R).ind
    end

    return LeadFollowRelationships(index_fore, index_rear)
end

function get_active_vehicles(lead_follow::LeadFollowRelationships)
    active_vehicles = Set{Int}()
    for i in 1:length(lead_follow.index_fore)
        if lead_follow.index_fore[i] != 0 && lead_follow.index_rear[i] != 0
            push!(active_vehicles, i)
        end
    end
    return active_vehicles
end


"""
    LogLinearSharedFactor

A factor ϕ over a set of random variables is a mapping of those
variables to a non-negative real value.

    ϕ(x) → v ≥ 0

A log-linear factor has the specific form ϕ(x) → exp(θᵀ⋅fs(x)),
where fs(x) is a list of scalar features (or one vector feature function)
"""
immutable LogLinearSharedFactor{F <: Function}
    extract::F # (v, scene, roadway, vehicle_indices::NTuple{Int}) -> nothing; extracts internal values and deposites them in v
    v::Vector{Float64} # values
    θ::Vector{Float64} # weights
end
function AutomotiveDrivingModels.extract!(
    ϕ::LogLinearSharedFactor,
    scene::Scene,
    roadway::Roadway,
    vehicle_indices::NTuple{Int},
    )

    ϕ.extract(ϕ.v, scene, roadway, vehicle_indices)
    return nothing
end
Base.dot(ϕ::LogLinearSharedFactor) = dot(ϕ.v, ϕ.θ)
Base.exp(ϕ::LogLinearSharedFactor) = exp(dot(ϕ))

uses_s{F}(ϕ::LogLinearSharedFactor{F}) = error("uses_s not implemented for LogLinearSharedFactor{F}")
uses_t{F}(ϕ::LogLinearSharedFactor{F}) = error("uses_t not implemented for LogLinearSharedFactor{F}")
uses_v{F}(ϕ::LogLinearSharedFactor{F}) = error("uses_v not implemented for LogLinearSharedFactor{F}")
uses_ϕ{F}(ϕ::LogLinearSharedFactor{F}) = error("uses_ϕ not implemented for LogLinearSharedFactor{F}")

"""
    For a given scene and roadway, return all of the vehicle_indices::NTuple{Int} that are valid for this factor
These values are returned as a Vector{Tuple{Vararg{Int}}}
"""
build_structure{F}(ϕ::LogLinearSharedFactor{F}, scene::Scene, roadway::Roadway, active_vehicles::Set{Int}, lead_follow::LeadFollowRelationships) = error("build_structure not implemented for LogLinearSharedFactor{F}")

type SceneStructure{Factors <: Tuple{Vararg{LogLinearSharedFactor}}} # a Factor Graph
                        # NOTE: hash is based on object pointer, which is what we want
    lead_follow::LeadFollowRelationships
    active_vehicles::Set{Int} # set of vehicles that can be manipulated (vehicle index)
    factor_assignments::Dict{LogLinearSharedFactor, Vector{Tuple{Vararg{Int}}}}
end

function gen_scene_structure{Factors <: Tuple{Vararg{LogLinearSharedFactor}}}(
    scene::Scene,
    roadway::Roadway,
    factors::Factors,
    vehicle_indices::AbstractVector{Int} = 1:length(scene),
    )

    lead_follow = LeadFollowRelationships(scene, roadway, vehicle_indices)
    active_vehicles = get_active_vehicles(lead_follow)

    factor_assignments = Dict{LogLinearSharedFactor, Vector{Tuple{Vararg{Int}}}}
    for ϕ in factors
        factor_assignments[ϕ] = build_structure(ϕ, scene, roadway, active_vehicles, lead_follow)
    end

    SceneStructure(factor_assignments, active_vehicles, lead_follow)
end


#### BASIC

function standardize(x::Real, μ::Real, σ::Real)
    @assert σ > 0
    return (x-μ)/σ
end

function extract_basic_road_feature!(
    v::Vector{Float64},
    scene::Scene,
    roadway::Roadway,
    vehicle_indices::NTuple{Int},
    )

    vehicle_index = vehicle_indices[1]
    v[1] = scene[vehicle_index].state.v
    nothing
end
uses_s(ϕ::LogLinearSharedFactor{extract_basic_road_feature!}) = false
uses_t(ϕ::LogLinearSharedFactor{extract_basic_road_feature!}) = false
uses_v(ϕ::LogLinearSharedFactor{extract_basic_road_feature!}) = true
uses_ϕ(ϕ::LogLinearSharedFactor{extract_basic_road_feature!}) = false
function build_structure(
    ϕ::LogLinearSharedFactor{extract_basic_road_feature!},
    scene::Scene,
    roadway::Roadway,
    active_vehicles::Set{Int},
    lead_follow::LeadFollowRelationships,
    )

    retval = Array(Tuple{Vararg{Int}}, length(active_vehicles))
    for (i,vehicle_index) in enumerate(active_vehicles)
        retval[i] = (vehicle_index,)
    end
    return retval
end

function extract_basic_follow_feature!(
    v::Vector{Float64},
    scene::Scene,
    roadway::Roadway,
    vehicle_indices::NTuple{Int},
    )

    v[1] = veh_fore.state.v - veh_rear.state.v
    nothing
end
uses_s(ϕ::LogLinearSharedFactor{extract_basic_follow_feature!}) = false
uses_t(ϕ::LogLinearSharedFactor{extract_basic_follow_feature!}) = false
uses_v(ϕ::LogLinearSharedFactor{extract_basic_follow_feature!}) = true
uses_ϕ(ϕ::LogLinearSharedFactor{extract_basic_follow_feature!}) = false
function build_structure(
    ϕ::LogLinearSharedFactor{extract_basic_follow_feature!},
    scene::Scene,
    roadway::Roadway,
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