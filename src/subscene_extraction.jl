type SubSceneExtractParams
    center::VecSE2 # center position of scene
    length::Float64
    box::ConvexPolygon

    function SubSceneExtractParams(
        center::VecSE2,
        length::Float64=100.0, # distance along scene orientation to extend the scene to [ft]
        width::Float64=50.0, # distance to either side to extend the scene to [ft]
        )

        x = polar(length/2, center.θ)
        y = polar(width/2, center.θ+π/2)

        o = convert(VecE2, center)
        box = ConvexPolygon(4)
        push!(box, o + x - y)
        push!(box, o + x + y)
        push!(box, o - x + y)
        push!(box, o - x - y)

        AutomotiveDrivingModels.ensure_pts_sorted_by_min_polar_angle!(box)

        new(center, length, box)
    end
end

"""
at least one vehicle has passed the scene and at least one vehicle has yet to enter it
"""
function is_in_bounds(scene::Scene, scene_params::SubSceneExtractParams)

    max_dist_front = 0.0
    max_dist_rear = 0.0

    for i in 1 : length(scene)
        veh = scene.vehicles[i]
        p_rel = inertial2body(veh.state.posG, scene_params.center)
        max_dist_front = max(max_dist_front, p_rel.x + veh.def.length/2)
        max_dist_rear = min(max_dist_rear, p_rel.x - veh.def.length/2)
    end

    max_dist_front ≥ scene_params.length/2 &&
    -max_dist_rear ≥ scene_params.length/2
end

"""
True if there is is always headway separation between all vehicles
"""
function is_there_longitudinal_room(scene::Scene, roadway::Roadway, vehicle_indeces::AbstractVector{Int}=1:length(scene))

    F = VehicleTargetPointFront()
    R = VehicleTargetPointRear()

    for vehicle_index in vehicle_indeces

        veh = scene[vehicle_index]

        res_fore = get_neighbor_fore_along_lane(scene, vehicle_index, roadway, F, R, F)
        if res_fore.ind != 0 && res_fore.Δs < 0.0
            return false
        end

        res_rear = get_neighbor_rear_along_lane(scene, vehicle_index, roadway, R, F, R)
        if res_rear.ind != 0 && res_rear.Δs < 0.0
            return false
        end
    end

    true
end

function is_scene_well_behaved(
    scene::Scene,
    roadway::Roadway,
    scene_params::SubSceneExtractParams,
    mem::CPAMemory=CPAMemory();
    check_is_in_bounds::Bool=true,
    check_longitudinal_room::Bool=true,
    )

    if check_is_in_bounds && !is_in_bounds(scene, scene_params)
        return false
    elseif check_longitudinal_room && !is_there_longitudinal_room(scene, roadway)
        return false
    end

    true
end


function pull_subscene(scene::Scene, scene_params::SubSceneExtractParams)

    vehicle_indeces = Int[]

    for (i,veh) in enumerate(scene)
        if contains(scene_params.box, veh.state.posG)
            push!(vehicle_indeces, i)
        end
    end

    vehicle_indeces
end