"""
Extract the subscene consisting of all vehicles whose centers are inside of the oriented bounding box
"""
function extract_subscene!(
    subscene::EntityFrame{VehicleState, BoundingBoxDef, Int},
    scene::EntityFrame{VehicleState, BoundingBoxDef, Int},
    region::OBB,
    )

    empty!(subscene)
    for veh in scene
        P = VecE2(get_center(veh))
        if contains(region, P)
            push!(subscene, veh)
        end
    end
    return subscene
end

"""
Returns the number of vehicles that lie upstream, in, and downstream of the scene
"""
function get_num_vehicles_upstream_in_and_downstream(scene::EntityFrame{VehicleState, BoundingBoxDef, Int}, region::OBB)

    n_upstream = 0
    n_in = 0
    n_downstream = 0

    for veh in scene
        C = VecSE2(region.aabb.center, region.θ)
        pos_rel = inertial2body(veh.state.posG, C) # vehicle in the body frame
        if pos_rel.x > region.aabb.len/2 # downstream
            n_downstream += 1
        elseif pos_rel.x < -region.aabb.len/2 # upstream
            n_upstream += 1
        elseif contains(region, VecE2(get_center(veh))) # in
            # NOTE: some vehicles may be alongside the region, which we do not want to count
            n_in += 1
        end
    end

    return (n_upstream, n_in, n_downstream)
end

