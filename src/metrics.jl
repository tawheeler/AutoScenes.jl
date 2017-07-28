"""
KLDivMetric

A metric which can be extracted and then binned such
that the categorical Kullbeck Leibler divergence can be computed

Each function should implement:
    * f(scene, vehicle_indices, roadway) and produce a FeatureValue
    * assign_metric(f, scene, roadway) and produce a Vector{Assignment} that point to vehicle indices
"""
struct KLDivMetric{F<:Function}
    f::F # val::FeatureValue ← f(scene, vehicle_indices, roadway)
    disc::AbstractDiscretizer
end

function assign_metric end

"""
    get_counts(metric::KLDivMetric, rec::ListRecord)

Returns the counts for the KLDivMetric across a dataset of scenes.
"""
function get_counts{F,S,D,I,R}(metric::KLDivMetric{F}, rec::ListRecord{S,D,I}, roadway::R, assignments::Vector{Vector{Assignment}})

    f, disc = metric.f, metric.disc
    counts = zeros(Int, nlabels(disc))

    for (scene, frame_assignments) in zip(ListRecordFrameIterator(rec), assignments)
        for vehicle_indices in frame_assignments # assignments in scene
            fval = f(scene, vehicle_indices, roadway)
            if is_feature_valid(fval)
                counts[encode(disc, convert(Float64, fval))] += 1
            end
        end
    end

    return counts
end

"""
    kullbeck_leibler_divergence(countsP, countsQ, disc)

Returns the Kullbeck Leibler divergence for the peicewise uniform distributions formed by the discretizer and its counts.
"""
function kullbeck_leibler_divergence(countsP::Vector{Int}, countsQ::Vector{Int}, disc::LinearDiscretizer)

    totP = sum(countsP)
    totQ = sum(countsQ)

    retval = 0.0
    for i in 1 : length(countsP)
        w = binwidth(disc, i)
        P = countsP[i] / totP
        Q = countsQ[i] / totQ
        if P > 0
            retval += P * log(P/Q)
        end
    end
    return retval
end

# scene::Scene, vehicle_indices::Assignment, roadway::MobiusRoadway

# extract_speed(scene::Scene, vehicle_index::Int) = (scene[vehicle_index].state.v, true)
# extract_dcl(scene::Scene, vehicle_index::Int) = (scene[vehicle_index].state.posF.t, true)
# extract_phi(scene::Scene, vehicle_index::Int) = (scene[vehicle_index].state.posF.ϕ, true)
# function extract_timegap(scene::Scene, vehicle_index::Int)
#     ind_fore = get_neighbor_index_fore(scene, vehicle_index)
#     if ind_fore != 0
#         timegap = get_headway_time_between(scene[vehicle_index], scene[ind_fore])
#         @assert(!isnan(timegap))
#         (timegap, true)
#     else
#         (NaN, false)
#     end
# end
# function extract_headway(scene::Scene, vehicle_index::Int)
#     ind_fore = get_neighbor_index_fore(scene, vehicle_index)
#     if ind_fore != 0
#         dist = get_headway_dist_between(scene[vehicle_index], scene[ind_fore])
#         @assert(!isnan(dist))
#         (dist, true)
#     else
#         (NaN, false)
#     end
# end
# function extract_a_req(scene::Scene, vehicle_index::Int)
#     ind_fore = get_neighbor_index_fore(scene, vehicle_index)
#     if ind_fore != 0
#         veh_rear = scene[vehicle_index]
#         veh_fore = scene[ind_fore]

#         sr = veh_rear.state.posF.s
#         vr = veh_rear.state.v
#         sf = veh_fore.state.posF.s
#         vf = veh_fore.state.v

#         dv = vf - vr
#         if dv ≥ 0.0 # they are pulling away; we are good
#             (0.0,true)
#         end

#         dx = sf - sr
#         a_req = -dv*dv / (2dx)
#         (a_req, true)
#     else
#         (NaN, false)
#     end
# end
# function extract_ittc(scene::Scene, vehicle_index::Int)
#     ind_fore = get_neighbor_index_fore(scene, vehicle_index)
#     if ind_fore != 0
#         veh_rear = scene[vehicle_index]
#         veh_fore = scene[ind_fore]

#         sr = veh_rear.state.posF.s
#         vr = veh_rear.state.v
#         sf = veh_fore.state.posF.s
#         vf = veh_fore.state.v

#         dv = vf - vr
#         if dv ≥ 0.0 # they are pulling away; we are good
#             (0.0,true)
#         end

#         dx = sf - sr
#         ittc = -dv/dx
#         (ittc, true)
#     else
#         (NaN, false)
#     end
# end