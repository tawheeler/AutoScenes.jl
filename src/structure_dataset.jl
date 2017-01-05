immutable SceneSource
    trajdata_index::Int
    frame::Int
end

type SceneStructureDataset
    trajdatas::Vector{Trajdata}
    sources::Vector{SceneSource}
    structures::Vector{SceneStructure}
    factors::Vector{SharedFactor}
end
SceneStructureDataset(factors) = SceneStructureDataset(Trajdata[], SceneSource[], SceneStructure[], factors)

Base.length(dset::SceneStructureDataset) = length(dset.structures)
function nvehicles(dset::SceneStructureDataset)
    count = 0
    for subscene in dset.subscenes
        count += length(subscene.vehicle_indeces)
    end
    count
end

function Base.append!(target::SceneStructureDataset, donor::SceneStructureDataset)

    # NOTE: this assumes the factors are identical

    # only
    trajdata_indeces = Array(Int, length(donor.trajdatas))
    for (i, td) in enumerate(donor.trajdatas)
        j = findfirst(td_target->td_target === td, target.trajdatas)
        if j == 0
            push!(target.trajdatas, td)
            j = length(target.trajdatas)
        end
        trajdata_indeces[i] = j
    end

    sources = deepcopy(donor.sources)
    for i in 1 : length(sources)
        source = sources[i]
        sources[i] = SceneSource(trajdata_indeces[source.trajdata_index], source.frame)
    end
    append!(target.sources, sources)

    append!(target.structures, donor.structures)

    target
end

function get_scene_structure_and_roadway!(scene::Scene, dset::SceneStructureDataset, index::Int)
    subscene = dset.sources[index]
    trajdata = dset.trajdatas[subscene.trajdata_index]
    get!(scene, trajdata, subscene.frame)
    structure = dset.structures[index]
    (scene, structure, trajdata.roadway)
end

function pull_scene_dataset(
    trajdata::Trajdata,
    extractparams::SubSceneExtractParams;

    factors::Vector{SharedFactor}=create_shared_factors(),
    scene::Scene=Scene(),
    max_sample_size::Int = typemax(Int),
    mem::CPAMemory=CPAMemory(),
    frames::AbstractVector{Int} = 1:10:nframes(trajdata),

    check_is_in_bounds::Bool=true,
    check_longitudinal_room::Bool=true,
    )

    sources = SceneSource[]
    structures = SceneStructure[]

    for frame in frames

        get!(scene, trajdata, frame)
        vehicle_indeces = pull_subscene(scene, extractparams)

        if is_scene_well_behaved(scene, trajdata.roadway, extractparams, mem,
                                 check_is_in_bounds=check_is_in_bounds,
                                 check_longitudinal_room=check_longitudinal_room)

            source = SceneSource(1, frame)
            structure = gen_scene_structure(scene, trajdata.roadway, factors, vehicle_indeces)

            if !isempty(structure.factor_assignments) # we can learn from it
                push!(sources, source)
                push!(structures, structure)
            end
        end

        if length(sources) ≥ max_sample_size
            break
        end
    end

    SceneStructureDataset([trajdata], sources, structures, factors)
end