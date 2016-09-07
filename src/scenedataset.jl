type SceneDataset
    sources::Vector{SceneSource} # source in the original NGSIM dataset
    states::Vector{TrajdataState} # vehicle states
    frames::Vector{Tuple{Int,Int}} # start and end of state ranges
end
SceneDataset() = SceneDataset(SceneSource[], TrajdataState[], Tuple{Int,Int}[])

Base.length(sdset::SceneDataset) = length(sdset.frames)

function Base.push!(sdset::SceneDataset, scene::Scene, source::SceneSource)
    
    lo = length(sdset.states) + 1
    for veh in scene
        push!(sdset.states, TrajdataState(veh.def.id, veh.state))
    end
    hi = length(sdset.states)

    push!(sdset.frames, (lo, hi))
    push!(sdset.sources, source)

    sdset
end

function Base.write(io::IO, sdset::SceneDataset)
    println(io, "SCENEDATASET")

    # sources
    println(io, length(sdset.sources)) # number of sources
    for source in sdset.sources
        @printf(io, "%d %d\n", source.trajdata_index, source.frame)
    end

    # states
    println(io, length(sdset.states)) # number of states
    for trajdatastate in sdset.states
        state = trajdatastate.state
        @printf(io, "%d (%.4f %.4f %.4e) (%d %.4f %d %d) (%.4f %.4f %.4e) %.4f\n",
                     trajdatastate.id,
                     state.posG.x, state.posG.y, state.posG.θ,
                     state.posF.roadind.ind.i, state.posF.roadind.ind.t,
                     state.posF.roadind.tag.segment, state.posF.roadind.tag.lane,
                     state.posF.s, state.posF.t, state.posF.ϕ,
                     state.v
                )
    end

    # frames
    println(io, length(sdset.frames)) # number of frames (should be same as number of sources)
    for tup in sdset.frames
        @printf(io, "%d %d\n", tup[1], tup[2])
    end
end
function Base.read(io::IO, ::Type{SceneDataset})
    lines = readlines(io)
    line_index = 1
    if contains(lines[line_index], "SCENEDATASET")
        line_index += 1
    end

    function advance!()
        line = strip(lines[line_index])
        line_index += 1
        line
    end

    N = parse(Int, advance!())
    sources = Array(SceneSource, N)
    for i in 1 : N
        tokens = split(advance!(), ' ')
        trajdata_index = parse(Int, tokens[1])
        frame = parse(Int, tokens[2])
        sources[i] = SceneSource(trajdata_index, frame)
    end

    N = parse(Int, advance!())
    states = Array(TrajdataState, N)
    for i in 1 : N
        line = advance!()
        cleanedline = replace(line, r"(\(|\))", "")
        tokens = split(cleanedline, ' ')
        id = parse(Int, tokens[1])
        x = parse(Float64, tokens[2])
        y = parse(Float64, tokens[3])
        θ = parse(Float64, tokens[4])
        ind_i = parse(Int, tokens[5])
        ind_t = parse(Float64, tokens[6])
        seg = parse(Int, tokens[7])
        line = parse(Int, tokens[8])
        s = parse(Float64, tokens[9])
        t = parse(Float64, tokens[10])
        ϕ = parse(Float64, tokens[11])
        v = parse(Float64, tokens[12])
        states[i] = TrajdataState(id, VehicleState(VecSE2(x,y,θ), Frenet(RoadIndex(CurveIndex(ind_i, ind_t), LaneTag(seg, line)), s, t, ϕ), v))
    end

    N = parse(Int, advance!())
    frames = Array(Tuple{Int,Int}, N)
    for i in 1 : N
        tokens = split(advance!(), ' ')
        lo = parse(Int, tokens[1])
        hi = parse(Int, tokens[2])
        frames[i] = (lo, hi)
    end

    SceneDataset(sources, states, frames)
end

function get_scene_and_roadway!(scene::Scene, sdset::SceneDataset, trajdatas::Vector{Trajdata}, scene_index::Int)
    origin = sdset.sources[scene_index]
    trajdata = trajdatas[origin.trajdata_index]
    get!(scene, trajdata, origin.frame)

    frame = sdset.frames[scene_index]
    for (i, j) in enumerate(frame[1] : frame[2])
        scene[i].state = sdset.states[j].state
    end

    (scene, trajdata.roadway)
end