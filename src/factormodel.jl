struct FactorModel{F<:Tuple{Vararg{Function}}}
    features::F
    weights::Vector{Float64}
end

function Base.write(io::IO, ::MIME"text/plain", model::FactorModel)
    for (i,f) in enumerate(model.features)
        print(io, f, i == length(model.features) ? "\n" : " ")
    end
    for (i,w) in enumerate(model.weights)
        print(io, w, i == length(model.weights) ? "\n" : " ")
    end
    return nothing
end

macro load_factor_model(filename)
    ex = quote
        open($filename, "r") do io
            features = Tuple(eval(parse(str)) for str in split(readline(io)))
            weights = [parse(Float64, s) for s in split(readline(io))]
            FactorModel(features, weights)
        end
    end
    return esc(ex)
end