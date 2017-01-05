
immutable GraphFeatureTemplate
    form::Int # feature form
    values::Vector{Float64} # [t,v,ϕ] for road, [Δs, Δv] for follow, etc.
    normals::Vector{Normal{Float64}} # mean and stdevs for standardization
    mem::CPAMemory
end
function GraphFeatureTemplate(form::Int, normals::Vector{Normal{Float64}})
    values = Array(Float64, length(normals))
    mem = CPAMemory()
    GraphFeatureTemplate(form, values, normals, mem)
end


immutable GraphFeatureInstance
    form::Int
    index::Int # for discrete values
    exponents::Vector{Int} # exponents for each value
end
GraphFeatureInstance(form::Int, index::Int) = GraphFeatureInstance(form, index, Int[])
GraphFeatureInstance(form::Int, exponents::Vector{Int}) = GraphFeatureInstance(form, 0, exponents)

_standardize(v::Float64, μ::Float64, σ::Float64) = (v-μ)/σ
_standardize(v::Float64, normal::Normal) = (v-normal.μ)/normal.σ
function _set_and_standardize!(template::GraphFeatureTemplate, v::Float64, i::Int)
    template.values[i] = _standardize(v, template.normals[i])
    template
end
function _set_standardize_and_clamp!(template::GraphFeatureTemplate, v::Float64, i::Int)
    template.values[i] = clamp(_standardize(v, template.normals[i]), -1.0, 1.0)
    template
end
