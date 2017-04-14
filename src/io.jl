# function Base.write(io::IO, ϕ::SharedFactor)
#     println(io, "template:")
#     println(io, "\t", ϕ.template.form)
#     print(io, "\t")
#     for v in ϕ.template.values
#         @printf(io, "%.16e  ", v)
#     end
#     print(io, "\n")
#     print(io, "\t")
#     for v in ϕ.template.normals
#         @printf(io, "%.16e %.16e  ", v.μ, v.σ)
#     end
#     print(io, "\n")

#     println(io, "instances: ", length(ϕ.instances))
#     for (instance, w) in zip(ϕ.instances, ϕ.weights)
#         @printf(io, "\t %.16e %d %d ", w, instance.form, instance.index)
#         for e in instance.exponents
#             print(io, " ", e)
#         end
#         println(io, "")
#     end
# end
# function Base.write(io::IO, factors::Vector{SharedFactor})
#     println(io, "Shared Factors")
#     println(io, length(factors))
#     for ϕ in factors
#         write(io, ϕ)
#     end
# end