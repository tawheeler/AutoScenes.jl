type BatchIterator
    perm::Array{Int} # permutation over all samples in the dataset
    i::Int # running count of number of samples pulled
end

"""
    BatchIterator(m::Int)

Construct a BatchIterator where m is the number of samples, aka the epoch size.
Randomly initializes a permuation.
"""
BatchIterator(m::Int) = BatchIterator(randperm(m), 0)

epoch_size(sampler::BatchIterator) = length(sampler.perm)
samples_so_far(sampler::BatchIterator) = sampler.i
current_epoch(sampler::BatchIterator) = div(sampler.i, epoch_size(sampler)) + 1
get_sample(sampler::BatchIterator, i::Int=sampler.i) = sampler.perm[mod(i-1, epoch_size(sampler))+1]

function Base.clear!(sampler::BatchIterator)
    shuffle!(sampler.perm)
    sampler.i = 0
    return sampler
end
function next_sample!(sampler::BatchIterator)
    sampler.i += 1
    epoch = length(sampler.perm)
    v = get_sample(sampler)
    if v == sampler.perm[end] # last item
        shuffle!(sampler.perm)
    end
    return v
end

function pull_batch!{R}(batch::Vector{FactorGraph{R}}, factorgraphs::Vector{FactorGraph{R}}, sampler::BatchIterator)
    for i in 1 : length(batch)
        batch[i] = factorgraphs[next_sample!(sampler)]
    end
    return batch
end