using Base.Test
using AutomotiveDrivingModels
using AutoScenes
using NBInclude

let
    nbinclude(Pkg.dir("AutoScenes", "docs", "OneFloat.ipynb"))
end

let
    nbinclude(Pkg.dir("AutoScenes", "docs", "1DScene.ipynb"))
end

let
    nbinclude(Pkg.dir("AutoScenes", "docs", "OneFloatSampling.ipynb"))
end

let
    nbinclude(Pkg.dir("AutoScenes", "docs", "HeadwaySampling.ipynb"))
end

let
    nbinclude(Pkg.dir("AutoScenes", "docs", "Metrics.ipynb"))
end