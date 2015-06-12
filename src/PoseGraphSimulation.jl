### Module for Simulating Pose Graphs ###
### Ted Steiner, MIT AeroAstro/CSAIL ###
### August 2014 - June 2015 ###

module PoseGraphSimulation

import Graphs
import Winston
import Geodesy
import OpenStreetMap

export Pose2D,
       Meas2D,
       BetweenFactor,
       PosegraphData
       
export makePoseGraph
export getPoses2D, getPoseOdometry, getMeasurements, getLoopFactors, getPriorMeasurements
export getWaypoints, computeRoute, interpolateRoute!, distanceFromStart
export posegraphError, epsilonTrajectory, errorRatio, errorRatios, alignPoses!
export plotPoseGraph, plotPoseGraphTruth, plotEpsilons
export readiSAMOutput, readiSAMTruth, readiSAMOutputCov
export writeMeasurements, writeTruthFile
export isam

include("posegraph_types.jl")
include("posegraph_routing.jl")
include("posegraph_gen.jl")
include("posegraph_meas.jl")
include("posegraph_io.jl")
include("posegraph_error.jl")
include("posegraph_plot.jl")
include("posegraph_solve.jl")

end


