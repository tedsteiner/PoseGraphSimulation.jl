### Types Used with Pose Graphs ###
### Ted Steiner ###
### September 2014 ###

abstract Pose
abstract Measurement
abstract Factor

# 2D Pose
type Pose2D <: Pose
    x
    y
    theta
end


# Measurement between 2D poses
type Meas2D <: Measurement
    dx::Float64
    dy::Float64
    dT::Float64
end


# Values of sqrt information matrix
type Inf2D 
    ff      # forward-forward
    fs
    fr
    ss      # sideward-sideward
    sr
    rr      # rotate-rotate
end


# Data required for BetweenFactor (Pose2d_Pose2d_Factor) in iSAM
type BetweenFactor <: Factor
    s::Int
    t::Int
    meas::Meas2D
    info::Inf2D
    nodeS::Int
    nodeT::Int
end


# Data required for a Pose2d_Factor in iSAM
type PriorFactor <: Factor
    pose::Int
    meas::Meas2D
    info::Inf2D
    node::Int
end


# Data required to generate a posegraph from existing measurements
type PosegraphData
    waypoints::Vector{Int}      # Sequence of node IDs
    route::Vector{Int}          # Sequence of node IDs comprising route between waypoints
    poses::Vector{Pose2D}       # Truth poses
    odometry_factors::Vector{BetweenFactor}     # Odometry measurement factors
    loop_factors::Dict{(Int,Int),BetweenFactor} # Loop-closure measurement factors
    prior_meas::Dict{Int,Meas2D} # Prior measurements to first instance of each anchor in route
    data::Dict{String,Any}      # Any additional information can be stored here
end


### Dijkstra visitors, for use with Graphs.jl
# Terminate when any listed node is found
type EndWhenAnyNode <: Graphs.AbstractDijkstraVisitor
    n::Array{Graphs.KeyVertex{Int}}    # Nodes to find
end

function Graphs.include_vertex!(visitor::EndWhenAnyNode,u,v,d)
    !(v in visitor.n)
end


