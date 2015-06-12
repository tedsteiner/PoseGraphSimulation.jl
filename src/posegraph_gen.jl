### Functions to Generate Pose Graphs ###
### Ted Steiner ###
### October 2014 ###


# Generate pose graph measurements from a route, set of poses, and list of loop-closure sites.
function makePoseGraph( nodes::Dict, 
                        anchors::Array{Int,1}, 
                        route::Array{Int,1}, 
                        poses::Vector{Pose2D}, 
                        filename::String="posegraph.txt" )
    
    anchor_sets = Array{Int,1}[]
    push!(anchor_sets,anchors)
    
    filenames = String[]
    push!(filenames,filename)
    
    return makePoseGraph(nodes,anchor_sets,route,poses,filenames)
end


# Generate pose graphs from a single route and set of poses and loop-closures to multiple anchor sets
# Array of filenames should align with array of anchor_sets
function makePoseGraph( nodes::Dict, 
                        anchor_sets::Array{Array{Int,1},1}, 
                        route::Array{Int,1}, 
                        poses::Vector{Pose2D}, 
                        filenames::Array{String,1} )

    odom_inf = diagm([5,5,50])
    loop_inf = diagm([50,50,100])
    odom_cov = inv(odom_inf)
    loop_cov = inv(loop_inf)
    
    odometry = getPoseOdometry(route,poses,odom_cov)
    meas = getMeasurements(route,poses,anchor_sets,odometry,loop_cov)

    for k = 1:length(anchor_sets)
        writeMeasurements(filenames[k],meas[k])
    end
    
    return nothing
end

function makePoseGraph( nodes::Dict, 
                        anchor_sets::Array{Array{Int,1},1}, 
                        route::Array{Int,1}, 
                        poses::Vector{Pose2D},
                        odometry::Vector{BetweenFactor},
                        filenames::Array{String,1} )

    loop_inf = diagm([50,50,100])
    loop_cov = inv(loop_inf)

    meas = getMeasurements(route,poses,anchor_sets,odometry,loop_cov)

    for k = 1:length(anchor_sets)
        writeMeasurements(filenames[k],meas[k])
    end
    
    return nothing
end


function makePoseGraph( anchor_sets::Array{Array{Int,1},1}, 
                        route::Vector{Int}, 
                        poses::Vector{Pose2D},
                        odometry_factors::Vector{BetweenFactor},
                        loop_factors::Dict{(Int,Int),BetweenFactor},
                        prior_meas::Dict{Int,Meas2D},
                        prior_inf::Inf2D,
                        filenames::Vector{String} )
                          
    meas = getMeasurements(route,poses,anchor_sets,odometry_factors,loop_factors,prior_meas,prior_inf)

    for k = 1:length(anchor_sets)
        writeMeasurements(filenames[k],meas[k])
    end
    
    return nothing
end


