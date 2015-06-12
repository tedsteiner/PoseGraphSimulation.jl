### Functions for Converting a Route to a Pose Graph ###
### Ted Steiner ###
### August 2014 ###


# Get truth poses
function getPoses2D( nodes::Dict{Int,OpenStreetMap.ENU}, route )
    poses = Pose2D[]
    
    # Add initial pose
    push!(poses,Pose2D(nodes[route[1]].east,nodes[route[1]].north,0.0))
    
    for k = 1:length(route)-1
        push!(poses,getPose2D(nodes,route[k],route[k+1]))
    end
    
    # Add final pose
    push!(poses,Pose2D(nodes[route[end]].east,nodes[route[end]].north,0.0))
    
    return poses
end


# Pose of node A, using orientation from A -> B
function getPose2D( nodes::Dict{Int,OpenStreetMap.ENU}, A::Int, B::Int )
    theta = atan2(nodes[B].north-nodes[A].north,nodes[B].east-nodes[A].east)
    return Pose2D( nodes[A].east, nodes[A].north, theta )
end


# Generate odometry measurements between poses
function getPoseOdometry( route, poses, noise_odom )
    odom = BetweenFactor[]
    
    info_odom = getInfoMatrix(noise_odom)
    
    for k = 0:length(poses)-2
        s = k
        t = k+1
        nodeT = route[t]

        if k == 0
            push!(odom,BetweenFactor(s,t,Meas2D(0,0,poses[t+1].theta),info_odom,0,nodeT))
        else
            nodeS = route[s]
            push!(odom,BetweenFactor(s,t,getPoseMeas2D(poses[s+1],poses[t+1],noise_odom),info_odom,nodeS,nodeT))
        end
    end
    
    return odom
end


# Combine odometry measurements with loop closures given anchors and poses
function getMeasurements( route, 
                          poses, 
                          anchors::Array{Int,1}, 
                          odometry_factors::Array{BetweenFactor,1}, 
                          noise_loop )

    anchor_sets = Array{Int,1}[]
    push!(anchor_sets,anchors)
    return getMeasurements(route,poses,anchor_sets,odometry_factors,noise_loop)[1]
end


# Generate measurement sets for anchor sets
# Any loop closure measurement present in mulitple measurement sets will have 
# the same noise, enabling comparison of anchor sets
function getMeasurements( route, 
                          poses, 
                          anchors::Array{Array{Int,1},1}, 
                          odometry_factors::Array{BetweenFactor,1}, 
                          noise_loop )

    all_anchors = Int[]
    meas = Array{BetweenFactor,1}[]
    info_loop = getInfoMatrix(noise_loop)

    for a in anchors
        push!(all_anchors,a...)
        push!(meas,BetweenFactor[])
    end
    all_anchors = Set(all_anchors...)

    for k = 0:length(poses)-2
        for a = 1:length(anchors)
            push!(meas[a],odometry_factors[k+1])
        end

        if k > 10
            t = k+1
            nodeT = route[t]
            if nodeT in all_anchors
                for kk = 1:k-5
                    if route[kk] == nodeT
                        # Generate loop-closure constraint
                        s = kk
                        nodeS = route[kk]

                        factor = BetweenFactor(s,t,getAnchorPoseMeas2D(poses[s+1],poses[t+1],noise_loop),info_loop,nodeS,nodeT)

                        for a = 1:length(anchors)
                            if route[t] in anchors[a]
                                push!(meas[a],factor)
                            end
                        end

                        break # Add constraint only to original pose
                    end
                end
            end
        end
    end

    return meas
end

# For use when factors are already computed
function getMeasurements( route, 
                          poses, 
                          anchors::Array{Array{Int,1},1}, 
                          odometry_factors::Array{BetweenFactor,1}, 
                          loop_factors::Dict{(Int,Int),BetweenFactor},
                          prior_meas::Dict{Int,Meas2D}=Dict{Int,Meas2D}(),
                          prior_inf::Inf2D=Inf2D(50,0,0,50,0,100) )

    all_anchors = Int[]
    meas = Array{Factor,1}[]

    for a in anchors
        push!(all_anchors,a...)
        push!(meas,BetweenFactor[])
    end
    all_anchors = Set(all_anchors...)

    for k = 0:length(poses)-2
        # Odometry Factors
        for a = 1:length(anchors)
            push!(meas[a],odometry_factors[k+1])
        end

        if k > 10
            t = k+1
            if route[t] in all_anchors

                # Loop-closure Factors
                loop_found = false
                for s = 1:k-5
                    if route[s] == route[t]
                        loop_found = true
                        # Find and add loop-closure constraint
                        if haskey(loop_factors,(s,t))
                            factor = loop_factors[(s,t)]
                            for a = 1:length(anchors)
                                if route[t] in anchors[a]
                                    push!(meas[a],factor)
                                end
                            end
                        else
                            warn("Loop closure missing between poses $s and $t.")
                        end

                        break # Add constraint only to original pose
                    end
                end

                # Prior Factors: Only if first occurence of this node
                if !loop_found && haskey(prior_meas,route[t])
                    factor = PriorFactor(t,prior_meas[route[t]],prior_inf,route[t])
                    for a = 1:length(anchors)
                        if route[t] in anchors[a]
                            push!(meas[a],factor)
                        end
                    end
                end
            end
        end
    end

    return meas
end

# Generate loop-closure measurements and return as dictionary
# This is used to precompute measurements when the actual anchor sets are not yet known
function getLoopFactors( route,
                         poses, 
                         anchors::Vector{Int}, 
                         noise_loop )

    factors = Dict{(Int,Int),BetweenFactor}()
    info_loop = getInfoMatrix(noise_loop)
    all_anchors = Set(anchors)

    for k = 0:length(poses)-2
        if k > 10
            t = k+1
            nodeT = route[t]
            if nodeT in all_anchors
                for kk = 1:k-5
                    if route[kk] == nodeT
                        # Generate loop-closure constraint
                        s = kk
                        nodeS = route[kk]

                        factors[(s,t)] = BetweenFactor(s,t,getAnchorPoseMeas2D(poses[s+1],poses[t+1],noise_loop),info_loop,nodeS,nodeT)

                        break # Add constraint only to original pose
                    end
                end
            end
        end
    end

    return factors
end


function getPriorMeasurements( route, poses, anchors::Vector{Int}, noise_prior )
    priors = Dict{Int,Meas2D}()
    
    for a in anchors
        for k = 1:length(route)
            if route[k] == a
                priors[a] = getPriorMeas2D(poses[k],noise_prior)
                break
            end
        end
    end
    
    return priors
end

function getPriorMeas2D( pose, meas_cov )
    dx = pose.x
    dy = pose.y
    dt = pose.theta

    m = [dx,dy,dt] + meas_cov*randn(3)
    
    return Meas2D(m[1],m[2],m[3])
end


function getPoseMeas2D( pose0, pose1, meas_cov=zeros(3) )
    dx = sqrt((pose1.x - pose0.x)^2 + (pose1.y - pose0.y)^2)
    dy = 0.0
    dt = pose1.theta - pose0.theta

    m = [dx,dy,dt]
    
    return Meas2D(m[1],m[2],m[3])
end


function getAnchorPoseMeas2D( pose0, pose1, meas_cov=zeros(3) )
    dx = 0.0
    dy = 0.0
    dt = pose1.theta - pose0.theta
    
    m = [dx,dy,dt] + meas_cov*randn(3)
    
    return Meas2D(m[1],m[2],m[3])
end


function getInfoMatrix( noise )
    I = inv(noise)
    return Inf2D(I[1,1],I[1,2],I[1,3],I[2,2],I[2,3],I[3,3])
end

