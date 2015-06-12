### Functions to Compute Pose Graph Error ###
### Ted Steiner ###
### September 2014 ###


# Compute pose position RMS error
function posegraphError( posesA::Array{Pose2D,1}, posesB::Array{Pose2D,1} )
    alignPoses!(posesA,posesB)
    
    err = poseError(posesA,posesB)
    
    rmse = sqrt( sum(err.*err) / length(posesA) )
    return rmse
end


function poseError( posesA::Array{Pose2D,1}, posesB::Array{Pose2D,1} )
    err = zeros(length(posesA),3)
    
    for k = 1:length(posesA)
        err[k,:] = poseError(posesA[k],posesB[k])
    end

    return err
end


function poseError( poseA::Pose2D, poseB::Pose2D )
    err = zeros(3)
    err[1] = poseA.x - poseB.x
    err[2] = poseA.y - poseB.y
    err[3] = 0.0 #poseA.theta - poseB.theta
    return err
end


function posegraphError( truth_filename::String, result_filename::String )
    posesTruth = readiSAMOutput(truth_filename)
    posesResult = readiSAMOutput(result_filename)
    return posegraphError(posesTruth,posesResult)
end


function poseErrorCovTrace( cov::Array{Float64,2} )
    err = zeros(size(cov,1))
    
    for k = 1:length(err)
        err[k] = sqrt(cov[k,1]) + sqrt(cov[k,2]) + sqrt(cov[k,3])
    end
    
    return err
end


# Pose Sigma (Pose position error standard deviation)
function poseSigma( cov::Array{Float64,2} )
    return sqrt( cov[1,1] + cov[1,2] )
end

function poseSigmas( covs::Array{Float64,2} )
    err = zeros(size(covs,1))
    
    for k = 1:length(err)
        err[k] = poseSigma( covs[k,:] )
    end
    
    return err
end

function epsilonTrajectory( covs::Array{Float64,2} )
    return mean(poseSigmas(covs))
end


# Trajectory Error Epsilon
function errorRatio( mean_sigmas::Array, rounds::Int=Inf )
    if rounds > size(mean_sigmas,1)
        rounds = size(mean_sigmas,1)
    end
    
    return mean(errorRatios(mean_sigmas,rounds),1)
end


function errorRatios( mean_sigmas::Array, rounds::Int=Inf )
    if rounds > size(mean_sigmas,1)
        rounds = size(mean_sigmas,1)
    end
    
    anchors = size(mean_sigmas,2)-1
    e_ratios = zeros(rounds,anchors)

    for r = 1:rounds
        e_full = mean_sigmas[r,end]

        for a = 1:anchors
            e_ratios[r,a] = (mean_sigmas[r,a] - e_full) / e_full
        end
    end

    return e_ratios
end


######################
### Pose Alignment ###
######################

function alignPoses!(posesA::Array{Pose2D,1}, posesB::Array{Pose2D,1})
    R, t = kabsch(posesA,posesB)
    transformPoses!(posesB,R',-t)
    return nothing
end


# Kabsch Algorithm: Computes rotation and translation between two sets of points
function kabsch( posesA::Array{Pose2D,1}, posesB::Array{Pose2D,1} )
    A = poses2matrix(posesA)
    B = poses2matrix(posesB)
    
    return kabsch(A,B)
end


function kabsch( A::Array{Float64,2}, B::Array{Float64,2} )
    assert(size(A,1) == size(B,1))
    
    centroid_A = mean(A,1)
    centroid_B = mean(B,1)

    H = (A - repmat(centroid_A, size(A,1), 1))' * (B - repmat(centroid_B, size(B,1), 1))
    U,S,V = svd(H)
    R = V*U';

    if det(R) < 0
        println("Reflection detected.")
        V[:,3] *= -1
        R = V*U'
    end

    t = -R*centroid_A' + centroid_B'
    
    return R, t
end


function transformPoses!( poses::Array{Pose2D,1}, R, t )
    R_theta = atan2(R[2,1],R[1,1])
    
    for pose in poses
        pose.x += t[1]
        pose.y += t[2]
        pose.theta += R_theta
    end
    
    return nothing
end


function poses2matrix( poses::Array{Pose2D,1} )
    A = zeros(length(poses),3)
    
    for k = 1:length(poses)
        A[k,1] = poses[k].x
        A[k,2] = poses[k].y
    end
    
    return A
end 


