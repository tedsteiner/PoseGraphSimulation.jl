### Functions for Pose Graph Input/Output ###
### Ted Steiner ###
### September 2014 ###

# Note: See file_format.txt for text file information.

##############
### Output ###
##############

# Write BetweenFactors to text file as EDGE2's
function writeMeasurements( filename::String, meas::Union(Vector{BetweenFactor},Array{Factor,1}) )
    f = open(filename,"w")
    writeMeasurements(f,meas)
    close(f)
    nothing
end

# Write out truth data and trajectory information
function writeTruthFile( filename::String,
                         poses::Array{Pose2D,1};
                         waypoints::Array{Int,1}=Int[],
                         anchors::Array{Array{Int,1},1}=Array{Int,1}[], 
                         flags::Dict{String,Any}=Dict{String,Any}() )
    f = open(filename,"w")

    writeFlags(f,flags)
    write(f,@sprintf("\n"))

    writeWaypoints(f,waypoints)
    write(f,@sprintf("\n"))

    if length(anchors) > 0
        writeAnchors(f,anchors)
        write(f,@sprintf("\n"))
    end
    write(f,@sprintf("\n"))
    write(f,@sprintf("\n"))

    writePoses(f,poses)
    write(f,@sprintf("\n"))

    close(f)
    return nothing
end

    
function writeMeasurements( f::IOStream, meas::Union(Vector{BetweenFactor},Vector{Factor}) )
    for m in meas
        write(f,factor2string(m))
    end
    
    return nothing
end

function factor2string( m::BetweenFactor )
    @sprintf("EDGE2 %d %d %f %f %f %f 0.0 0.0 %f 0.0 %f\n",m.s,m.t,m.meas.dx,m.meas.dy,m.meas.dT,m.info.ff,m.info.ss,m.info.rr)
end

function factor2string( m::PriorFactor )
    @sprintf("PRIOR2 %d %f %f %f %f 0.0 0.0 %f 0.0 %f\n",m.pose,m.meas.dx,m.meas.dy,m.meas.dT,m.info.ff,m.info.ss,m.info.rr)
end


# Write poses to text file as Pose2d_Node's
function writePoses( f::IOStream, poses::Array{Pose2D,1} )
    count = 0
    for p in poses
        str = @sprintf("Pose2d_Node %d (%f, %f, %f)\n",count,p.x,p.y,p.theta)
        write(f,str)
        count += 1
    end
    
    return nothing
end

function writeAnchors( f::IOStream, anchors::Array{Int,1} )
    str = @sprintf("ANCHOR %d [%s]\n",length(anchors),vectorToString(anchors))
    write(f,str)
    return nothing
end

function writeAnchors( f::IOStream, anchors::Array{Array{Int,1},1} )
    for a in anchors
        str = @sprintf("ANCHOR %d [%s]\n",length(a),vectorToString(a))
        write(f,str)
    end
    return nothing
end


function writeWaypoints( f::IOStream, waypoints::Array{Int,1} )
    str = @sprintf("WAYPOINTS %d [%s]\n",length(waypoints),vectorToString(waypoints))
    write(f,str)
    return nothing
end


function writeFlags( f::IOStream, flags::Dict{String,Any} )
    for (name,value) in flags
        write(f,"FLAG $(uppercase(name)) $value\n")
    end
    return nothing
end


function vectorToString( v::Array{Int,1} )
    v_str = String[]
    for k = 1:length(v)
        push!(v_str,string(v[k]))
    end
    
    return join(v_str," ")
end

#############
### Input ###
#############

# Read pose locations from iSAM output text file
function readiSAMOutput( filename::String )
    return open(parseISAMOutput, filename)
end

function readiSAMTruth( filename::String )
    return open(parseISAMTruth, filename)
end

function readiSAMOutputCov( filename::String )
    return open(parseISAMOutputCov, filename)
end


function parseISAMOutput( f::IOStream )
    poses = Pose2D[]
    delim = Char[' ';',';'(';')']
    
    for line in eachline(f)
        if beginswith(line,"Pose2d_Node")
            s = split(line,delim)
            x = float(s[4])
            y = float(s[6])
            theta = float(s[8])
            push!(poses,Pose2D(x,y,theta))
        end
    end
    
    return poses
end


function parseISAMOutputCov( f::IOStream )
    cov_all = readdlm(f,' ',String)
    cov_s = cov_all[:,[3,6,8]]
    cov = float(cov_s)
    
    return cov
end


function parseISAMTruth( f::IOStream )
    # Helper function
    checkint(x) = 
        try
            int(string(x))
            return true
        catch
            return false
        end
        
    truth = Dict{String,Any}()
    anchor_sets = Array{Int,1}[]
    delim = Char[' ';',';'(';')';'[';']';'\n']
    
    for line in eachline(f)
        s = split(line,delim)
        if beginswith(line,"ANCHOR")
            anchors = Int[]
            for k = 3:length(s)
                if checkint(s[k])
                    push!(anchors,int(s[k]))
                end
            end
            push!(anchor_sets,anchors)
        elseif beginswith(line,"WAYPOINTS")
            waypoints = Int[]
            for k = 3:length(s)
                if checkint(s[k])
                    push!(waypoints,int(s[k]))
                end
            end
            truth[s[1]] = waypoints
        elseif beginswith(line,"FLAG")
            s = split(line,delim)
            if checkint(s[3])
                truth[s[2]] = int(s[3])
            else
                truth[s[2]] = s[3]
            end
        end
    end
    
    truth["poses"] = parseISAMOutput(f)

    if length(anchor_sets) > 0
        truth["anchor_sets"] = anchor_sets
    end
    
    return truth
end


