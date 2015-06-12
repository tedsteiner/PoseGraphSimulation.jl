### Interface to iSAM ###
### Ted Steiner ###
### January 2014   ###


# Note: To use covariances, the isam executable must be modified to 
# compute these and output them to the file. One of the isam examples provides 
# and example of how this can be done. 
# The iSAM library is available at people.csail.mit.edu/kaess/isam/
function isam( isam_path::String,
               fname_graph::String;
               fname_result::String="NA",
               fname_cov::String="NA",
               fname_cov_rt::String="NA",
               batch::Bool=false,
               quiet::Bool=false,
               powell::Bool=true )

    args = String[]
    
    if batch
        push!(args,"-B")
    end
    if quiet
        push!(args,"-q")
    end
    if powell
        push!(args,"-P")
    end
    if fname_cov != "NA"
        push!(args,"-C -U $fname_cov")
    end
    if fname_cov_rt != "NA"
        push!(args,"-K $fname_cov_rt")
    end
    if fname_result != "NA"
        push!(args,"-W $fname_result")
    end

    c = `$isam_path $args $fname_graph`
    run(`$isam_path $args $fname_graph`)
    return c
end
               
