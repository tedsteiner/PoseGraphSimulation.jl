### Functions to Visualize Pose Graphs and Errors ###
### Ted Steiner ###
### September 2014 ###


###################
### Error Plots ###
###################

function plotCovError( cov_trace, y_max=1000 )
    x = [1:length(cov_trace)]
    
    fignum = Winston.figure()
    p = Winston.plot(x,cov_trace)
    Winston.ylim(0,y_max)
    display(p)
    return fignum
end


function plotCovError( full_cov_trace, filter_cov_trace, y_max=1000 )
    x1 = [1:length(full_cov_trace)]
    x2 = [1:length(filter_cov_trace)]
    
    fignum = Winston.figure()
    p = Winston.plot(x1,full_cov_trace,"b-",x2,filter_cov_trace,"r-")
    Winston.ylim(0,y_max)
    display(p)
    return fignum
end


function plotEpsilons( epsilons::Array{Array{Real,1},1}, show_points::Bool=false )
    c = ["-b","-g","-r","-c","-m","-y"]

    for k = 1:ceil(length(epsilons)/length(c))
        push!(c,c...)
    end

    fignum = Winston.figure()
    p = Winston.plot()

    for k = 1:length(epsilons)
        if k == length(epsilons)
            col = "-k"
        else
            col = c[k]
        end

        if show_points
            col *= "o"
        end

        Winston.oplot([1:length(epsilons[k])],epsilons[k],col)
    end

    display(p)
    return fignum
end


##########################
### Pose-Graph Display ###
##########################


function plotPoseGraph( poses::Array{Pose2D,1}; 
                        poses_truth::Array{Pose2D,1}=Pose2D[], 
                        vertices=false, 
                        legend=false,
                        width=500, 
                        height=0,
                        km=false,
                        nodes::Dict=Dict(),
                        anchors::Vector{Int}=Int[] ) 
    # Parameters
    g_lw = 2
    v_size = 0.25
    fig_name = "Pose Graph Display"
    g_color = "blue"
    v_color = "black"
    x_lab = "x (m)"
    y_lab = "y (m)"
    fontsize = 4

    if length(poses_truth) > 6
        print("Aligning poses... ")
        alignPoses!(poses_truth,poses)
        println("done.")
    end
    coords = poses2cart(poses)

    if km
        coords /= 1000.0
        x_lab = "x (km)"
        y_lab = "y (km)"
    end
    
    if height <= 0
        height = width
    end
    fignum = Winston.figure(name=fig_name,height=height,width=width)
    p = Winston.FramedPlot("xlabel",x_lab,"ylabel",y_lab)

    g = Winston.Curve(coords[:,1],coords[:,2],color=g_color,"linewidth",g_lw)
    Winston.setattr(g,"label","Posegraph")
    Winston.add(p,g)
    
    if vertices
        v = Winston.Points(coords[:,1],coords[:,2],color=v_color,kind="circle",size=v_size)
        Winston.setattr(v,"label","Poses")
        Winston.add(p,v)
    end

    if length(anchors) > 0
        coords_a = zeros(length(anchors),2)
        for k = 1:length(anchors)
            coords_a[k,1] = nodes[anchors[k]].east
            coords_a[k,2] = nodes[anchors[k]].north
        end
        if km
            coords_a /= 1000.0
        end
        a = 
        Winston.Points(coords_a[:,1],coords_a[:,2],color="red",kind="circle",size=.75)
        Winston.setattr(a,"label","Database Locations")
        Winston.add(p,a)
    end

    Winston.setattr(p.x1,"label_style",[:fontsize=>fontsize])
    Winston.setattr(p.y1,"label_style",[:fontsize=>fontsize])
    Winston.setattr(p.x1,"ticklabels_style",[:fontsize=>fontsize])
    Winston.setattr(p.y1,"ticklabels_style",[:fontsize=>fontsize])

    if legend
        if length(anchors) > 0
            if vertices
                lgnd = Winston.Legend(0.65,0.9,{g,v,a})
            else
                lgnd = Winston.Legend(0.65,0.9,{g,a})
            end
        elseif vertices
            lgnd = Winston.Legend(0.65,0.9,{g,v})
        else
            lgnd = Winston.Legend(0.65,0.9,{g})
        end
        Winston.add(p,lgnd)
    end
    
    Winston.display(p)
    return p
end

function poses2cart( poses::Array{Pose2D,1} )
    coords = zeros(length(poses),2)
    
    for k = 1:length(poses)
        coords[k,1] = poses[k].x
        coords[k,2] = poses[k].y
    end
    
    return coords
end


function plotPoseGraph( results::String; truth::String="", width=500, height=0, legend=false, vertices=false )
    poses_results = readiSAMOutput(results)
    
    if truth != ""
        isam_truth = readiSAMTruth(truth)
        poses_truth = isam_truth["poses"]
    else
        poses_truth = nothing
    end
    
    return plotPoseGraph(poses_results,poses_truth=poses_truth,width=width,height=height,vertices=vertices,legend=legend)
end


function plotPoseGraphTruth( truth::String; width=500, height=0 )
    isam_truth = readiSAMTruth(truth)
    poses_truth = isam_truth["poses"]
    return plotPoseGraph(poses_truth,width=width,height=height)
end


