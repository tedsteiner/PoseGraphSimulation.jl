### Extra Functions for Route Planning ###
### Ted Steiner, MIT AeroAstro ###
### August 2014 - February 2015  ###


# Convenience function to get a list of routable waypoints in an environment.
function getWaypoints( network::OpenStreetMap.Network,  # Routing network
                       candidates::Vector{Int},         # List of candidate waypoints (vertices in network)
                       min_dist::Real,                  # Minimum distance between waypoints 
                       num_waypoints::Int,              # Desired waypoint count
                       avoid::Vector{Int}=Int[],        # Optional list of candidates to avoid selecting
                       min_distance::Real=0 )         # Optional minimum route distance

    waypoints = Int[]
    valid, waypoints, route_dist = randomRouteRecursive(network,candidates,waypoints,min_distance=min_distance,min_waypoints=num_waypoints,min_waypoint_dist=min_dist,avoid=avoid)
    return waypoints
end


### Generate a list of routable waypoints
# NOTE: If network.w is times (for fastest routing), distance below refers 
# instead to time.
function randomRouteRecursive( network::OpenStreetMap.Network,  # Routing network
                               candidates::Vector{Int},         # List of candidate waypoints
                               route::Vector{Int},              # List of waypoints
                               route_distance::Float64=0.0;     # Accumulated route distance
                               avoid::Array{Int,1}=Int[],       # Optional list of candidates to skip
                               min_waypoints::Int=2,            # Minimum number of waypoints in route
                               min_waypoint_dist::Real=0,       # Minimum distance between waypoints
                               min_distance::Real=0 )           # Minimum route distance

    if length(route) >= min_waypoints && route_distance >= min_distance
        return true, route, route_distance
    end
    
    order = randperm(length(candidates))
    
    for index in order
        if !(candidates[index] in avoid)
            if length(route) < 1
                push!(route,candidates[index])
                valid_future, route_new, route_distance = randomRouteRecursive(network,candidates,route,route_distance,avoid=avoid,min_waypoints=min_waypoints,min_waypoint_dist=min_waypoint_dist,min_distance=min_distance)
                if !valid_future
                    pop!(route)
                end
            else
                valid_this, next_dist = existsValidRoute(network,route[end],candidates[index],min_waypoint_dist)
                if valid_this
                    if route[end] != candidates[index]
                        # To prevent getting stuck or cycling, make sure next point is not one of the previous 5 points
                        if length(route) < 3 || (route[end-1] != candidates[index])
                            if length(route) < 4 || (route[end-2] != candidates[index])
                                if length(route) < 5 || (route[end-3] != candidates[index])
                                    if length(route) < 6 || (route[end-4] != candidates[index])
                                        push!(route,candidates[index])
                                        valid_future, route_new, route_distance_new = randomRouteRecursive(network,candidates,route,route_distance+next_dist,avoid=avoid,min_waypoints=min_waypoints,min_waypoint_dist=min_waypoint_dist,min_distance=min_distance)
                                        if valid_future
                                            return true, route_new, route_distance_new
                                        else
                                            pop!(route)
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
    end
    
    return false, route, route_distance
end


# Compute a route along a list of waypoints
function computeRoute( network::OpenStreetMap.Network,  # Routing network
                       waypoints::Vector{Int},          # List of waypoint node IDs
                       fastest::Bool )                  # True: fastest time, False: shortest distance
                       
    route = Int[]
    metric = 0.0 # Time if fastest=true, else distance
    
    for k = 2:length(waypoints)
        s = waypoints[k-1]
        t = waypoints[k]
        if fastest
            route_nodes, route_metric = OpenStreetMap.fastestRoute(network,s,t)
        else
            route_nodes, route_metric = OpenStreetMap.shortestRoute(network,s,t)
        end
        
        if k == 2
            push!(route, route_nodes...)
        else
            push!(route, route_nodes[2:end]...)
        end
        
        metric += route_metric
    end
    
    return route, metric
end


# Get distances from route start to all subsequent route distances
function distanceFromStart( nodes::Dict{Int,Geodesy.ENU},
                            route::Vector{Int} )
    dist = 0.0
    dists = zeros(length(route))
    prev_point = nodes[route[1]]
    for i = 2:length(route)
        point = nodes[route[i]]
        dist += OpenStreetMap.distance(prev_point,point)
        dists[i] = deepcopy(dist)
        prev_point = point
    end
    return dists
end


# Add nodes along a route according to some fixed-size spacing
function interpolateRoute!( nodes::Dict{Int,Geodesy.ENU},   # Node locations
                            route::Vector{Int},             # List of node IDs
                            d_spacing::Real )               # Spacing between interpolated nodes (meters)
    interp_route = Int[]
    push!(interp_route,route[1])
    for k = 2:length(route)
        push!(interp_route,interpolateNodes!(nodes,route[k-1],route[k],d_spacing)...)
        push!(interp_route,route[k])
    end

    return interp_route
end


# Helper function for interpolateRoute! to place the nodes
function interpolateNodes!( nodes, s, t, d_spacing )
    if s == t
        return Int[]
    end
    
    x1 = nodes[s].east
    x2 = nodes[t].east
    y1 = nodes[s].north
    y2 = nodes[t].north
    d_tot = sqrt((x2-x1)^2 + (y2-y1)^2)
    d_frac = d_spacing / d_tot

    num_interp = int(floor(d_tot/d_spacing))-1
    if num_interp < 1
        return Int[]
    end
    
    interp = zeros(Int,num_interp)
    for k = 1:length(interp)
        x = x1 + k*d_frac*(x2-x1)
        y = y1 + k*d_frac*(y2-y1)
        loc = OpenStreetMap.ENU(x,y)
        interp[k] = OpenStreetMap.addNewNode!(nodes,loc)
    end

    return interp
end


# Helper function for randomRouteRecursive()
function existsValidRoute(network, node0, node1, min_dist=0.0)
    start_vertex = network.v[node0]
    finish_vertex = network.v[node1]
    finish_index = network.v[node1].index

    dijkstra_result = Graphs.dijkstra_shortest_paths(network.g,network.w,start_vertex,visitor=EndWhenAnyNode([finish_vertex]))
    
    dist = dijkstra_result.dists[finish_index]
    if dist < Inf && dist > min_dist
        return true, dist
    end
    
    return false, -1
end


