#!/usr/bin/env julia
# Change to 'julia' instead of 'julia167' if you don't have a symlink setup

using RobotOS
using iLQGameSolver
using SparseArrays
using LinearAlgebra


@rosimport geometry_msgs.msg: Twist, Pose
rostypegen()
using .geometry_msgs.msg: Twist, Pose

function setupgame()
    # Setup the problem

    dt = 0.1                    # Step size [s]
    tf = 10.0                    # Horizon [s]
    N = Int(tf/dt)         # Number of steps (knot points)
    println(N)

    # Define cost matrices 
    nx = 4 
    nu = 2
    Nplayer = 3

    Nu = nu * Nplayer
    Nx = nx * Nplayer

    Q1 = sparse(zeros(Nx,Nx))     # State cost for agent 1
    Q1[1:nx,1:nx] .= 3.0*I(nx)
    Qn1 = Q1                    # Terminal cost for agent 1

    Q2 = sparse(zeros(Nx,Nx))     # State cost for agent 2
    Q2[nx+1:2*nx,nx+1:2*nx] .= 3.0*I(nx)
    Qn2 = Q2                    # Terminal cost for agent 2

    Q3 = sparse(zeros(Nx,Nx))     # State cost for agent 2
    Q3[2*nx+1:3*nx,2*nx+1:3*nx] .= 3.0*I(nx)
    Qn3 = Q3                    # Terminal cost for agent 2

    R11 = sparse(1.0*I(2))              # Control cost for player 1
    R12 = sparse(0.0*I(2))     # Control cost for player 1 associated with player 2's controls
    R13 = sparse(0.0*I(2))     # Control cost for player 2 associated with player 1's controls
    R21 = sparse(0.0*I(2))     # Control cost for player 2 associated with player 1's controls
    R22 = sparse(1.0*I(2))              # Contorl cost for player 2
    R23 = sparse(0.0*I(2))     # Control cost for player 2 associated with player 1's controls
    R31 = sparse(0.0*I(2))     # Control cost for player 2 associated with player 1's controls
    R32 = sparse(0.0*I(2))     # Control cost for player 2 associated with player 1's controls
    R33 = sparse(1.0*I(2))     # Control cost for player 2 associated with player 1's controls

    dmax = 2.0                  # Distance that both agents should keep between each other [m]
    ρ = 500.0                   # Penalty factor for violating the distance constraint

    # Q's are stacked vertically
    Q = sparse(zeros(Float32, Nx*Nplayer, Nx))
    # @show size([Q1; Q2; Q3]), size(Q)
    #Q .= [Q1; Q2]
    Q .= [Q1; Q2; Q3]

    # Qn's are stacked vertically
    Qn = sparse(zeros(Float32, Nx*Nplayer, Nx))
    #Qn .= [Qn1; Qn2]
    Qn .= [Qn1; Qn2; Qn3]

    # R's are stacked as a matrix
    R = sparse(zeros(Float32, Nu, Nu))
    #R .= [R11 R12; R21 R22]
    R .= [R11 R12 R13; R21 R22 R23; R31 R32 R33]

    NHor = 20
    tol = 1e-1

    game = iLQGameSolver.GameSetup(nx, nu, Nplayer, Q, R, Qn, dt, tf, NHor, dmax, ρ, tol)

    solver = iLQGameSolver.iLQSetup(Nx, Nu, Nplayer, NHor);

    x₀= [   5.0; 0.0; 0.0; 0.0; 
        0.0; 5.0; 0.0; 0.0; 
        0.0; 0.0; 0.0; 0.0] 
        # Initial state

    xgoal = [   5.0; 10.0; 0.0; 0.0; 
                10.0; 5.0; 0.0; 0.0; 
                10.0; 10.0; 0.0; 0.0]   
            # Final state

    # Input constraints
    umax = [2.0, 2.0, 
            2.0, 2.0, 
            2.0, 2.0]   

    umin = [-2.0, -2.0, 
            -2.0, -2.0, 
            -2.0, -2.0]

    ugoal = [   0.0, 0.0, 
                0.0, 0.0,  
                0.0, 0.0]     

    game.x0 .= x₀
    game.xf .= xgoal
    game.umin .= umin
    game.umax .= umax
    game.uf .= ugoal;

    #xₜ, uₜ = iLQGameSolver.recedingHorizon(game, solver, iLQGameSolver.pointMass, iLQGameSolver.costPointMass);
    
    return  solver,game #xₜ, uₜ
end

function commands(pub0, pub1)
    agent1 = Twist()
    agent2 = Twist()

    solver, game = setupgame()
    Nplayer = game.Nplayer
    Nx = game.nx * Nplayer
    Nu = game.nu * Nplayer
    dt = game.dt
    tf = game.tf
    NHor = game.NHor
    # solver.P = rand(NHor, Nu, Nx)*0.01
    # solver.α = rand(NHor, Nu)*0.01

    N = trunc(Int, tf/dt)

    X = zeros(N, Nx) 
    U = zeros(N-1, Nu)
    X[1,:] = game.x0

    #rate = Rate(100) # 10 Hz
    tstart = time_ns()
    for k = 1:N-1-NHor
        xₜ, uₜ = iLQGameSolver.solveILQGame(game, solver, iLQGameSolver.pointMass, iLQGameSolver.costPointMass, X[k,:], false)
        X[k+1,:], U[k,:] = xₜ[2,:,:], uₜ[1,:,:]
        agent1.linear.x = U[k,1]#*cos(rad2deg(xₜ[i,3]))
        agent1.linear.y = U[k,2]#uₜ[i,1]*sin(rad2deg(xₜ[i,3]))
        agent1.angular.z = 0.0#uₜ[i,2]


        agent2.linear.x = U[k,3]#*cos(rad2deg(xₜ[i,6]))
        agent2.linear.y = U[k,4]#*sin(rad2deg(xₜ[i,6]))
        agent2.angular.z = 0.0#uₜ[i,4]

        publish(pub1, agent1)
        publish(pub0, agent2)
        #rossleep(rate)
    end

    for k = N-NHor:N-1
        xₜ, uₜ = iLQGameSolver.solveILQGame(game, solver, iLQGameSolver.pointMass, iLQGameSolver.costPointMass, X[k,:], true)
        X[k+1,:], U[k,:] = xₜ[2,:,:], uₜ[1,:,:]
        #println(k)
    end
    
    tend = time_ns()

    rate = N / (tend - tstart) * 1e9
    println("Controller ran at $rate Hz")

    # for i = 1:size(uₜ)[1]
    #     agent1.linear.x = uₜ[i,1]#*cos(rad2deg(xₜ[i,3]))
    #     #agent1.linear.y = uₜ[i,1]*sin(rad2deg(xₜ[i,3]))
    #     agent1.angular.z = uₜ[i,2]


    #     agent2.linear.x = uₜ[i,3]#*cos(rad2deg(xₜ[i,6]))
    #     #agent2.linear.y = uₜ[i,3]*sin(rad2deg(xₜ[i,6]))
    #     agent2.angular.z = uₜ[i,4]

    #     # loginfo(hello_str)
    #     publish(pub1, agent1)
    #     publish(pub0, agent2)
    #     rossleep(rate)
    # end
end

function main()
    init_node("turtlebot3_cmd") # node name
    pub0 = Publisher{Twist}("/tb3_0/cmd_vel", queue_size=10) # topic name
    pub1 = Publisher{Twist}("/tb3_1/cmd_vel", queue_size=10) # topic name

    commands(pub0, pub1)
end

if !isinteractive()
    main()
end
