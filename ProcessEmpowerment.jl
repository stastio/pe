using Plots; pyplot()
using BSON: @save, @load
using StatsBase
using Convex
using LinearAlgebra
using SCS, GLPK

### choose/uncomment a world. default: RLMaze ###
worldType = "RLMaze"
# worldType = "BridgeWorld"
# worldType = "RaceWorld"
# worldType = "Rectangle"

### choose dimensions  ###
if worldType == "BridgeWorld"
	### bridge ###
	dS₁, dS₂        = 5, 8
elseif worldType == "RaceWorld" 
	### race ###
	dS₁, dS₂        = 8, 8
elseif worldType == "RLMaze" 
	### maze ###
	dS₁, dS₂        = 8, 8
elseif worldType == "Rectangle" 
	### maze ###
	dS₁, dS₂        = 3, 6
end

dS		= dS₁*dS₂
T		= 2
TYPE 		= Float64
PROBTENSOR 	= Array{TYPE, 2*T+1} 
nIters          = 5

Actions = Dict( 
1=>(-1, 0), 
2=>(+1, 0), 
3=>(0, -1), 
4=>(0, 1), 
5=>(0,0),
)
ActionNames = Dict( 
1=>"\U2191", 
2=>"\U2193", 
3=>"\U2190", 
4=>"\U2192",
5=>"\U21BB",
)

BridgeActionNames = Dict( 	1=>"\U2192", 
2=>"\U2198", 
3=>"\U2193", 
4=>"\U2199", 
5=>"\U2190", 
6=>"\U2196", 
7=>"\U2191", 
8=>"\U2196")

BridgeActions = Dict( 	1=>( 0, +1), 
2=>(+1, +1), 
3=>(+1,  0), 
4=>(+1, -1), 
5=>( 0, -1), 
6=>(-1, -1), 
7=>(-1, 0), 
8=>(-1, +1)
)

RaceActionNames = Dict(
1=>"R", 
2=>"D", 
3=>"L", 
4=>"U", 
)
ā = 1
RaceActions = Dict(
1=>(0, +1), 
2=>(+1, 0), 
3=>(0, -1), 
4=>(-1, 0), 
)

# UpdateMazeStates_display(x) = display(x)
UpdateMazeStates_display(x) = nothing 
function UpdateRaceState(s, a, test)
	p′ = s[1:2] .+ s[3:4] # position 
	q′ = clamp.(s[3:4] .+ a.second, -v̄, +v̄) # velocity
	s′ = (p′..., q′...)
	s′ = s′ ∈ land ? s′ : death
	"UpdateRaceState($(@__LINE__)): $(s), $(a), $(s′)" |> UpdateMazeStates_display
	s′
end

function UpdateMazeState(s, a, test)
	p′ = s[1:2] .+ s[3:4] # position 
	q′ = clamp.(s[3:4] .+ a.second, -v̄, +v̄) # velocity
	s′ = (p′..., q′...)
	s′ = validMove(s[1:2], s′[1:2], test) ? s′ : death
end

if worldType == "BridgeWorld"
	dA = length(BridgeActions)
elseif worldType == "RaceWorld"

	plotsPath = "you path to plots for RaceWorld "

	Actions = RaceActions
	ActionNames = RaceActionNames

	ActionsTest = RaceActions
	ActionTestNames = RaceActionNames

	UpdateState = UpdateRaceState#(s, a)

	dA = length(Actions)
elseif worldType == "RLMaze"

	plotsPath = "you path to plots for RLMaze "

	Actions = Actions
	ActionNames = ActionNames

	ActionsTest = Actions
	ActionTestNames = ActionNames

	UpdateState = UpdateMazeState#(s, a)

	dA = length(Actions)
elseif worldType == "Rectangle"

	plotsPath = "you path to plots for Rectangle "


	Actions = RaceActions
	ActionNames = RaceActionNames

	ActionsTest = RaceActions
	ActionTestNames = RaceActionNames

	dA = length(Actions)
	UpdateState = UpdateMazeState#(s, a)
end


### design a desired obstacle pattern  ###
Obstacle = []
ObstacleSetTrain = []
if worldType == "RLMaze"
	ObsX1 = [ (x, y) for x ∈ (4.0,), y ∈ range(1, dS₂/2+1) ]
	ObsX2 = [ (x, y) for x ∈ (6.0,), y ∈ range(4, dS₂/1) ]
	ObsX1Plot = [ (x, y) for x ∈ (4.0,), y ∈ range(1, dS₂/2+1) ]
	ObsX2Plot = [ (x, y) for x ∈ (6.0,), y ∈ range(4, dS₂/1 +0) ]
	Obs = [ObsX1..., ObsX2...]
	for xy ∈ Obs
		push!(ObstacleSetTrain, Set( (xy, ((xy[1]-1), xy[2]  ))))
		push!(ObstacleSetTrain, Set( (xy, ((xy[1]-1), xy[2]-1))))
		push!(ObstacleSetTrain, Set( (xy, ((xy[1]-1), xy[2]+1))))
	end
elseif worldType ∈ ("Rectangle", "RaceWorld")
	ObstacleSetTrain = []
end

ObstacleSetTest = [ObstacleSetTrain...]	
if worldType == "Rectangle"
	bxL = 2.0
	bxH = 3.0
	ObsX1 = [ (x, y) for x ∈ (bxL,), y ∈ range(3, 4) ]
	ObsX2 = [ (x, y) for x ∈ (bxH,), y ∈ range(3, 4) ]
	ObsX3 = [ (bxL, 4), (bxH, 4) ]
	ObsX1Plot = [ (x, y) for x ∈ (bxL,), y ∈ range(3, 4) ]
	ObsX2Plot = [ (x, y) for x ∈ (bxH,), y ∈ range(3, 4) ]
	ObsX3Plot = [ (bxL, 4), (bxH, 4) ]
	Obs = [ObsX1..., ObsX2...]
	for xy ∈ Obs
		push!(ObstacleSetTest, Set( (xy, ((xy[1]-1), xy[2]  ))))
		push!(ObstacleSetTest, Set( (xy, ((xy[1]-1), xy[2]-1))))
		push!(ObstacleSetTest, Set( (xy, ((xy[1]-1), xy[2]+1))))
	end
	for xy ∈ ObsX3
		push!(ObstacleSetTest, Set( (xy, ((xy[1]), xy[2]+1  ))))
		push!(ObstacleSetTest, Set( (xy, ((xy[1]-1), xy[2]+1))))
		push!(ObstacleSetTest, Set( (xy, ((xy[1]+1), xy[2]+1))))
	end
end

if worldType == "RLMaze"
	ObstacleSetForPlot = ObstacleSetTrain
elseif worldType ∈ (["RaceWorld"])
	ObstacleSetForPlot = ObstacleSetTest
elseif worldType ∈ (["Rectangle"])
	ObstacleSetForPlot = ObstacleSetTest
end
Trap = [(x, y) for x ∈ (1, 5), y ∈ range(1, dS₂-1)]
Push = [(x, y) for x ∈ (2, 4), y ∈ range(1, dS₂-1)]

validState(s) = all(s .> 0) && all(s[1] <= dS₁) && all(s[2]  <= dS₂) && !(s ∈ Obstacle)
inWorld(s) = all(s .> 0) && all(s[1] <= dS₁) && all(s[2]  <= dS₂)# && !(s ∈ Obstacle)

if worldType == "Rectangle"
	ObstacleSet = ()
end

validMove(s, s′, test) = all(s′ .> 0) && all(s′[1] <= dS₁) && all(s′[2]  <= dS₂) && (Set((s, s′)) ∉ (test=="TEST" ? ObstacleSetTest : ObstacleSetTrain) )


water  = [(x, y) for x ∈ (1, dS₁), y ∈ (2, 3, 4, )]
bridge = [(x, y) for x ∈ (2:dS₁-1), y ∈ (2, 3, 4, )]
Land( dims, water ) = setdiff((x->x.I).( CartesianIndices( dims ))[:], water)
Room( land, bridge ) = setdiff(land, bridge)
WorldLinearIdx(s, world) = findall(w->w==s, world)[1]
LinearIdx(s, part) = (findall(w->w==s, part)[1])


v̄  = 1
v̄₁ = v̄
v̄₂ = v̄
dv = 1
function LandRaceQuarter()
	land = Tuple{Int64, Int64, Int64, Int64}[]
	for x ∈ 1:dS₁
		if     x == 1
			(yv->push!(land, (x, yv.I...))).(CartesianIndices((3:4, -v̄₁:dv:+v̄₁, -v̄₂:dv:+v̄₂)))
		elseif x == 2
			(yv->push!(land, (x, yv.I...))).(CartesianIndices((2:4, -v̄₁:dv:+v̄₁, -v̄₂:dv:+v̄₂)))
		elseif x == 3
			(yv->push!(land, (x, yv.I...))).(CartesianIndices((1:3, -v̄₁:dv:+v̄₁, -v̄₂:dv:+v̄₂)))
		elseif x == 4
			(yv->push!(land, (x, yv.I...))).(CartesianIndices((1:2, -v̄₁:dv:+v̄₁, -v̄₂:dv:+v̄₂)))
		end
	end
	land
end
function LandRaceHalf()
	land = Tuple{Int64, Int64, Int64, Int64}[]
	for x ∈ 1:dS₁
		if     x == 1
			(yv->push!(land, (x, yv.I...))).(CartesianIndices((3:6, -v̄₁:dv:+v̄₁, -v̄₂:dv:+v̄₂)))
		elseif x == 2
			(yv->push!(land, (x, yv.I...))).(CartesianIndices((2:7, -v̄₁:dv:+v̄₁, -v̄₂:dv:+v̄₂)))
		elseif x == 3
			(yv->push!(land, (x, yv.I...))).(CartesianIndices((1:3, -v̄₁:dv:+v̄₁, -v̄₂:dv:+v̄₂)))
			(yv->push!(land, (x, yv.I...))).(CartesianIndices((6:8, -v̄₁:dv:+v̄₁, -v̄₂:dv:+v̄₂)))
		elseif x == 4
			(yv->push!(land, (x, yv.I...))).(CartesianIndices((1:2, -v̄₁:dv:+v̄₁, -v̄₂:dv:+v̄₂)))
			(yv->push!(land, (x, yv.I...))).(CartesianIndices((7:8, -v̄₁:dv:+v̄₁, -v̄₂:dv:+v̄₂)))
		end
	end
	land
end
function LandRace()
	land = []
	for x ∈ 1:dS₁
		if     x == 1
			(yv->push!(land, (x, yv.I...))).(CartesianIndices((3:6, -v̄₁:dv:+v̄₁, -v̄₂:dv:+v̄₂)))
		elseif x == 2
			(yv->push!(land, (x, yv.I...))).(CartesianIndices((2:7, -v̄₁:dv:+v̄₁, -v̄₂:dv:+v̄₂)))
		elseif x == 3
			(yv->push!(land, (x, yv.I...))).(CartesianIndices((1:3, -v̄₁:dv:+v̄₁, -v̄₂:dv:+v̄₂)))
			(yv->push!(land, (x, yv.I...))).(CartesianIndices((6:8, -v̄₁:dv:+v̄₁, -v̄₂:dv:+v̄₂)))
		elseif x == 4
			(yv->push!(land, (x, yv.I...))).(CartesianIndices((1:2, -v̄₁:dv:+v̄₁, -v̄₂:dv:+v̄₂)))
			(yv->push!(land, (x, yv.I...))).(CartesianIndices((7:8, -v̄₁:dv:+v̄₁, -v̄₂:dv:+v̄₂)))
		elseif x == 5
			(yv->push!(land, (x, yv.I...))).(CartesianIndices((1:2, -v̄₁:dv:+v̄₁, -v̄₂:dv:+v̄₂)))
			(yv->push!(land, (x, yv.I...))).(CartesianIndices((7:8, -v̄₁:dv:+v̄₁, -v̄₂:dv:+v̄₂)))
		elseif x == 6
			(yv->push!(land, (x, yv.I...))).(CartesianIndices((1:3, -v̄₁:dv:+v̄₁, -v̄₂:dv:+v̄₂)))
			(yv->push!(land, (x, yv.I...))).(CartesianIndices((6:8, -v̄₁:dv:+v̄₁, -v̄₂:dv:+v̄₂)))
		elseif x == 7
			(yv->push!(land, (x, yv.I...))).(CartesianIndices((2:7, -v̄₁:dv:+v̄₁, -v̄₂:dv:+v̄₂)))
		elseif x == 8
			(yv->push!(land, (x, yv.I...))).(CartesianIndices((3:6, -v̄₁:dv:+v̄₁, -v̄₂:dv:+v̄₂)))
		end
	end
	land
end


if worldType == "BridgeWorld"
	death  = (-777, -777) # water states are the death state 
	land  = Land((dS₁, dS₂), water[:])
	room  = Room(land, bridge[:])
elseif worldType == "RaceWorld"
	death = (-777, -777, -777, -777)
	land     = LandRace()
elseif worldType  ∈ ("RLMaze", "Rectangle") 
	death = (-777, -777, -777, -777)
	land = (z->z.I).(CartesianIndices((dS₁, dS₂, -v̄₁:dv:+v̄₁, -v̄₂:dv:+v̄₂)))[:]
end
world = [land..., death]
worldLen = length(world)

function VisualizeRL(Ps_a, Vs)
	dS₁, dS₂, dA = size(Ps_a)[1:3]
	reshape(Vs, (dS₁, dS₁)) |> display

	M = Array{String, 2}(undef, dS₁, dS₂)
	F = Array{Tuple,  2}(undef, dS₁, dS₂)
	for i ∈ 1:dS₁, j ∈ 1:dS₂
		M[i, j] = ActionNames[findmax(Ps_a[i, j, :, 1, 1, 1])[2]]
		F[i, j] = Actions[findmax(Ps_a[i, j, :, 1, 1, 1])[2]]
	end
	drift = Dict{Tuple, Tuple}()
	for s ∈ land
		drift[s] = F[(s[1:2])...]
	end
	drift
end

function Visualize(E, land::Vector{Tuple{Int64, Int64}})
	M = zeros(TYPE, dS₁, dS₂)
	for s ∈ land
		i = LinearIdx(s, land)
		M[s[1], s[2]] = E[i]
	end
	M
end
function Visualize(E, land::Vector{Tuple{Int64, Int64, Int64, Int64}})
	M = zeros(TYPE, dS₁, dS₂)
	for s ∈ (filter(z->(z[3]==0 & z[4]==0), land) |> unique)
		s |> display
		s |> display
		i = LinearIdx(s, land)
		M[s[1], s[2]] = E[i]
	end
	M
end

function VisualizeTrajectoryOnLand(land, trajectory, plotsPath, fileName, mytitle, drift)
	landNoVel = []
	for s ∈ land
		push!(landNoVel, s[1:2])
	end
	landNoVel = landNoVel |> unique 

	M = zeros(Int, dS₁, dS₂)
	for i ∈ 1:dS₁
		for j ∈ 1:dS₂
			if (i, j) ∉ landNoVel
				M[i, j] = 1
			end
		end
	end
	heatmap(1:dS₂, 1:dS₁, M, 
	c=cgrad([:white,:black]), size=(1000, 1000),
	title=mytitle, colorbar=:none, showaxis = true, aspect_ratio=:equal, ticks = true, framestyle = :box, widen=false, tickfontsize=15, xticks=1:dS₂, yticks=1:dS₁, thickness_scaling = 3)

	X = []
	Y = []
	V = []
	U = []
	driftNoVel = Dict{Tuple, Tuple}()
	for s ∈ drift
		if s.first == death
			continue
		end
		s_noVel = s.first[1:2]
		driftNoVel[s_noVel] = s.second
	end

	for d in driftNoVel
		push!(X, d.first[2])
		push!(Y, d.first[1])
		push!(U, 0.6*d.second[1])
		push!(V, 0.6*d.second[2])
	end
	quiver!(X, Y, quiver=(V, U); color = :blue, markersize=0.1, lw=1, arrow=Plots.arrow(:open, :head, 1, 1))

	for i in CartesianIndices((-1:1, -1:1))
		V = []
		U = []
		drift_scale = 0.2 
		for d in driftNoVel
			if abs(i.I[1]) == abs(i.I[2])
				push!(U, drift_scale*i.I[1]/1.4)
				push!(V, drift_scale*i.I[2]/1.4)
			else
				push!(U, drift_scale*i.I[1])
				push!(V, drift_scale*i.I[2])
			end
		end
		quiver!(X, Y, quiver=(V, U); color = :black, markersize=0.1, lw=0.7, arrow=Plots.arrow(:closed, :head, 0.01, 0.01))
	end

	Y = repeat(1:dS₁, inner=(1, dS₂))
	X = repeat(reshape(1:dS₂, 1, dS₂), inner=(dS₁, 1))
	V = zeros(Int, dS₁, dS₂)
	U = zeros(Int, dS₁, dS₂)
	for i in 1:(length(trajectory)-1)
		U[trajectory[i][1], trajectory[i][2]] = trajectory[i+1][1]-trajectory[i][1]
		V[trajectory[i][1], trajectory[i][2]] = trajectory[i+1][2]-trajectory[i][2]
	end

	quiver!(X, Y, quiver=(V, U); color = :green, markersize=0.1, lw=3, arrowlength=0, yflip=true, legend=false)

	markStartEnd = false
	if markStartEnd
		scatter!((trajectory[1][2], trajectory[1][1]), color= :red, label=:none, legend=false, ms=8)
		scatter!((trajectory[end][2], trajectory[end][1]), color= :green, label=:none, legend=false, ms=8)
	end

	if !isempty( ObstacleSetForPlot ) 
		reverseAndShiftH(s::Tuple) = (x = (2*s[1]-1)/2; y = s[2]; s = (x, y); s = reverse(s))		
		reverseAndShiftV(s::Tuple) = (reverse(s))		
		plot!(reverseAndShiftH.(ObsX1Plot) |> vec, lc=:black, lw=10)
		plot!(reverseAndShiftH.(ObsX2Plot) |> vec, lc=:black, lw=10)
		if worldType == "Rectangle"
			plot!(reverseAndShiftH.(ObsX3Plot) |> vec, lc=:black, lw=10)
		end

	end

	plot!(; xticks=1:dS₂, yticks=1:dS₁)

	@show savefig(plotsPath * fileName * ".png")
	@save plotsPath * "/BSON/" * fileName * ".bson" land trajectory plotsPath fileName mytitle drift

	nothing
end

function DynamicsInLinearStateIndexes( Psa_s′ )
	dS2 = dS₁*dS₂#size(Psa_s′)[1]^2	
	A = ( A = Array{CartesianIndex, 2}(undef, dS₁, dS₂); (z->(A[z.I[1], z.I[2]]=z)).(CartesianIndices((dS₁, dS₂))) )
	if T ≥ 2
		f = zeros(TYPE, dS2, dS2, ntuple(i->1, T-2)..., dA, ntuple(i->1,T-1)..., 1)
	else
		f = zeros(TYPE, dS2,  dA, dS2)
	end
	for sᵢ in 1:dS2
		cᵢ = A[sᵢ].I
		for aᵢ in 1:dA
			for sⱼ in 1:dS2
				cⱼ = A[sⱼ].I
				if T ≥ 2
					f[sᵢ, sⱼ, ntuple(i->1, T-2)..., aᵢ, ntuple(i->1,T-1)..., 1] = Psa_s′[cᵢ..., aᵢ, cⱼ...]
				else
					f[sᵢ, aᵢ, sⱼ] = Psa_s′[cᵢ..., aᵢ, cⱼ...] 
				end
			end
		end
	end
	f
end


function RaceDynamicsDriftProfile()
	drift = Dict{Tuple, Tuple}()
	drift_pos = false
	s_drift = ()
	for s ∈ world
		if     s[1] ≤ 4 && s[2] ≤ 4
			if drift_pos
				s_drift = ( (s[1:2] .+ (-1, +1))..., 0, 0)
			else
				drift[s] = (-1, 1)
			end
			s_drift = s_drift ∈ land ? s_drift : death
		elseif s[1] ≤ 4 && s[2] > 4
			if drift_pos
				s_drift = ( (s[1:2] .+ (+1, +1))..., 0, 0)
			else
				drift[s] = (+1, +1)
			end
			s_drift = s_drift ∈ land ? s_drift : death
		elseif s[1] > 4 && s[2] > 4
			if drift_pos
				s_drift = ( (s[1:2] .+ (+1, -1))..., 0, 0)
			else
				drift[s] = (+1, -1)
			end
			s_drift = s_drift ∈ land ? s_drift : death
		elseif s[1] > 4 && s[2] ≤ 4
			if drift_pos
				s_drift = ( (s[1:2] .+ (-1, -1))..., 0, 0)
			else
				drift[s] = (-1, -1)
			end
			s_drift = s_drift ∈ land ? s_drift : death
		end
	end
	drift
end


function RectangleDriftProfile()
	drift = Dict{Tuple, Tuple}()
	for s ∈ world
		drift[s] = (0, 1)
	end
	drift
end


MazeDynamicsWithDriftProfile(x) = display(x)
# MazeDynamicsWithDriftProfile(x) = nothing
function MazeDynamicsWithDriftProfile( α, drift::Dict, ζ )

	P = zeros(TYPE, worldLen, dA, worldLen)
	γ = isempty( drift ) ? 1 : ζ
	PP = Dict()

	"MazeDynamicsWithDriftProfile($(@__LINE__)): γ=$(γ), α=$(α)" |> display

	for s ∈ world 
		s_drift = ()
		Idxᵢ = LinearIdx( s, world )

		if s == death
			P[Idxᵢ, :, Idxᵢ] .= 1.0  
			continue 
		end

		S′ = []
		for a ∈ ActionsTest
			s′ = UpdateState(s, a, "TEST")
			push!(S′, s′)
		end
		S′ = S′ |> unique

		if !isempty(drift)
			s_drift = (s[1:2]..., drift[s]...)
		end

		for a ∈ ActionsTest
			s′ = UpdateState(s, a, "TEST")
			if s′ == death
				P[Idxᵢ, a.first, LinearIdx( death, world )] = 1.0
				continue
			end

			Idxⱼ  = LinearIdx( s′, world )

			if s′ == s_drift
				P[Idxᵢ, a.first, Idxⱼ] = 1*(1-α)
			else
				if γ > 0
					P[Idxᵢ, a.first, Idxⱼ] = γ*(1-α)
				end

				if !isempty(drift) && γ < 1.0
					P[Idxᵢ, a.first, LinearIdx(s_drift, world)] = (1-γ)*(1-α)
				end
			end

			if α > 0
				if α < 1
					for s′′ ∈ filter(x->(x ∉ (s′, s_drift)), S′)
						P[Idxᵢ, a.first, LinearIdx(s′′, world)] = α/length(setdiff(S′, (s′, s_drift)))
					end
				elseif α == 1
					for s′′ ∈ (S′..., s_drift)
						P[Idxᵢ, a.first, LinearIdx(s′′, world)] = α/length( (S′..., s_drift) )
					end

				end
			end

		end
	end
	f = zeros(TYPE, worldLen, worldLen, ntuple(i->1, T-2)..., dA, ntuple(i->1,T-1)..., 1)
	for s ∈ 1:worldLen, a ∈ 1:dA, s′ ∈ 1:worldLen
		f[s, s′, ntuple(i->1, T-2)..., a, ntuple(i->1,T-1)..., 1] = P[s, a, s′]
	end

	"MazeDynamicsWithDriftProfile(line: $(@__LINE__)): size(f)=$(size(f)), size(P)=$(size(P))" |> MazeDynamicsWithDriftProfile

	sumP = sum(P, dims=3)
	findall(z->!isapprox(z, 1.0, atol=1e-3), sumP) |> display
	sumP[findall(z->!isapprox(z, 1.0, atol=1e-3), sumP) ] |> display
	(P, f)
end

# RaceDynamics_display(x)  = display(x)
RaceDynamics_display(x)  = nothing 
function RaceDynamicsWithDriftProfile(α, drift::Dict{Tuple, Tuple})

	priorVersion = false
	P = zeros(TYPE, worldLen, dA, worldLen)
	for s ∈ world 
		Idxᵢ = LinearIdx( s, world )
		if s == death
			P[Idxᵢ, :, Idxᵢ] .= 1.0  
			continue 
		end

		s_drift = (s[1:2]..., drift[s]...)

		if α == 1
			s̃ = [s̃..., s_drift]
			for s′′ ∈ s̃
				P[Idxᵢ, :, LinearIdx(s′′, world)] .= ( α )/(length(s̃))
			end
			continue
		end

		for a ∈ Actions
			s′ = UpdateState( s, a, "TEST" )
			if s′ ∉ land
				P[Idxᵢ, a.first, LinearIdx( death, world )] = 1
				continue
			end
			Idxⱼ  = LinearIdx( s′, world )

			P[Idxᵢ, a.first, Idxⱼ] = (1-α)*0.25 

			P[Idxᵢ, a.first, LinearIdx( s_drift, world )] = (1-α)*0.75 

			s̃ = [UpdateState( s, a, "TEST" ) for a ∈ Actions] |> unique
			s̃ = setdiff( s̃, (s′, s_drift) )
			for s′′ ∈ s̃
				P[Idxᵢ, a.first, LinearIdx(s′′, world)] = ( α )/(length(s̃)-0)
			end
			s_next = (-9, -9, -9, -9)#UpdateState(s, a, "TEST")
		end
	end

	f = zeros(TYPE, worldLen, worldLen, ntuple(i->1, T-2)..., dA, ntuple(i->1,T-1)..., 1)
	for s ∈ 1:worldLen, a ∈ 1:dA, s′ ∈ 1:worldLen
		f[s, s′, ntuple(i->1, T-2)..., a, ntuple(i->1,T-1)..., 1] = P[s, a, s′]
	end

	normP = sum(P, dims=3) 
	IdxFull = findall(z->(z == 0.7),  normP)
	for s₁ ∈ world
		i₁ = LinearIdx(s₁, world)
		for s₂ ∈ world
			i₂ = LinearIdx(s₂, world)
			IdxPart = IdxFull[findall(z -> (z.I[1]==i₁ && z.I[3]==i₂), IdxFull)]
			if !isempty(IdxPart)
				"s₁=$(s₁), s₂=$(s₂), $(IdxPart)" |> RaceDynamics_display
			end
		end
	end
	"RaceDynamics(line: $(@__LINE__)): size(f)=$(size(f)), size(P)=$(size(P)), norm(P)=$(sum(P, dims=3))" |> display

	(P, f)
end

# RaceDynamics_display(x) = nothing  ###
# RaceDynamics_display(x) = display(x)
function RaceDynamicsWithDrift(α)

	P = zeros(TYPE, worldLen, dA, worldLen)
	drift_pos = false
	for s ∈ world 
		Idxᵢ = LinearIdx( s, world )
		if s == death
			P[Idxᵢ, :, Idxᵢ] .= 1.0  
			continue 
		end

		s̃ = [UpdateState( s, a, "NOT IN USE IN RACE" ) for a ∈ RaceActions] |> unique

		for a ∈ RaceActions
			s′ = UpdateState( s, a, "NOT IN USE IN RACE" )
			if s′ ∉ land
				P[Idxᵢ, a.first, LinearIdx( death, world )] = 1
				continue
			end
			Idxⱼ  = LinearIdx( s′, world )
			P[Idxᵢ, a.first, Idxⱼ] = α/2 

			s_drift  = ()
			if     s[1] ≤ 4 && s[2] ≤ 4
				if drift_pos
					s_drift = ( (s[1:2] .+ (-1, +1))..., 0, 0)
				else
					s_drift = (s[1:2]..., (-1, +1)...)
				end
				s_drift = s_drift ∈ land ? s_drift : death
			elseif s[1] ≤ 4 && s[2] > 4
				if drift_pos
					s_drift = ( (s[1:2] .+ (+1, +1))..., 0, 0)
				else
					s_drift = (s[1:2]..., (+1, +1)...)
				end
				s_drift = s_drift ∈ land ? s_drift : death
			elseif s[1] > 4 && s[2] > 4
				if drift_pos
					s_drift = ( (s[1:2] .+ (+1, -1))..., 0, 0)
				else
					s_drift = (s[1:2]..., (+1, -1)...)
				end
				s_drift = s_drift ∈ land ? s_drift : death
			elseif s[1] > 4 && s[2] ≤ 4
				if drift_pos
					s_drift = ( (s[1:2] .+ (-1, -1))..., 0, 0)
				else
					s_drift = (s[1:2]..., (-1, -1)...)
				end
				s_drift = s_drift ∈ land ? s_drift : death
			end
			P[Idxᵢ, a.first, LinearIdx( s_drift, world)] = α/1 

			for s′′ in filter(x->(x ∉ (s′, s_drift)), s̃)
				P[Idxᵢ, a.first, LinearIdx(s′′, world)] = (1 - α - α/2)/(length(s̃)-2)
			end
		end
	end
	f = zeros(TYPE, worldLen, worldLen, ntuple(i->1, T-2)..., dA, ntuple(i->1,T-1)..., 1)
	for s ∈ 1:worldLen, a ∈ 1:dA, s′ ∈ 1:worldLen
		f[s, s′, ntuple(i->1, T-2)..., a, ntuple(i->1,T-1)..., 1] = P[s, a, s′]
	end

	"RaceDynamics(line: $(@__LINE__)): size(f)=$(size(f)), size(P)=$(size(P)), norm(P)=$(sum(P, dims=3))" |> RaceDynamics_display
	(P, f)
end

function RaceDynamics(α)

	P = zeros(TYPE, worldLen, dA, worldLen)

	for s ∈ world 
		Idxᵢ = LinearIdx( s, world )
		if s == death
			P[Idxᵢ, :, Idxᵢ] .= 1.0  
			continue 
		end

		s̃ = [UpdateState( s, a ) for a ∈ RaceActions] |> unique

		for a ∈ RaceActions
			s′ = UpdateState( s, a )
			if s′ ∉ land
				P[Idxᵢ, a.first, LinearIdx( death, world )] = 1
				continue
			end
			Idxⱼ  = LinearIdx( s′, world )
			P[Idxᵢ, a.first, Idxⱼ] = α 
			for s′′ in filter(x->(x != s′), s̃)
				P[Idxᵢ, a.first, LinearIdx(s′′, world)] = (1 - α)/(length(s̃)-1)
			end
		end
	end
	f = zeros(TYPE, worldLen, worldLen, ntuple(i->1, T-2)..., dA, ntuple(i->1,T-1)..., 1)
	for s ∈ 1:worldLen, a ∈ 1:dA, s′ ∈ 1:worldLen
		f[s, s′, ntuple(i->1, T-2)..., a, ntuple(i->1,T-1)..., 1] = P[s, a, s′]
	end

	"RaceDynamics(line: $(@__LINE__)): size(f)=$(size(f)), size(P)=$(size(P)), norm(P)=$(sum(P, dims=3))" |> RaceDynamics_display
	(P, f)
end

function RoomBridgeRoomDynamics( dims::Tuple, α )

	P = zeros(TYPE, worldLen, length(BridgeActions), worldLen)

	for s ∈ world
		if s == death
			P[WorldLinearIdx(s, world), :, WorldLinearIdx(s, world)] .= 1
			continue
		end
		for a ∈ BridgeActions
			s′   = s .+ a[2] # with prob α
			### adjacent diagonal actions to a###

			s′′  = s .+ BridgeActions[circshift(1:length(BridgeActions), +1)[a[1]]] # with prob (1-α)/2
			s′′′ = s .+ BridgeActions[circshift(1:length(BridgeActions), -1)[a[1]]] # with prob (1-α)/2

			S = [s′, s′′, s′′′]
			for (i, s_) ∈ enumerate(S)
				if s_ ∈ water 
					S[i] = death
				elseif s_ ∉ (land..., water...) # out of boundaries
					S[i] = s # stay in place
				end
			end
			S = unique(S) # all the states can stay in place
			if s′ ∈ room
				P[WorldLinearIdx(s, world), a[1], WorldLinearIdx(s′, world)] = 1
			else
				if length(S) == 3
					P[WorldLinearIdx(s, world), a[1], WorldLinearIdx(S[1], world)] = α
					P[WorldLinearIdx(s, world), a[1], WorldLinearIdx(S[2], world)] = (1-α)/2
					P[WorldLinearIdx(s, world), a[1], WorldLinearIdx(S[3], world)] = (1-α)/2
				elseif length(S) == 2
					P[WorldLinearIdx(s, world), a[1], WorldLinearIdx(S[1], world)] =     0.7
					P[WorldLinearIdx(s, world), a[1], WorldLinearIdx(S[2], world)] = 1 - 0.7
				elseif length(S) == 1
					P[WorldLinearIdx(s, world), a[1], WorldLinearIdx(S[1], world)] = 1
				end
			end
		end
	end
	f = zeros(TYPE, worldLen, worldLen, ntuple(i->1, T-2)..., length(BridgeActions), ntuple(i->1,T-1)..., 1)
	for s ∈ 1:worldLen, a ∈ 1:length(BridgeActions), s′ ∈ 1:worldLen
		f[s, s′, ntuple(i->1, T-2)..., a, ntuple(i->1,T-1)..., 1] = P[s, a, s′]
	end
	(P, f)
end

function BridgeDynamics( dS::Tuple, dA, α)

	# current state is valid
	for s in filter(validState, (x->x.I).(CartesianIndices( dS )))

		if s in Trap
			Psa_s′[s..., :, s...] .= 1
			continue
		end
		if s in Push # deterministic push of the bridge
			s′ = s
			if s[1] == 2
				s′ = s .+ (-1, 0) # push UP
			elseif s[1] == 4
				s′ = s .+ (+1, 0) # push DOWN
			end
			Psa_s′[s..., :, s′...] .= 1
			continue
		end

		for a in BridgeActions 

			s′   = validState(s  .+    a[2]) ? s  .+    a[2] : s  # move by action 
			s′′  = validState(s′ .+ (-1, 0)) ? s′ .+ (-1, 0) : s′ # move by wind up
			s′′′ = validState(s′ .+ (+1, 0)) ? s′ .+ (+1, 0) : s′ # move by wind down

			Psa_s′[s..., a[1], s′...]   = α
			Psa_s′[s..., a[1], s′′...]  = (1-α)/2
			Psa_s′[s..., a[1], s′′′...] = (1-α)/2
		end 
	end 
	(Psa_s′, DynamicsInLinearStateIndexes(Psa_s′))
end

function DynamicsMazeNoNoise( dS::Tuple, dA, α)

	Psa_s′  = zeros(TYPE, dS..., dA, dS..., 1) # singleton for a′
	Rsa_s′  = -1ones(TYPE, dS..., dA, dS..., 1)
	Goal    = [(dS₁, dS₂)]

	# current state is valid
	for s in filter(inWorld, (x->x.I).(CartesianIndices(dS)))
		if s in Goal
			Psa_s′[s..., :, s..., 1] .= 1
			Rsa_s′[s..., :, s..., 1] .= 0
			continue
		end

		S′ = []
		for a ∈ Actions
			s′ = s .+ a[2]
			if validMove(s, s′, "TRAIN")
				push!(S′, s′)
			end
		end
		S′ = S′ |> unique

		for a in Actions 
			s′ = s .+ a[2]
			if validMove(s, s′, "TRAIN")
				Psa_s′[s..., a[1], s′..., 1] = α
			else
				s′ = s
				if inWorld(s′) # crossing a fence
					Psa_s′[s..., a[1], s′..., 1] = 1
					Rsa_s′[s..., a[1], s′..., 1] = -1000
				else # leaving the world / stay in place
					s′ = s
					Psa_s′[s..., a[1], s′..., 1] = 1
					Rsa_s′[s..., a[1], s′..., 1] = -1000
				end
			end
		end 
	end 
	(Psa_s′, Rsa_s′, DynamicsInLinearStateIndexes(Psa_s′))
end

function RL(dS::Tuple, Psa_s′, Rsa_s′)

	Pa_s = (p_ = rand(dS..., dA, 1, 1, 1); p_ ./ (sum(p_, dims=(3))))
	Qas  = zeros(dS..., dA, 1, 1, 1)
	K = 100

	for k in 1:K

		Pa′_s′   = permutedims(Pa_s, [4, 5, 6, 1, 2, 3])
		Qa′s′    = permutedims(Qas,  [4, 5, 6, 1, 2, 3])

		### dS, dS, dA, dS, dS, dA ###
		Ps′a′_sa = Psa_s′ .* Pa′_s′

		R̃ = sum(Rsa_s′ .* Psa_s′, dims=(4, 5))

		Qas = (R̃ + sum(Ps′a′_sa.*Qa′s′, dims=(4, 5, 6)))

		if k % 5 == 0
			Pa_s = zeros(dS..., dA, 1, 1, 1)
			Pa_s[findmax(Qas, dims=3)[2]] .= 1.0
		end
	end
	Vs = sum(Qas .* Pa_s, dims=3)
	drift = VisualizeRL(Pa_s, Vs)
	(Pa_s, Qas, Vs, drift)
end


function dynamics( dS::Tuple, dA, α)

	Psa_s′ = zeros(TYPE, dS₁, dS₂, dA, dS₁, dS₂)

	# current state is valid
	for s in filter(validState, (x->x.I).(CartesianIndices(dS)))

		if s in Trap
			Psa_s′[s..., :, s...] .= 1
			continue
		end

		# all valid unique next states
		S′ = [validState(s .+ a[2]) ? s .+ a[2] : s for a in Actions] |> unique

		for a in Actions 
			s′ = validState(s .+ a[2]) ? s .+ a[2] : s 
			Psa_s′[s..., a[1], s′...] = α

			# for noisy dynamics
			for s′′ in filter(x->(x != s′), S′)
				Psa_s′[s..., a[1], s′′...] = (1 - α)/(length(S′)-1)	
			end
		end 
	end 
	(Psa_s′, DynamicsInLinearStateIndexes(Psa_s′))
end




function Shift(f::PROBTENSOR, t)

	if t == 0 return f; end

	if t ≥ T error("$(@__LINE__): t ≥ T"); return; end

	perm = 1:ndims(f) |> collect
	### next state ###
	if t ≤ T - 2 
		perm[2] = t + 2
		perm[t + 2] = 2
	else
		perm[2] = 2*T + 1
		perm[2*T + 1] = 2
	end
	f = permutedims(f, perm)
	### current state ###
	perm = 1:ndims(f) |> collect
	perm[1] = t + 1
	perm[t + 1] = 1

	f = permutedims(f, perm)

	### current action ###
	perm = 1:ndims(f) |> collect
	perm[T + 1] = T + 1 + t
	perm[T + t + 1] = T + 1

	f = permutedims(f, perm)

	f
end


# PI_display(x) = display(x) #nothing
PI_display(x) = nothing
function PolicyInit( t::Int, s₀::Tuple)

	### layout(T=3): s₀s₁s₂a₀a₁a₂s₃ ###
	shape = ntuple(i->1, 2T+1)  |> collect 

	"PolicyInit($(@__LINE__)): shape=$(shape)" |> PI_display

	if worldType == "BoxWorld"
		shape[    1:t]     .= dS # for rectangular worlds
	elseif worldType ∈ ("BridgeWorld", "RaceWorld")
		shape[    1:t]     .= worldLen # for others 
		if ~isempty(s₀)
			shape[1] = 1 
		end
	end

	shape[(T+1):(T+t)] .= dA
	π̃ = rand(TYPE, shape...)
	πₜ = π̃ ./ sum(π̃, dims=(T+t))
	πₜ
end

CI_display(x) = nothing 
function ChannelInit(f::PROBTENSOR, T::Int, s₀::Tuple)

	if worldType == "BoxWorld"
		F = [ ones(TYPE, (dS, ntuple(i->1, T-1)..., dA, ntuple(i->1, T-1)..., 1)) ] # 'dynamics placeholder for π(a_0|s_0) .* F[1]' in P₁;
	elseif worldType ∈ ("BridgeWorld", "RaceWorld", "RLMaze", "Rectangle")
		if ~isempty(s₀)
			F = [ ones(TYPE, (       1, ntuple(i->1, T-1)..., dA, ntuple(i->1, T-1)..., 1)) ] # 'dynamics placeholder for π(a_0|s_0) .* F[1]' in P₁;
		else
			F = [ ones(TYPE, (worldLen, ntuple(i->1, T-1)..., dA, ntuple(i->1, T-1)..., 1)) ] # 'dynamics placeholder for π(a_0|s_0) .* F[1]' in P₁;
		end
	end

	f₀ = similar(f)
	if ~isempty(s₀)
		Idx₀ = LinearIdx(s₀, world)
		f₀ = reshape(f[Idx₀, ntuple(i->:, ndims(f)-1)...], (1, size(f)[2:end]...)) 
	end

	for t ∈ 0:(T-1)
		if ~isempty(s₀) && t == 0
			push!(F, Shift(f₀, t))
		else
			push!(F, Shift(f,  t))
		end
	end
	F
end

function P₂(F::Vector{PROBTENSOR})
	F[end]
end

### P₁(S[T+1], S[T...t'+1], A[T...t'+1] | s[t'...0], a[t'...0]) = P₂ * P₁ ### 
function P₃(f::PROBTENSOR, Π::Vector{PROBTENSOR}, t′::Int)
	P₂(f) .* P₁(f, Π, t′)
end

### P = P₁ * P₂ ###
function ReverseChannel(P::PROBTENSOR)
	indexOfZero = findall(iszero, P)
	Q  = P ./ sum(P, dims=(2:(2T)))
	Q[indexOfZero] .= 0
	if ~isempty(findall(isnan, Q))
		"NAN in Q" |> display
	end
	Q
end

### P₁(S[T...t'+1], A[T...t'+1] | s[t'...0], a[t'...0]) ###
# P₁_display(x) = display(x)
P₁_display(x) = nothing
function P₁(F::Vector{PROBTENSOR}, Π::Vector{PROBTENSOR}, t′::Int)
	### t′ ∈ [0...T-1] is the time index.
	"P₁($(@__LINE__)): size(F)=$(size(F)), size(Π)=$(size(Π)), t′=$(t′)" |> P₁_display
	P = Π[T] .* F[T]
	for t ∈ (T-1):-1:(t′+1) #t'=0 is for the first π(a₀|s₀)
		P = P .* (Π[t] .* F[t]) 
	end
	P 
end
PU_display(x) = nothing
# PU_display(x) = display(x)
function PolicyUpdate(F::Vector{PROBTENSOR}, Π::Vector{PROBTENSOR}, Q::PROBTENSOR, t′::Int)

	dims = ( (t′+2):T..., (T+t′+2):2T..., 2T+1 )

	indexOfAₜ = findlast( aₜ->aₜ ≠ 1, size(Π[t′+1])[(T+1):2T] ) + T 
	if t′ == T - 1
		P_ = ones(TYPE, size(Q))
		P  = P₂(F)
	else
		P_ = 	      P₁(F, Π, t′+1)
		P  = P₂(F) .* P_
	end

	πₜ = exp.( sum( log.( ( Q ./ P_ ) .^ P ), dims=dims ) )
	zeroIdx = findall(iszero, πₜ)
	πₜ = πₜ ./ sum(πₜ, dims=indexOfAₜ)
	πₜ[zeroIdx] .= 0.0
	πₜ
end

function PolicyUpdate(F::Vector{PROBTENSOR}, Π::Vector{PROBTENSOR}, Q::PROBTENSOR)

	Π′ = PROBTENSOR[]

	for t′ ∈ 0:(T-1)
		push!(Π′, PolicyUpdate(F, Π, Q, t′))
	end

	Π′
end

ABA_display(x)  = nothing
# function ABA(F::Vector{PROBTENSOR}, s₀::Tuple)
function ABA(f::PROBTENSOR, T::Int, s₀::Tuple)

	F = ChannelInit( f, T, s₀)
	Π = map(t->PolicyInit( t, s₀ ), 1:T) # random initialization of feedback policies.
	P = P₂(F) .* P₁(F, Π, 0) # joint: P(S[T+1], S[T...1], A[T...0] | s[0]) = F[end] * P₁(T); t′ = 0 in the last arg 
	Q = ReverseChannel( P )

	E = []
	for i in 1:nIters
		Π  = PolicyUpdate(F, Π, Q)
		P  = P₂(F) .* P₁(F, Π, 0)
		Q  = ReverseChannel( P )
		if isempty(s₀)
			Eᵢ = MI(F, Π)
			push!(E, Eᵢ) 
		else
			Eᵢ = MI(F, Π)
			push!(E, Eᵢ) 
		end
	end
	(Π, Q, E)
end

function MI(F::Vector{PROBTENSOR}, Π::Vector{PROBTENSOR})

	Pxy = P₂(F) .* P₁(F, Π, 0) # joint:       P(S[T+1], S[T...1], A[T...0] | s[0]) = F[end] * P₁(T); t′ = 0 in the last arg 
	Px  = sum(Pxy, dims=(2T+1)) # marginal of P(        S[T...1], A[T...0] | s[0])
	Py  = sum(Pxy, dims=(2:2T)) # marginal of P(S[T+1]                     | s[0])
	zeroIdx = findall(iszero, Pxy)
	I = (I_ = log.( (Pxy ./ (Px .* Py)) .^ Pxy ); I_[zeroIdx] .= 0; sum(I_, dims=(2:(2T+1))))[:]
end

function OpenLoopEmpowerment3(P, s₀::Tuple)

	MIₒ(F, πᵢ) = (	
	Pxy = F .* πᵢ; # joint:
	Px  = sum(Pxy, dims=ndims(F)); # marginal 
	Py  = sum(Pxy, dims=(2, 3, 4)); # marginal 
	zeroIdx = findall(iszero, Pxy);
	I = (I_ = log.( (Pxy ./ (Px .* Py)) .^ Pxy ); I_[zeroIdx] .= 0; sum(I_, dims=(2:(ndims(F)))));
	I[:]
	)

	if isempty(s₀)
		F = zeros(TYPE, worldLen, ntuple(i->dA, 3)..., worldLen)
	else
		F = zeros(TYPE,        1, ntuple(i->dA, 3)..., worldLen)
	end

	if isempty(s₀)
		S̄₀ = 1:worldLen
	else
		S̄₀ = [s₀]
	end
	A⃗ = (z->z.I).(CartesianIndices((dA, dA, dA)))
	bySampling  = false
	if bySampling 
		for sᵢ ∈ S̄₀
			for a⃗ ∈ A⃗
				for n ∈ 1:1000
					if isempty(s₀)
						w₁= Weights(P[sᵢ, a⃗[1], :])
					else
						w₁= Weights(P[LinearIdx(sᵢ, world), a⃗[1], :])
					end
					s₁ = sample(1:worldLen, w₁)
					w₂= Weights(P[s₁, a⃗[2], :])
					s₂ = sample(1:worldLen, w₂)
					w₃= Weights(P[s₂, a⃗[3], :])
					s₃ = sample(1:worldLen, w₃)
					if isempty(s₀)
						F[sᵢ, a⃗..., s₂] = F[sᵢ, a⃗..., s₂] + 1 
					else
						F[1, a⃗..., s₂] = F[1, a⃗..., s₂] + 1 
					end
				end
			end
		end
		F  = F ./ sum(F, dims=ndims(F))
	else
		if isempty(s₀)
			dimS = worldLen	
			idx₀ = Colon()
		else
			dimS = 1
			idx₀  = LinearIdx(s₀, world)
		end
		### s0 a0 s1 a1 s2 a2 s3 ###
		Ps₀a₀_s₁ = zeros(TYPE, dimS,        dA, worldLen,  1,          1,  1,        1)
		Ps₁a₁_s₂ = zeros(TYPE,    1,         1, worldLen, dA,   worldLen,  1,        1)
		Ps₂a₂_s₃ = zeros(TYPE,    1,         1,        1,  1,   worldLen, dA, worldLen)

		Pa₀      =  ones(TYPE,    1,        dA,        1,  1,          1,  1,        1)/dA	
		Pa₁      =  ones(TYPE,    1,         1,        1, dA,          1,  1,        1)/dA	
		Pa₂      =  ones(TYPE,    1,         1,        1,  1,          1, dA,        1)/dA	

		Ps₀a₀_s₁[:, :, :, 1, 1, 1, 1] = P[idx₀, :, :] 
		Ps₁a₁_s₂[1, 1, :, :, :, 1, 1] = P 
		Ps₂a₂_s₃[1, 1, 1, 1, :, :, :] = P 
		F̄ = sum(Ps₀a₀_s₁ .* Pa₀ .* Ps₁a₁_s₂ .* Pa₁ .* Ps₂a₂_s₃ .* Pa₂, dims=(3, 5))
		F̄ = F̄ ./  sum(F̄, dims=7)
		F[:, :, :, :, :] .= F̄[:, :, 1, :, 1, :, :]
	end
	if isempty(s₀)
		πᵢ = (p = rand(TYPE, worldLen, ntuple(i->dA, 3)..., 1); p ./ sum(p, dims=(2, 3, 4)))
	else
		πᵢ = (p = rand(TYPE,        1, ntuple(i->dA, 3)..., 1); p ./ sum(p, dims=(2, 3, 4)))
	end
	E = []
	for n in 1:nIters 
		q  = ( q = F .* πᵢ; zeroIdx = findall(iszero, q); q = q ./ sum(q, dims=(2, 3, 4)); q[zeroIdx] .= 0.0; q )
		πᵢ = ( p = exp.( sum( log.( q .^ F ), dims=ndims(F)) ); zeroIdx = findall(iszero, p); p = p ./ sum(p, dims=(2, 3, 4)); p[zeroIdx] .= 0.0; p )
		Eᵢ = MIₒ(F, πᵢ)
		push!(E, Eᵢ)
	end
	(MIₒ(F, πᵢ), E)
end
function CloseLoopEmpowerment2(P, s₀::Tuple)

	MIₒ(F, πᵢ) = (	
	Pxy = F .* πᵢ; # joint:
	Px  = sum(Pxy, dims=ndims(F)); # marginal 
	Py  = sum(Pxy, dims=(2, 3)); # marginal 
	zeroIdx = findall(iszero, Pxy);
	I = (I_ = log.( (Pxy ./ (Px .* Py)) .^ Pxy ); I_[zeroIdx] .= 0; sum(I_, dims=(2:(ndims(F)))));
	I[:]
	)

	A⃗ = (z->z.I).(CartesianIndices((dA, dA)))

	if isempty(s₀)
		F = zeros(TYPE, worldLen, ntuple(i->dA, 2)..., worldLen)
	else
		F = zeros(TYPE,        1, ntuple(i->dA, 2)..., worldLen)
	end

	if isempty(s₀)
		S̄₀ = 1:worldLen
	else
		S̄₀ = [s₀]
	end

	bySampling = false
	if bySampling
		for sᵢ ∈ S̄₀
			for a⃗ ∈ A⃗
				for n ∈ 1:5000
					if isempty(s₀)
						w₁= Weights(P[sᵢ, a⃗[1], :])
					else
						w₁= Weights(P[LinearIdx(sᵢ, world), a⃗[1], :])
					end
					s₁ = sample(1:worldLen, w₁)
					w₂ = Weights(P[s₁, a⃗[2], :])
					s₂ = sample(1:worldLen, w₂)
					if isempty(s₀)
						F[sᵢ, a⃗..., s₂] = F[sᵢ, a⃗..., s₂] + 1 
					else
						F[ 1, a⃗..., s₂] = F[ 1, a⃗..., s₂] + 1 
					end
				end
			end
		end
		F  = F ./ sum(F, dims=ndims(F))
	else
		if isempty(s₀)
			dimS = worldLen	
			idx₀ = Colon()
		else
			dimS = 1
			idx₀  = LinearIdx(s₀, world)
		end
		### s0 a0 s1 a1 s2 ###
		Ps₀a₀_s₁ = zeros(TYPE, dimS,        dA, worldLen, 1,         1)
		Ps₁a₁_s₂ = zeros(TYPE,    1,         1, worldLen, dA, worldLen)
		Pa₀      =  ones(TYPE,    1,        dA,        1,  1,        1)/dA	
		Pa₁      =  ones(TYPE,    1,         1,        1, dA,        1)/dA	

		Ps₀a₀_s₁[:, :, :, 1, 1] = P[idx₀, :, :] 
		Ps₁a₁_s₂[1, 1, :, :, :] = P 
		F̄ = sum(Ps₀a₀_s₁ .* Pa₀ .* Ps₁a₁_s₂ .* Pa₁, dims=(3))
		F̄ = F̄ ./  sum(F̄, dims=5)
		F[:, :, :, :] .= F̄[:, :, 1, :, :]
	end
	if isempty(s₀)
		πᵢ = ( p = rand(TYPE, worldLen, ntuple(i->dA, 2)..., 1); p ./ sum(p, dims=(2, 3)) )
	else
		πᵢ = ( p = rand(TYPE, 	     1, ntuple(i->dA, 2)..., 1); p ./ sum(p, dims=(2, 3)) )
	end
	E = []
	for n in 1:nIters 
		q  = ( q = F .* πᵢ; zeroIdx = findall(iszero, q); q = q ./ sum(q, dims=(2, 3)); q[zeroIdx] .= 0.0; q )
		πᵢ = ( p = exp.( sum( log.( q .^ F ), dims=ndims(F)) ); zeroIdx = findall(iszero, p); p = p ./ sum(p, dims=(2, 3)); p[zeroIdx] .= 0.0; p )
		Eᵢ = MIₒ(F, πᵢ)
		push!(E, Eᵢ)
	end
	(MIₒ(F, πᵢ), E)
end

function OpenLoopEmpowerment2(P, s₀::Tuple)

	MIₒ(F, πᵢ) = (	
	Pxy = F .* πᵢ; # joint:
	Px  = sum(Pxy, dims=ndims(F)); # marginal 
	Py  = sum(Pxy, dims=(2, 3)); # marginal 
	zeroIdx = findall(iszero, Pxy);
	I = (I_ = log.( (Pxy ./ (Px .* Py)) .^ Pxy ); I_[zeroIdx] .= 0; sum(I_, dims=(2:(ndims(F)))));
	I[:]
	)

	A⃗ = (z->z.I).(CartesianIndices((dA, dA)))

	if isempty(s₀)
		F = zeros(TYPE, worldLen, ntuple(i->dA, 2)..., worldLen)
	else
		F = zeros(TYPE,        1, ntuple(i->dA, 2)..., worldLen)
	end

	if isempty(s₀)
		S̄₀ = 1:worldLen
	else
		S̄₀ = [s₀]
	end

	bySampling = false
	if bySampling
		for sᵢ ∈ S̄₀
			for a⃗ ∈ A⃗
				for n ∈ 1:5000
					if isempty(s₀)
						w₁= Weights(P[sᵢ, a⃗[1], :])
					else
						w₁= Weights(P[LinearIdx(sᵢ, world), a⃗[1], :])
					end
					s₁ = sample(1:worldLen, w₁)
					w₂ = Weights(P[s₁, a⃗[2], :])
					s₂ = sample(1:worldLen, w₂)
					if isempty(s₀)
						F[sᵢ, a⃗..., s₂] = F[sᵢ, a⃗..., s₂] + 1 
					else
						F[ 1, a⃗..., s₂] = F[ 1, a⃗..., s₂] + 1 
					end
				end
			end
		end
		F  = F ./ sum(F, dims=ndims(F))
	else
		if isempty(s₀)
			dimS = worldLen	
			idx₀ = Colon()
		else
			dimS = 1
			idx₀  = LinearIdx(s₀, world)
		end
		### s0 a0 s1 a1 s2 ###
		Ps₀a₀_s₁ = zeros(TYPE, dimS,        dA, worldLen, 1,         1)
		Ps₁a₁_s₂ = zeros(TYPE,    1,         1, worldLen, dA, worldLen)
		Pa₀      =  ones(TYPE,    1,        dA,        1,  1,        1)/dA	
		Pa₁      =  ones(TYPE,    1,         1,        1, dA,        1)/dA	

		Ps₀a₀_s₁[:, :, :, 1, 1] = P[idx₀, :, :] 
		Ps₁a₁_s₂[1, 1, :, :, :] = P 
		F̄ = sum(Ps₀a₀_s₁ .* Pa₀ .* Ps₁a₁_s₂ .* Pa₁, dims=(3))
		F̄ = F̄ ./  sum(F̄, dims=5)
		F[:, :, :, :] .= F̄[:, :, 1, :, :]
	end
	if isempty(s₀)
		πᵢ = ( p = rand(TYPE, worldLen, ntuple(i->dA, 2)..., 1); p ./ sum(p, dims=(2, 3)) )
	else
		πᵢ = ( p = rand(TYPE, 	     1, ntuple(i->dA, 2)..., 1); p ./ sum(p, dims=(2, 3)) )
	end
	E = []
	for n in 1:nIters 
		q  = ( q = F .* πᵢ; zeroIdx = findall(iszero, q); q = q ./ sum(q, dims=(2, 3)); q[zeroIdx] .= 0.0; q )
		πᵢ = ( p = exp.( sum( log.( q .^ F ), dims=ndims(F)) ); zeroIdx = findall(iszero, p); p = p ./ sum(p, dims=(2, 3)); p[zeroIdx] .= 0.0; p )

		Eᵢ = MIₒ(F, πᵢ)
		push!(E, Eᵢ)
	end
	(MIₒ(F, πᵢ), E)
end


function OpenLoopEmpowerment(P, s′)
	E′ = []
	if T == 2
		I, E′ = OpenLoopEmpowerment2(P, s′)
	elseif T == 3
		I, E′ = OpenLoopEmpowerment3(P, s′)
	end
	(I, E′)
end

function Controller(P, f::PROBTENSOR, T::Int, s::Tuple, lenOfTraj::Int, empType::String, EmpPreCalc=Dict{Tuple, Int}())

	Ē = []
	S = []

	"Controller($(@__LINE__)) with length(EmpPreCalc)=$(length(EmpPreCalc))" |> display

	for t in 1:lenOfTraj
		E  = []
		S′ = []
		A  = []
		for a ∈ ActionsTest
			s′ = UpdateState(s, a, "TEST")
			E′ = 0
			if empType == "Process Empowerment" 
				E′ = isempty(EmpPreCalc) ? (ABA(f, T, s′)[end])[end] : EmpPreCalc[s′] 	
			elseif empType == "Open-Loop Empowerment" 
				E′ = isempty(EmpPreCalc) ? (OpenLoopEmpowerment(P, s′)[end])[end] : EmpPreCalc[s′]
			end
			push!(E, E′)
			push!(S′, s′)
			push!(A, a.second)
		end

		if death ∈ S′
			"Controller($(@__LINE__)), DEATH in S′ => break" |> display
			break
		end

		maxIdx = findmax(E)[2]
		s = S′[maxIdx]
		push!(S, s)
		push!(Ē, E[maxIdx])
	end
	(S, Ē)	
end

function FullLandscape(P, f)

	"FullLandscape($(@__LINE__)): START" |> display

	s  = () # full landscape   

	Ē₁ = (ABA(f, T, s)[end])[end] 	
	Ē₂ = (OpenLoopEmpowerment(P, s)[end])[end] 	
	"FullLandscape($(@__LINE__)): DONE" |> display

	### Dict is expected in Controller ###
	E₁ = Dict{Tuple, TYPE}()
	E₂ = Dict{Tuple, TYPE}()
	for s ∈ world 
		idx = LinearIdx(s, world)
		E₁[s] = Ē₁[idx]
		E₂[s] = Ē₂[idx]
	end

	(E₁, E₂)
end

function BridgeExperiment(P, f, αᵢ, α, plotsPath, fileName)
	### make landscapes ###
	E₁ = []
	E₂ = []
	s  = () # full landscape   

	E₁ = (ABA(f, T, s)[end])[end] 	
	if T == 2
		E₂ = ( OpenLoopEmpowerment2(P, s)[end] )[end] 	
	elseif T == 3
		E₂ = ( OpenLoopEmpowerment3(P, s)[end] )[end] 	
	end

	E₁  = Visualize(E₁[:], land)
	E₂  = Visualize(E₂[:], land)

	"BridgeExperiment($(@__LINE__)): size(E₁)=$(size(E₁))" |> display
	"BridgeExperiment($(@__LINE__)): size(E₁)=$(size(E₂))" |> display
	"BridgeExperiment($(@__LINE__)): α=$(α)" |> display

	E₁ |> display
	E₂ |> display


	p1 = heatmap(E₁, title="Process Empowerment, α=$(round(α, digits=1))", colorbar=true, showaxis = true, aspect_ratio=:equal, ticks = true, framestyle = :box, widen=false, titlefontsize = 17, tickfontsize=15, xticks=1:dS₂, yticks=1:dS₁, thickness_scaling = 1, colorbar_tickfontsize=13, colorbar_title="nats", colorbar_titlefontsize=17)
	###
	X = []
	Y = []
	V = []
	U = []
	drift_scale = 0.4
	for d in bridge
		for noise in (z->z.I).(CartesianIndices((-1:1, -1:1)))
			push!(X, d[2])
			push!(Y, d[1])
			if abs(noise[1]) == abs(noise[2])
				push!(U, drift_scale*noise[1]/1.4)
				push!(V, drift_scale*noise[2]/1.4)
			else
				push!(U, drift_scale*noise[1])
				push!(V, drift_scale*noise[2])
			end
		end
	end
	quiver!(X, Y, quiver=(V, U); xticks=1:dS₂, yticks=1:dS₁, color = :blue, markersize=0.1, lw=0.7, arrow=Plots.arrow(:open, :head, 0.5, 0.5))
	scatter!([1], [(dS₁+1)/2], color= :green, label=:none, legend=false, ms=15)

	plot!(1:dS₂, E₁[(dS₁+1)/2 |> Int, :], framestyle = :box, titlefontsize = 12, tickfontsize=15, xticks=1:dS₂, thickness_scaling = 1, fontsize=15, lw=3, lc=:blue, markershape=:circle, ms=8, mc=:green, legend=false, labelfontsize=17, legendfontsize = 15, legend_position=:topleft,grid=true, inset=bbox(0.59,0.61,0.2,0.2, :bottom, :left), subplot=2, background_color=:transparent )

	@show savefig(plotsPath * fileName * "ProcessAll" * ".png")
	@save plotsPath * fileName * "ProcessAll" * ".bson" 

	p2 = heatmap(E₂, title="Open-Loop Empowerment, α=$(round(α, digits=1))", colorbar=true, showaxis = true, aspect_ratio=:equal, ticks = true, framestyle = :box, widen=false, titlefontsize=17, tickfontsize=15, xticks=1:dS₂, yticks=1:dS₁, thickness_scaling = 1, colorbar_tickfontsize=13, colorbar_title="nats", colorbar_titlefontsize=17)

	quiver!(X, Y, quiver=(V, U); color = :blue, xticks=1:dS₂, yticks=1:dS₁, markersize=0.1, lw=0.7, arrow=Plots.arrow(:open, :head, 0.5, 0.5))
	scatter!([1], [(dS₁+1)/2], color= :green, label=:none, legend=false, ms=15)

	plot!(1:dS₂, E₂[(dS₁+1)/2 |> Int, :], framestyle = :box, titlefontsize = 12, tickfontsize=15, xticks=1:dS₂, thickness_scaling = 1, fontsize=15, lw=3, lc=:blue, markershape=:circle, ms=8, mc=:green, legend=false, labelfontsize=17, legendfontsize = 15, legend_position=:topleft,grid=true, inset=bbox(0.59,0.61,0.2,0.2, :bottom, :left), subplot=2, background_color=:transparent )
	#
	###

	@show savefig(plotsPath * fileName * "OpenAll" * ".png")
	@save plotsPath * fileName * "OpenAll" * ".bson" 

	(E₁, E₂)
end

function RaceExperiment(P, f, αᵢ, α, plotsPath, fileName)
	H₁ = []
	H₂ = []
	lenOfTraj = 30 # ~30
	numOfTraj = 50 # ~50 for statistics
	for i in 1:numOfTraj

		s₀ = (sample(land)[1:2]..., 0, 0)

		empType = "Process Empowerment"
		Trj, E = Controller(P, f, T, s₀, lenOfTraj, empType)
		fileName  = "$(worldType)_$(s₀)_$(empType)_T$(T)_α$(α)_dS1-$(dS₁)_dS2-$(dS₂)_nIter$(nIters)_lenOfTraj$(lenOfTraj)"
		VisualizeTrajectoryOnLand(land, Trj, plotsPath, fileName, empType, drift)
		push!(H₁, length(unique(Trj)))

		empType = "Open-Loop Emppowerment" 
		Trj, E = Controller(P, f, T, s₀, lenOfTraj, empType)
		fileName  = "$(worldType)_$(s₀)_$(empType)_T$(T)_α$(α)_dS1-$(dS₁)_dS2-$(dS₂)_nIter$(nIters)_lenOfTraj$(lenOfTraj)"
		VisualizeTrajectoryOnLand(land, Trj, plotsPath, fileName, empType, drift)
		push!(H₂, length(unique(Trj)))
	end
	makeTrajLengthHisto = false
	if makeTrajLengthHisto
		histogram(H₁, bins=10, color=:green, label="Process")
		histogram!(H₂, bins=10, color=:blue, label="Open Loop")
		@show savefig(plotsPath * fileName * "Histo" * ".png")
		@save plotsPath * fileName * "Histo" * ".bson" 
	end

	### Empowerment for the maximal velocity 	 ###
	makeLandscapeOnLand = true
	AverageByActions = true
	if makeLandscapeOnLand
		"RaceExperiment($(@__LINE__)): makeLandscapeOnLand=$(makeLandscapeOnLand)" |> display

		landNoVel = []
		for s ∈ land
			push!(landNoVel, s[1:2])
		end
		landNoVel = landNoVel |> unique 

		M₁ = zeros(Float64, dS₁, dS₂)
		M₂ = zeros(Float64, dS₁, dS₂)
		for s ∈ landNoVel
			E₁ = []
			E₂ = []
			S̄  = []
			if AverageByActions == false
				for velocity ∈ [(-1, +1)] # aligned with drift
					s̄ = (s..., velocity...)
					E = ABA(f, T, s̄)[end][end][:]
					push!(E₁, E[1])

					E = OpenLoopEmpowerment2(P, s̄)[end][end][:]
					push!(E₂, E[1])
				end
			elseif AverageByActions == true
				for velocity ∈ (z->z.I).(CartesianIndices((-v̄₁:+v̄₁, -v̄₂:+v̄₂))) # all velocities 
					for a ∈ ActionsTest
						s̄ = UpdateState((s..., velocity...), a, "TEST")
						if s̄ == death 
							continue
						end
						E = ABA(f, T, s̄)[end][end][:]
						push!(E₁, E[1])

						E = OpenLoopEmpowerment2(P, s̄)[end][end][:]
						push!(E₂, E[1])
						push!(S̄, s̄)
					end
				end
			end
			M₁[s...] = sum(E₁)/length(E₁)
			M₂[s...] = sum(E₂)/length(E₂)

		end
		heatmap(1:dS₂, 1:dS₁, M₁, 
		size=(1000, 1000),
		title="Process", colorbar= true, showaxis = true, aspect_ratio=:equal, ticks = true, framestyle = :box, widen=false, tickfontsize=15, xticks=1:dS₂, yticks=1:dS₁, thickness_scaling = 2, yflip=true)
		@show savefig(plotsPath * "ProcessEmpForMaxVelocity" * fileName * ".png")

		heatmap(1:dS₂, 1:dS₁, M₂, 
		size=(1000, 1000),
		title="Open Loop", colorbar= true, showaxis = true, aspect_ratio=:equal, ticks = true, framestyle = :box, widen=false, tickfontsize=15, xticks=1:dS₂, yticks=1:dS₁, thickness_scaling = 2, yflip=true)
		@show savefig(plotsPath * "OpenLoopEmpForMaxVelocity" * fileName * ".png")
	end
	"RaceExperiment($(@__LINE__)): DONE" |> display
end


P = []
f = []
α = 0.5#with drift α < 3/4
α = 2/3 - 1e-5
TT₁ = []
TT₂ = []
EE₁ = []
EE₂ = []
if worldType == "RaceWorld" && false
	drift = RaceDynamicsDriftProfile()

	αᵢ = 1

	lenOfTraj = 200
	s₀ = (sample(land)[1:2]..., 0, 0)
	plotsPath = " you path "
	for ζ ∈ (0.2,), α ∈ (0.0, 0.25)
		"α=$(α), ζ=$(ζ)" |> display
		(P, f) = MazeDynamicsWithDriftProfile(α, drift, ζ)
		if true
			empType = "Open-Loop Empowerment" 
			Trj, E = Controller(P, f, T, s₀, lenOfTraj, empType)
			push!(TT₁, Trj)
			push!(EE₁,   E)

			mytitle = empType
			fileName  = "$(worldType)_α$(α)_ζ$(ζ)_$(s₀)_$(mytitle)_T$(T)_α$(α)_dS1-$(dS₁)_dS2-$(dS₂)_nIter$(nIters)_lenOfTraj$(lenOfTraj)"
			VisualizeTrajectoryOnLand(land, Trj, plotsPath, fileName, mytitle * ", μ=$(round(α, digits=2)), ζ=$(ζ)", drift)

			empType = "Process Empowerment"
			Trj, E = Controller(P, f, T, s₀, lenOfTraj, empType)
			push!(TT₂, Trj)
			push!(EE₂,   E)

			mytitle = empType
			fileName  = "$(worldType)_α$(α)_ζ$(ζ)_$(s₀)_$(mytitle)_T$(T)_α$(α)_dS1-$(dS₁)_dS2-$(dS₂)_nIter$(nIters)_lenOfTraj$(lenOfTraj)"
			VisualizeTrajectoryOnLand(land, Trj, plotsPath, fileName, mytitle * ", μ=$(round(α, digits=2)), ζ=$(ζ)", drift)
		end
	end
elseif worldType == "BridgeWorld"
	plotsPath = " your path "
	runBridgeExperiment = true
	Ē₁ = []
	Ē₂ = []
	α = 0.0

	if runBridgeExperiment
		A = (0.3)
		for (αᵢ, α) ∈ enumerate(A)
			(P, f) =  RoomBridgeRoomDynamics( (dS₁, dS₂), α )
			fileName  = "$(worldType)_T$(T)_α$(α)_αᵢ$(αᵢ)_dS1-$(dS₁)_dS2-$(dS₂)_nIter$(nIters)"
			(E₁, E₂) = BridgeExperiment(P, f, αᵢ, α, plotsPath, fileName)
			push!(Ē₁, sum(E₁)/length(land))
			push!(Ē₂, sum(E₂)/length(land))
		end

		(A, Ē₁, Ē₂) |> display

		plot( 1.0 .- (A), [Ē₁ Ē₂], label=["Process" "Open-Loop"], lw=3, tickfontsize=15, xticks=round.(1.0 .- (A), digits=1), legendfontsize=15, lagend_position=:best, framestyle = :box, widen = false, ylabel="Empowerment in nats", title="Noise Robustness Comparison", labelfontsize=17, xlabel="α (noise level)", titlefontsize=17)

		fileName  = "$(worldType)_T$(T)_αAll_dS1-$(dS₁)_dS2-$(dS₂)_nIter$(nIters)"
		@show savefig(plotsPath * fileName * "Robustness" * ".png")
		@save plotsPath * fileName * "Robustness" * ".bson" A Ē₁ Ē₂
	end

elseif worldType == "RLMaze"

	α = 1.0 # to build a deterministic maze
	Psa_s′, Rsa_s′, f = DynamicsMazeNoNoise( (dS₁, dS₂), dA, α)
	Pa_s, Qas, Vs, drift = RL((dS₁, dS₂), Psa_s′, Rsa_s′)

	lenOfTraj = 100	

	for α ∈ (0.0, 0.2, 0.5), ζ ∈ (0.01, 0.1, 0.25)

		"α=$(α), ζ=$(ζ)" |> display 
		(P, f) = MazeDynamicsWithDriftProfile(α, drift, ζ)

		s₀ = (2, 1, 0, 0)

		empType = "Open-Loop Empowerment" 
		Trj, E = Controller(P, f, T, s₀, lenOfTraj, empType)
		push!(TT₁, Trj)
		push!(EE₁,   E)

		mytitle = empType
		fileName  = "$(worldType)_$(s₀)_$(mytitle)_T$(T)_α$(α)_dS1-$(dS₁)_dS2-$(dS₂)_nIter$(nIters)_lenOfTraj$(lenOfTraj)_ζ$(ζ)"
		VisualizeTrajectoryOnLand(land, Trj, plotsPath, fileName, mytitle * ", μ=$(round(α, digits=2))" * ", ζ=$(round(ζ, digits=2))", drift)

		empType = "Process Empowerment"
		Trj, E = Controller(P, f, T, s₀, lenOfTraj, empType)
		push!(TT₂, Trj)
		push!(EE₂,   E)

		mytitle = empType
		fileName  = "$(worldType)_$(s₀)_$(mytitle)_T$(T)_α$(α)_dS1-$(dS₁)_dS2-$(dS₂)_nIter$(nIters)_lenOfTraj$(lenOfTraj)_ζ$(ζ)"
		VisualizeTrajectoryOnLand(land, Trj, plotsPath, fileName, mytitle * ", μ=$(round(α, digits=2))" * ", ζ=$(round(ζ, digits=2))", drift)
	end
elseif worldType == "Rectangle"

	"Rectangle: (dS₁, dS₂)=$((dS₁, dS₂)), Actions=$(Actions), size(world)=$(size(world)), ObstacleSetTest=$(ObstacleSetTest), ObstacleSetTrain=$(ObstacleSetTrain)" |> display

	drift = RectangleDriftProfile()

	lenOfTraj = 15
	E₁_avrg = Dict()
	E₂_avrg = Dict()

	αD = 2# 20
	ζD = 1# 20
	αRange = (0.0, 0.2)# range(0.0, 0.5, length=αD)
	ζRange = (0.3,)#range(0.0, 1.0, length=ζD) 

	TrjArr₁ = []
	TrjArr₂ = []
	E₁ = ()
	E₂ = ()

	for α ∈ αRange, ζ ∈ ζRange 
		for numExp ∈ 1:5

			"α=$(α), ζ=$(ζ)" |> display 

			(P, f) = MazeDynamicsWithDriftProfile(α, drift, ζ)

			doVisuals = true
			if doVisuals
				s₀ = (2, 1, 0, 0)
				empType = "Open-Loop Empowerment" 
				Trj, E = Controller(P, f, T, s₀, lenOfTraj, empType, E₂)

				push!(TrjArr₂, Trj)

				mytitle = "Open-Loop"
				fileName  = "numExp$(numExp)_$(worldType)_α$(α)_ζ$(ζ)_$(s₀)_$(mytitle)_T$(T)_α$(α)_dS1-$(dS₁)_dS2-$(dS₂)_nIter$(nIters)_lenOfTraj$(length(Trj |> unique))"
				VisualizeTrajectoryOnLand(land, Trj, plotsPath, fileName, mytitle * ", α=$(round(α, digits=2))" * ", ζ=$(round(ζ, digits=2))", drift)

				empType = "Process Empowerment"
				Trj, E = Controller(P, f, T, s₀, lenOfTraj, empType, E₁)

				push!(TrjArr₁, Trj)

				mytitle = "Process"
				fileName  = "numExp$(numExp)_$(worldType)_α$(α)_ζ$(ζ)_$(s₀)_$(mytitle)_T$(T)_α$(α)_dS1-$(dS₁)_dS2-$(dS₂)_nIter$(nIters)_lenOfTraj$(length(Trj |> unique))"
				VisualizeTrajectoryOnLand(land, Trj, plotsPath, fileName, mytitle * ", α=$(round(α, digits=2))" * ", ζ=$(round(ζ, digits=2))", drift)
			end
		end
	end

	avrg(x) = sum(x)/lenght(x)

	TrjArr₁_avrg = 0
	TrjArr₂_avrg = 0
	for i in 1:length(TrjArr₁)
		TrjArr₁_avrg = TrjArr₁_avrg + (TrjArr₁ |> unique) 
		TrjArr₂_avrg = TrjArr₂_avrg + (TrjArr₂ |> unique) 
	end
	"Avrg₁ = $(TrjArr₁_avrg/length(TrjArr₁))" |> display 
	"Avrg₂ = $(TrjArr₂_avrg/length(TrjArr₂))" |> display 


	E₁_avrg |> display
	E₂_avrg |> display

	EEE1 = Array{Float64, 2}(undef, αD, ζD);
	EEE2 = Array{Float64, 2}(undef, αD, ζD);

	for (i, α) ∈ enumerate(αRange), (j, ζ) ∈ enumerate(ζRange)
		EEE1[i,j] = E₁_avrg[(α, ζ)]
		EEE2[i,j] = E₂_avrg[(α, ζ)]
	end

	EEE1 |> display
	EEE2 |> display 

	mytitle = "Open-Loop Empowerment"
	fileName  = "Robustness_$(worldType)_$(mytitle)_T$(T)_α$(α)_dS1-$(dS₁)_dS2-$(dS₂)_nIter$(nIters)_lenOfTraj$(lenOfTraj)"
	heatmap(αRange, ζRange, EEE2', ylims=(0.0, 1.0), xticks=round.(αRange[1:4:end], digits=1), yticks=round.([ζRange[1:4:end]..., 1.0], digits=1), xflip=true, size=(600, 600),
	title=mytitle, ylabel="ζ (mixture coefficient)", xlabel="α (noise level)", framestyle = :box, widen=false, labelfontsize=17,  tickfontsize=19, titlefontsize=17, thickness_scaling = 1,colorbar_tickfontsize=17, colorbar_title="nats", colorbar_titlefontsize=17) 

	@show savefig(plotsPath * fileName * "_manual" * ".png")
	@save plotsPath * "/BSON/" * fileName * "_manual" * ".bson" EEE1 E₁_avrg

	mytitle = "Process Empowerment"
	fileName  = "Robustness_$(worldType)_$(mytitle)_T$(T)_α$(α)_dS1-$(dS₁)_dS2-$(dS₂)_nIter$(nIters)_lenOfTraj$(lenOfTraj)"
	heatmap(αRange, ζRange, EEE1', xticks=round.(αRange[1:4:end], digits=1), yticks=round.([ζRange[1:4:end]..., 1.0], digits=1), xflip=true, size=(600, 600),
	title=mytitle, ylabel="ζ (mixture coefficient)", xlabel="α (noise level)", framestyle = :box, widen=false, labelfontsize=17,  tickfontsize=19, titlefontsize=17, thickness_scaling = 1,colorbar_tickfontsize=17, colorbar_title="nats", colorbar_titlefontsize=17) 

	@show savefig(plotsPath * fileName * "_manual" * ".png")
	@save plotsPath * "/BSON/" * fileName * "_manual" * ".bson" EEE2 E₂_avrg

elseif worldType == "RaceWorld"

	"$(worldType): (dS₁, dS₂)=$((dS₁, dS₂)), Actions=$(Actions), size(world)=$(size(world)), ObstacleSetTest=$(ObstacleSetTest), ObstacleSetTrain=$(ObstacleSetTrain)" |> display

	drift = RaceDynamicsDriftProfile()

	lenOfTraj = 50
	E₁_avrg = Dict()
	E₂_avrg = Dict()

	αD = 2
	ζD = 1
	αRange = (0.0, 0.25)
	ζRange = (0.2,)

	TrjArr₁ = []
	TrjArr₂ = []

	E₁ = ()
	E₂ = ()
	for α ∈ αRange, ζ ∈ ζRange 

		for numExp ∈ 1:3

			"α=$(α), ζ=$(ζ)" |> display 

			(P, f) = MazeDynamicsWithDriftProfile(α, drift, ζ)

			doVisuals = true
			if doVisuals
				s₀ = (4, 1, 0, 0)
				empType = "Open-Loop Empowerment" 
				Trj, E = Controller(P, f, T, s₀, lenOfTraj, empType, E₂)

				push!(TrjArr₂, Trj)

				mytitle = "Open-Loop"
				fileName  = "numExp$(numExp)_$(worldType)_α$(α)_ζ$(ζ)_$(s₀)_$(mytitle)_T$(T)_α$(α)_dS1-$(dS₁)_dS2-$(dS₂)_nIter$(nIters)_lenOfTraj$(length(Trj |> unique))"
				VisualizeTrajectoryOnLand(land, Trj, plotsPath, fileName, mytitle * ", α=$(round(α, digits=2))" * ", ζ=$(round(ζ, digits=2))", drift)

				empType = "Process Empowerment"
				Trj, E = Controller(P, f, T, s₀, lenOfTraj, empType, E₁)

				push!(TrjArr₁, Trj)

				mytitle = "Process"
				fileName  = "numExp$(numExp)_$(worldType)_α$(α)_ζ$(ζ)_$(s₀)_$(mytitle)_T$(T)_α$(α)_dS1-$(dS₁)_dS2-$(dS₂)_nIter$(nIters)_lenOfTraj$(length(Trj |> unique))"
				VisualizeTrajectoryOnLand(land, Trj, plotsPath, fileName, mytitle * ", α=$(round(α, digits=2))" * ", ζ=$(round(ζ, digits=2))", drift)
			end
		end
	end

	avrg(x) = sum(x)/lenght(x)

	TrjArr₁_avrg = 0
	TrjArr₂_avrg = 0
	for i in 1:length(TrjArr₁)
		TrjArr₁_avrg = TrjArr₁_avrg + (TrjArr₁ |> unique) 
		TrjArr₂_avrg = TrjArr₂_avrg + (TrjArr₂ |> unique) 
	end
	"Avrg₁ = $(TrjArr₁_avrg/length(TrjArr₁))" |> display 
	"Avrg₂ = $(TrjArr₂_avrg/length(TrjArr₂))" |> display 

	doRobustness = false
	if doRobustness

		E₁_avrg |> display
		E₂_avrg |> display

		EEE1 = Array{Float64, 2}(undef, αD, ζD);
		EEE2 = Array{Float64, 2}(undef, αD, ζD);

		for (i, α) ∈ enumerate(αRange), (j, ζ) ∈ enumerate(ζRange)
			EEE1[i,j] = E₁_avrg[(α, ζ)]
			EEE2[i,j] = E₂_avrg[(α, ζ)]
		end

		EEE1 |> display
		EEE2 |> display 

		mytitle = "Open-Loop Empowerment"
		fileName  = "Robustness_$(worldType)_$(mytitle)_T$(T)_α$(α)_dS1-$(dS₁)_dS2-$(dS₂)_nIter$(nIters)_lenOfTraj$(lenOfTraj)"
		heatmap(αRange, ζRange, EEE2', ylims=(0.0, 1.0), xticks=round.(αRange[1:4:end], digits=1), yticks=round.([ζRange[1:4:end]..., 1.0], digits=1), xflip=true, size=(600, 600),
		title=mytitle, ylabel="ζ (mixture coefficient)", xlabel="α (noise level)", framestyle = :box, widen=false, labelfontsize=17,  tickfontsize=19, titlefontsize=17, thickness_scaling = 1,colorbar_tickfontsize=17, colorbar_title="nats", colorbar_titlefontsize=17) 

		@show savefig(plotsPath * fileName * "_manual" * ".png")
		@save plotsPath * "/BSON/" * fileName * "_manual" * ".bson" EEE1 E₁_avrg

		mytitle = "Process Empowerment"
		fileName  = "Robustness_$(worldType)_$(mytitle)_T$(T)_α$(α)_dS1-$(dS₁)_dS2-$(dS₂)_nIter$(nIters)_lenOfTraj$(lenOfTraj)"
		heatmap(αRange, ζRange, EEE1', xticks=round.(αRange[1:4:end], digits=1), yticks=round.([ζRange[1:4:end]..., 1.0], digits=1), xflip=true, size=(600, 600),
		title=mytitle, ylabel="ζ (mixture coefficient)", xlabel="α (noise level)", framestyle = :box, widen=false, labelfontsize=17,  tickfontsize=19, titlefontsize=17, thickness_scaling = 1,colorbar_tickfontsize=17, colorbar_title="nats", colorbar_titlefontsize=17) 

		@show savefig(plotsPath * fileName * "_manual" * ".png")
		@save plotsPath * "/BSON/" * fileName * "_manual" * ".bson" EEE2 E₂_avrg
	end
end

if worldType != "BridgeWorld" && false

	size(land) |> display

	for experiment ∈ 1:1
		"experiment=$(experiment)" |> display
		if worldType == "Rectangle"
			s₀ = (3, 1, 0, 0)
			plotsPath = " your path " 
		elseif worldType == "RaceWorld"
			s₀ = (sample(land)[1:2]..., 0, 0)
			plotsPath = " your path " 
		elseif worldType == "RLMaze"
			s₀ = (1, 1, 0, 0)
			plotsPath = " your path " 
		end

		lenOfTraj = 35

		empType = "Process Empowerment"
		Trj, E = Controller(P, f, T, s₀, lenOfTraj, empType)

		mytitle = empType
		fileName  = "$(worldType)_$(s₀)_$(mytitle)_T$(T)_α$(α)_dS1-$(dS₁)_dS2-$(dS₂)_nIter$(nIters)_lenOfTraj$(lenOfTraj)"
		VisualizeTrajectoryOnLand(land, Trj, plotsPath, fileName, mytitle, drift)

		empType = "Open-Loop Empowerment" 
		Trj, E = Controller(P, f, T, s₀, lenOfTraj, empType)

		mytitle = empType
		fileName  = "$(worldType)_$(s₀)_$(mytitle)_T$(T)_α$(α)_dS1-$(dS₁)_dS2-$(dS₂)_nIter$(nIters)_lenOfTraj$(lenOfTraj)"
		VisualizeTrajectoryOnLand(land, Trj, plotsPath, fileName, mytitle, drift)
	end
end

