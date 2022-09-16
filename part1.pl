% Yanran Wang
% z5295965

% Q1.1 List Processing

even_num(X):- 
    0 is X mod 2.

% special case when the list is empty
sumsq_even([], 0).
% main case 
sumsq_even([H|T], Sum):- 
    even_num(H),
    sumsq_even(T, Sum_2),
    Sum is Sum_2 + H*H;
    sumsq_even(T, Sum).


% Q1.2 Planning

is_mc(lab, mr).
is_mc(mr, cs).
is_mc(cs, off).
is_mc(off, lab).
is_mcc(lab, off).
is_mcc(off, cs).
is_mcc(cs, mr).
is_mcc(mr, lab).

% Q1.2 Planning

% State of the robot's world = state(RobotLocation, Robot Has Coffee, Sam Wants Coffee, Mail Waiting, Robot Has Mail)
% action(Action, State, NewState): Action in State produces NewState
% We assume robot never drops rubbish on floor and never pushes rubbish around

action( mc,						% robot move clockwise
	state(Pos1, X, Y, Z, D),			% Before action, robot at original position
	state(Pos2, X, Y, Z, D)) :-
    is_mc(Pos1, Pos2).					% After action, robot move to new position


action( mcc,						% robot move counterclockwise
	state(Pos1, X, Y, Z, D),			% Before action, robot at original position
	state(Pos2, X, Y, Z, D)) :-
    is_mcc(Pos1, Pos2).					% After action, robot move to new position


action( puc,					% robot pick up coffee
	state(cs, false, X, Y, Z),			% Before action, robot does not have coffee
	state(cs, true, X, Y, Z)).			% After action, robot has coffee

action( dc,				% robot deliever coffee
	state(off, true, _, Y, Z),			% Before action, robot has coffee
	state(off, false, false, Y, Z)).	% After action, robot has no coffee and sam doesn't want coffee

action( pum,			% robot pickup mail	
	state(mr, X, Y, true, false),		% Before action, robot does not have mail	
	state(mr, X, Y, false, true)).		% After action, robot picks up the mail

action( dm,				% robot deliver mail	
	state(off, X, Y, Z, true),			% Before action, robot has mail	
	state(off, X, Y, Z, false)).		% After action, robot delivers the mail

% plan(StartState, FinalState, Plan)

plan(State, State, []).				% To achieve State from State itself, do nothing

plan(State1, GoalState, [Action1 | RestofPlan]) :-
	action(Action1, State1, State2),		% Make first action resulting in State2
	plan(State2, GoalState, RestofPlan). 		% Find rest of plan

% Iterative deepening planner
% Backtracking to "append" generates lists of increasing length
% Forces "plan" to ceate fixed length plans

id_plan(Start, Goal, Plan) :-
    append(Plan, _, _),
    plan(Start, Goal, Plan).


%  Q1.3 Inductive Logic Programming

:- op(300, xfx, <-).
% Intra-construction
intra_construction(C1 <- B1, C2 <- B2, C1 <- Z1B, C <- Z2B, C <- B) :-
    C1 = C2,
	intersection(B1, B2, X),
	gensym(z, C),
	subtract(B1, X, Z2B),
	subtract(B2, X, B),
	append(X, [C], Z1B).

% absorption
absorption(C1 <- B1, C2 <- B2, C1 <- X1, C2 <- B2) :-
    C1 \= C2,
    subset(B2, B1),
    intersection(B1, B2, O),
    subtract(B1, O, Z),
    append([C2], Z, X1).
    
% Truncation
truncation(C1 <- B1, C2 <- B2, C2 <- X) :-
    C1 = C2,
    intersection(B1, B2, X).
