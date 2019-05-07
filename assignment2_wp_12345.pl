% candidate_number(12345).

% Find hidden identity by repeatedly calling agent_ask_oracle(oscar,o(1),link,L)
% find_identity(-A)
find_identity(A):-
  (part_module(2)   -> find_identity_2(A);
  otherwise  -> find_identity_o(A)
% otherwise -> writeln("Not implemented yet sry").
  ).

% Asking the oracle to find out the agent's identity
find_identity_2(A):-
  findall(Actor, actor(Actor), Actors),
  find_actor_identity(A, Actors).

% Predicate for recursively asking the oracle for the link and identify the new actors by filtering
% If it succeeds it returns the actor name (A).
find_actor_identity(A, Actors) :-
  agent_ask_oracle(oscar, o(1), link, L),
  include(link_on_page(L), Actors, NewActors),
  length(NewActors, Length),
  (Length = 1 -> nth0(0, NewActors, A);
   find_actor_identity(A, NewActors)
  ).

link_on_page(Link,A) :-
  links_on_page(A, Links),
  member(Link, Links).

%Generating links for one actor
links_on_page(A, Links):-
  actor(A),
  setof(Link, (link(Link), wp(A,WT),wt_link(WT,Link)), Links).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Part 3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate the actor links, find the location of all charging stations and oracles.
% P: Position; L:List.

find_identity_o(A):-
  findall(Actor, actor(Actor), Actors),
  find_actor_identity_o(A, Actors).

find_actor_identity_o(A, Actors) :-
  my_agent(Agent),
  solve_task_general(find_next_one(o(X)), _),
  query_world( agent_ask_oracle, [Agent, o(X), link, L]),
  include(link_on_page(L), Actors, NewActors),
  length(NewActors, Length),
  ( Length = 1 -> nth0(0, NewActors, A)
  ; find_actor_identity_o(A, NewActors)
  ).


%find_identity_o(A):-
%  generate_actor_link_list(ActorList),
%  my_agent(Agent),
%  writeln("1. Finding Charging Station locations now..."),
%  query_world(agent_current_position,[Agent,P]),
%  find_charging_stations(ChargingLocations,P,[]),
%  writeln("2. Finding Oracle locations now..."),
%  find_oracles(OraclesLocations,P,[]),
%  writeln("Finish!!!").

% Find the charging stations (2 in total)
%find_charging_stations(ChargingLocations,P,ChargingLocations_Draft):-
%  length(ChargingLocations_Draft, 2),
%  ChargingLocations = ChargingLocations_Draft.

%solve_task_Astar(Task,[Current|Agenda],RR,Cost,NewPos,Closelist) :-
%find_charging_stations(ChargingLocations,P,ChargingLocations_Draft):-
%  Task = find(c(C)),
%  solve_task_astar(Task,[[c(0,0,P),P]],0,R,Cost,NewPos,[]),
%  map_adjacent(NewPos,ChargingPos,c(C)),
%  \+ memberchk((ChargingPos,c(C)),ChargingLocations_Draft),
%  find_charging_stations(ChargingLocations,NewPos,[(ChargingPos,c(C))|ChargingLocations_Draft]),!.

%Find the oracles locations. (10 in total)
% find_oracles(OraclesLocations,P,OraclesLocations_Draft):-