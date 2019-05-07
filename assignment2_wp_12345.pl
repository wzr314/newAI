candidate_number(12345).

% Find hidden identity by repeatedly calling agent_ask_oracle(oscar,o(1),link,L)
% find_identity(-A)


find_identity(A):-
  (part_module(2)   -> find_identity_2(A)
  ; otherwise -> find_identity_o(A)
  ).


find_identity_2( A ):-
  findall( Actor, actor( Actor ), ListA ),
  find_actor_wiki( A, ListA ).


find_actor_wiki( A, ListA ) :-
  agent_ask_oracle( oscar, o(1), link, L ),
  include( check_links(L), ListA, FilterL ),

  length( FilterL, Size ),
  (Size = 1->nth0( 0, FilterL, A ); find_actor_wiki( A, FilterL )).


check_links(Link, A) :-
  actor( A ),
  setof(Link, ( link( Link ), wp(A, WT), wt_link( WT, Link) ), Links ),
  member( Link, Links ).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Part 3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate the actor links, find the location of all charing stations and oracles.
% P: Position; L:List.
%%

find_identity_o(A):-
  findall(Actor, actor(Actor), Actors),
  find_actor_identity_o(A, Actors).


find_actor_identity_o(A, Actors) :-
  solve_task_o(find_next_oracle(o(X)), _),
  my_agent(Agent),
  query_world( agent_ask_oracle, [Agent, o(X), link, L]),
  include(check_links( L ), Actors, NewActors),
  length(NewActors, Length),
  ( Length = 1 -> nth0(0, NewActors, A); find_actor_identity_o(A, NewActors)).


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
