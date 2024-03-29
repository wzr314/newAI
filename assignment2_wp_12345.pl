candidate_number(12345).

% Find hidden identity by repeatedly calling agent_ask_oracle(oscar,o(1),link,L)
% find_identity(-A)


find_identity(A) :-
  (part_module(2)   -> find_identity_2(A)
  ; otherwise -> find_identity_o(A)
  ).


find_identity_2( A ) :-
  findall( A,actor( A ),ListA ),
  % another function to call bkt
  find_wiki_actor( A,ListA ).


find_wiki_actor( A,ListA ) :-
  agent_ask_oracle( oscar,o(1),link,L ),
  % get the list of actors using links
  include( checking_links(L),ListA,FilterL ),
  % get size of the new list
  length( FilterL,Size ),
  % checking all elements of the filtered list until we find a result
  (Size=1->nth0( 0, FilterL, A ); find_wiki_actor( A, FilterL )).


checking_links( L,A ) :-
  actor( A ),
  % sort the result
  setof(L,(link( L ),wp( A, WT ),wt_link( WT, L )),Ls),
  % check if L is a member of Ls
  member( L, Ls ).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% For part3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% just draft
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

% Generate the actor links, find the location of all charing stations and oracles.
% P: Position; L:List.
%%


find_identity_o( A ) :-
  findall( A,actor( A ),ListA),
  % another function to call bkt
  find_wiki_actor_o( A,ListA ).


find_wiki_actor_o( A,ListA ) :-
  % integrate part4
  solve_general( goto_another_oracle( o(X) ), _ ),
  my_agent( Agent ),
  % trying to find the identity
  query_world(agent_ask_oracle,[Agent,o(X),link,L] ),
  % get the list of actors using links
  include( checking_links( L ),ListA,FilterL ),
  % get size of the filtered list
  length( FilterL,Size),
  % checking all elements of the filtered list until we find a result
  (Size=1 ->nth0(0,FilterL,A);
  find_wiki_actor_o( A,FilterL ) ).
