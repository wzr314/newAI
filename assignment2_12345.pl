candidate_number(12345).

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% How to make sure pass all part1 scince no tests are given
% A* agenda-based search for part1
% Breadth first search for part3 (shortest path first anwser)
% Always checking energy before moving


% Some that things could be improved:
% make sure get all marks for part1
% part2 seems fine
% part3 needs to be checked as well
% part4 if we have time
%%%%%%%%%%%%%%%%%%%%%%%%%%%%


solve_general( Task, Cost ) :-
  ( part_module(4) -> solve_task_part4_o( Task,Cost ); otherwise -> solve_task_o( Task, Cost ) ).


solve_task( Task, Cost ) :-
  b_setval(destination,Task),
  ( Task = go(_) -> nb_setval(flag, 1);
   otherwise -> nb_setval(flag, 0)
  ),
  my_agent(A),
  energy_status(A), % cheking energy
  query_world( agent_current_position, [A,P] ),

  solve_task_Astar(Task, [[c(0, 0, P), P]], R, Cost, _NewPos, []),!,
  nb_getval(flag, Flag),
  ( Flag = 1 -> reverse( R, [_Init | Path]),
    query_world( agent_do_moves, [A,Path] );
    otherwise -> R = [Last|_], solve_task( go(Last),_) ).


solve_task_part4_o( goto_another_oracle( o(X) ), Cost ) :-
  my_agent( A ),
  % see if needs to top up
  energy_status( A ),
  % retrieve the position of the agent
  query_world( agent_current_position, [A, Pos]),
  % find unvisited oracle
  go_bfs( goto_another_oracle( o(X) ), [[c( 0, 0, Pos ), Pos]], R, Cost, _NewPos, []),
  !,
  reverse(R, [_Init | Route] ),
  % going to unvisited oracle
  robustness( goto_another_oracle( o(X) ), [A, Route] ) ; query_world( agent_do_moves, [A, Route]).


% robustness search for part4
robustness( _, [_, [] ] ).
robustness( Task, [Agent, [Pos|ListMoves] ] ) :-
  % move each step while search a new path
  (query_world( check_pos, [Pos, empty] ) -> query_world(agent_do_moves, [Agent, [Pos] ]), robustness( Task, [Agent, ListMoves] );
  otherwise -> solve_task_part4_o( Task, _ ) ). % blocked


%%%%%%%%%%%%%%%%%%
%% for part3 %%

% set the lowest allowed energy here
energy_status( A ) :-
  query_world( agent_current_energy,[A, E] ),
  % set energy threshold here
  (E>50 -> true ; otherwise -> solve_task_o( find( c(_) ), _) ).   % if lower than 50, go charge first


% in case you run out of energy
solve_task_o( find( c(X) ), Cost ) :-
  my_agent( Agent ),
  % retrieve the position of the agent
  query_world(agent_current_position,[Agent, Pos]),
  % find a charging station using bfs
  go_bfs( find( c(X) ), [[c(0, 0, Pos), Pos]], R, Cost, _NewPos, []),
  !,
  reverse( R, [_Init | Route] ),
  % go to top up
  query_world( agent_do_moves, [Agent,Route] ),
  % agent increases energy level
  query_world(agent_topup_energy, [Agent,c(X)] ).


% find another oracle that has not been seen yet
solve_task_o( goto_another_oracle( o(X) ), Cost ) :-
  my_agent( Agent ),
  % see if needs to top up
  energy_status( Agent ),
  % retrieve the position of the agent
  query_world(agent_current_position, [Agent, Pos]),
  % find unvisited oracle
  go_bfs( goto_another_oracle( o(X) ), [[c( 0, 0, Pos ), Pos]], R, Cost, _NewPos, [] ),
  !,
  reverse( R, [_Init | Route ] ),
  % going to unvisited oracle
  query_world(agent_do_moves,[Agent, Route]).



%%%%%%%%% helper functions bfs
go_bfs( Task,[Curr | _],R,CostsBFS,NewPos,_) :-
  Curr=[c(_, G, P) | RPath],
  % check the costs
  CostsBFS=[cost( Cost ), depth( G )],
  achieved( Task, [c(G, P) | RPath], R, Cost, NewPos ).


go_bfs(Task,[Curr | Rest],RR,Cost,NewPos,Explored) :-
  Curr=[c(_, G, P) | RPath],
  (setof( Connections,find_connected( P,RPath,G,Connections,Explored ),ResultList) -> append( Rest,ResultList,ModifRest ); ModifRest=Rest),
  delete_seen( ModifRest,Explored,NewestRest ),
  go_bfs( Task, NewestRest, RR, Cost, NewPos, [P | Explored]).


find_connected(P,RPath,G,Connections,Explored) :-
  search(P,P1,R,_),
  \+ memberchk(R,RPath),    % check we have not been here already
  \+ memberchk(P1,Explored),
  G1 is G+1,
  % list of connections
  Connections=[c(G1,G1,P1), P1 | RPath].


delete_seen(Rest,[],Rest).
delete_seen( Rest,[P | Ps],ModifRest ) :-
  delete( Rest,[c(_,_,P) | _],ModifRest ),
  delete_seen( ModifRest,Ps, _).



%%%%%%%%%% Useful predicates %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% backtracking depth-first search, needs to be changed to agenda-based A*

solve_task_bt(Task,Current,Depth,RPath,[cost(Cost),depth(Depth)],NewPos) :-
  achieved(Task,Current,RPath,Cost,NewPos).

solve_task_bt(Task,Current,D,RR,Cost,NewPos) :-
  Current = [c(F,P)|RPath],
  search(P,P1,R,C),
  \+ memberchk(R,RPath),  % check we have not been here already
  D1 is D+1,
  F1 is F+C,
  solve_task_bt(Task,[c(F1,P1),R|RPath],D1,RR,Cost,NewPos).  % backtrack search


%%%% Agenda based A star %%%%%%%%%%%%%%%%%%%%%
solve_task_Astar(Task, [Current|_], RPath, Cost, NewPos, _):-
  achieved_Astar(Task,Current,RPath,Cost,NewPos).

solve_task_Astar(Task,[Current|Agenda],RR,Cost,NewPos,Closest) :-
  Current = [c(_, G,P)|RPath], G1 is G + 1,
  % add children to the agenda
  (setof([c(F1,G1,Pos1),Pos1|RPath],
   search_Astar(P,Pos1,F1,G1,Closest), Children) % search adjacent
  -> merge(Agenda, Children, NewAgenda);NewAgenda = Agenda),
  exclude( memberchk(Closest), NewAgenda, FinalAgenda), % filter
  solve_task_Astar(Task, FinalAgenda, RR, Cost, NewPos, [P | Closest]).


achieved_Astar(go(Exit),Current,RPath,Cost,NewPos) :-
  Current = [c(_,Cost,NewPos)|RPath],
  ( Exit=none -> true
  ; otherwise -> memberchk(Exit, RPath)
  ).

achieved_Astar(find(O),Current,RPath,Cost,NewPos) :-
  Current = [c(_,Cost,NewPos)|RPath],
  ( O=none    -> true
  ; otherwise -> memberchk(Last, RPath),
  map_adjacent( Last, _, O)
  ),
  otherwise -> true.


manhattan_distance(Pos, Goal, Distance):-
  Pos = p(X1,Y1), Goal = p(X2,Y2),
  Distance is abs(X1-X2) + abs(Y1-Y2).


search_Astar(P,N,F1,G1,Closest):-
  map_adjacent(P,N,empty), \+ memberchk(N, Closest),
  nb_getval(flag, Flag),
  ( Flag = 1 -> b_getval( destination, go( Goal ) ),
    manhattan_distance( N,Goal,Distance ), H is Distance;
    otherwise -> H is 0
  ),
  F1 is H + G1.


%%%%%%%%%%%%% achieved %%%%%%%%%%

achieved(go(Exit),Current,RPath,Cost,NewPos) :-
  Current = [c(Cost,NewPos)|RPath],
  ( Exit=none -> true
  ; otherwise -> RPath = [Exit|_]
  ).


achieved(find(O),Current,RPath,Cost,NewPos) :-
  Current = [c(Cost,NewPos)|RPath],
  ( O=none    -> true
  ; otherwise -> RPath = [Last|_], map_adjacent(Last,_,O)
  ).


%%%  for part3  %%%

achieved( goto_another_oracle( O ),Curr,RPath,Cost,NewPos ) :-
  Curr = [c( Cost, NewPos ) | RPath],
  my_agent( Agent ),
  ( O=none    -> true
  ; otherwise -> RPath = [Last|_], map_adjacent(Last,_,O),
  \+ query_world( agent_check_oracle,[Agent,O] ),
  % printing
  write("Visiting oracle "),
  write( O ),
  nl ).


search(F,N,N,1) :-
  map_adjacent(F,N,empty).
