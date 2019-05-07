candidate_number(12345).

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% How to make sure pass all part1 scince no tests are given
% A* agenda-based search for part1
% Breadth fisrt search for part3 (shortest path fisrt anwser)
% Always checking energy before moving


% Something could be improved:
% make sure get all marks for part1
% part2 seems fine
% part3 needs to be checked as well
% part4 if we have time
%%%%%%%%%%%%%%%%%%%%%%%%%%%

solve_general(Task,Cost):-
  ( part_module(4) -> solve_task_part4_o(Task,Cost);
    otherwise -> solve_task_o(Task,Cost)
  ).


solve_task(Task,Cost) :-
  b_setval(target_pos,Task),
  (Task = go(_) -> nb_setval(opt, 1);
   otherwise -> nb_setval(opt, 0)),
  my_agent(Agent),
  energy_status(Agent),
  query_world( agent_current_position, [Agent,P] ),
  solve_task_Astar(Task, [[c(0, 0, P), P]], R, Cost, _NewPos, []),!,
  nb_getval(opt, Opt),
  (Opt = 1 -> reverse(R,[_Init|Path]),query_world( agent_do_moves, [Agent,Path] );
  otherwise -> R = [Last|_],solve_task(go(Last),_)).


robustness(_,[_,[]]).
robustness(Task, [Agent, [Head|Tail]]):-
  ( query_world(check_pos, [Head, empty]) ->
    query_world( agent_do_moves, [Agent,[Head]] ), robustness(Task, [Agent, Tail])
  ; otherwise -> write("I'm blocked, need a new path!!"),nl,solve_task_part4_o(Task, _)
  ).

solve_task_part4_o(goto_another_oracle( o(X) ),Cost):-
  my_agent(Agent),
  energy_status(Agent),
  query_world( agent_current_position, [Agent,P] ),
  solve_bfs(goto_another_oracle( o(X) ),[[c(0,0,P),P]],R,Cost,_NewPos,[]),!,
  reverse(R,[_Init|Path]),
  robustness(goto_another_oracle( o(X) ), [Agent, Path])
  ; query_world( agent_do_moves, [Agent,Path] ).



%%%%%%%%%%%%%%%%%%
%% for part3

% checking, set the lowest energy here
energy_status( Agent ) :-
  query_world( agent_current_energy,[Agent,E] ),
  (E>50->true; otherwise->solve_task_o(find(c(_)),_)).


% in case you run out of energy
solve_task_o( find( c(X) ), Cost ) :-
  my_agent( Agent ),
  query_world(agent_current_position,[Agent, P]),
  solve_bfs( find(c(X)), [[c(0, 0, P), P]], R, Cost, _NewPos, []),
  !,
  reverse( R, [_Init | Path] ),
  query_world( agent_do_moves, [Agent,Path] ),
  query_world(agent_topup_energy, [Agent,c(X)] ),
  query_world( agent_current_energy, [Agent,E] ),
  write("Current energy is "),
  write(E),
  nl.


% find next oracle
solve_task_o( goto_another_oracle( o(X) ), Cost ) :-
  my_agent( Agent ),
  energy_status( Agent ),
  query_world(agent_current_position, [Agent, Pos]),
  solve_bfs( goto_another_oracle( o(X) ), [[c( 0, 0, Pos ), Pos]], R, Cost, _NewPos, [] ),
  !,
  reverse( R, [_Init | Path ] ),
  query_world(agent_do_moves,[Agent, Path]).



%%%%%%%%% helper functions
solve_bfs( Task,[Current | _],R,CostsBFS,NewPos,_) :-
  Current = [c(_, G, P) | RPath],
  CostsBFS = [cost( Cost ), depth( G )],
  achieved( Task,[c(G, P) | RPath],R,Cost,NewPos ).


solve_bfs(Task,[Current|Agenda],RR,Cost,NewPos,Visited):-
  Current=[c(_, G, P) | RPath],
  (setof( Child,findChild( P,RPath,G,Child,Visited ),FoundChild) -> append( Agenda,FoundChild,NewAgenda );
  NewAgenda=Agenda),
  delete_seen( NewAgenda,Visited,NewestAgenda ),
  solve_bfs( Task, NewestAgenda, RR, Cost, NewPos, [P | Visited]).


delete_seen(Agenda,[],Agenda).
delete_seen( Agenda,[P | Ps],NewAgenda ) :-
  delete( Agenda,[c(_,_,P) | _],NewAgenda ),
  delete_seen( NewAgenda,Ps, _).


findChild(P,RPath,G,Child,Visited) :-
  search(P,P1,R,_),
  \+ memberchk(R,RPath),
  \+ memberchk(P1,Visited),
  G1 is G+1,
  Child=[c(G1,G1,P1),P1 | RPath].



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
solve_task_Astar(Task,[Current|_],RPath,Cost,NewPos,_):-
  achieved_Astar(Task,Current,RPath,Cost,NewPos).

solve_task_Astar(Task,[Current|Agenda],RR,Cost,NewPos,Closelist) :-
  Current = [c(_,G,P)|RPath],
  G1 is G + 1,
  % add children to the agenda
  (setof([c(F1,G1,Pos1),Pos1|RPath], search_Astar(P,Pos1,F1,G1,Closelist), Children)
  -> merge(Agenda, Children, NewAgenda);
  NewAgenda = Agenda),

  exclude(memberchk(Closelist), NewAgenda, FilteredAgenda),
  solve_task_Astar(Task,FilteredAgenda,RR,Cost,NewPos,[P|Closelist]).

achieved_Astar(go(Exit),Current,RPath,Cost,NewPos) :-
  Current = [c(_,Cost,NewPos)|RPath],
  ( Exit=none -> true
  ; otherwise -> memberchk(Exit, RPath)
  ).

achieved_Astar(find(O),Current,RPath,Cost,NewPos) :-
  Current = [c(_,Cost,NewPos)|RPath],
  ( O=none    -> true
  ; otherwise -> memberchk(Last, RPath),map_adjacent(Last,_,O)
  ),
  otherwise -> true.


cal_manhattan_dis(Pos, Goal, Dist):-
  Pos = p(X_0,Y_0),
  Goal = p(X_1,Y_1),
  Dist is abs(X_0-X_1) + abs(Y_0-Y_1).

search_Astar(P,N,F1,G1,Closelist):-
  map_adjacent(P,N,empty),
  \+ memberchk(N,Closelist),
  nb_getval(opt, Opt),

  (Opt = 1 -> b_getval(target_pos, go(Goal)),
  cal_manhattan_dis(N, Goal, Dist),
  H is Dist;
  otherwise -> H is 0),
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
  ; otherwise -> RPath = [Last|_],map_adjacent(Last,_,O)
  ).


%%% for part3

achieved(goto_another_oracle(O),Current,RPath,Cost,NewPos) :-
  my_agent(Agent),
  Current = [c(Cost,NewPos)|RPath],
  ( O=none    -> true
  ; otherwise -> RPath = [Last|_], map_adjacent(Last,_,O),
  \+ query_world(agent_check_oracle,[Agent, O] ),
  write("Visited oracle "),
  write(O),
  nl ).


search(F,N,N,1) :-
  map_adjacent(F,N,empty).
