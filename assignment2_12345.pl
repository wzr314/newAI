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


% run different tasks based on which part you are going
solve_task_general(Task,Cost) :- 
  part_module(1) -> solve_task(Task,Cost),!;
  otherwise -> solve_task_o(Task,Cost).


solve_task(Task,Cost) :-
  b_setval(target_pos,Task),
  (Task = go(_) -> nb_setval(opt, 1);
   otherwise -> nb_setval(opt, 0)),
  my_agent(Agent),
  checking(Agent),
  query_world( agent_current_position, [Agent,P] ),
  solve_task_Astar(Task, [[c(0,0,P),P]],R,Cost,_NewPos,[]),!,
  nb_getval(opt, Opt),
  (Opt = 1 -> reverse(R,[_Init|Path]),query_world( agent_do_moves, [Agent,Path] );
  otherwise -> R = [Last|_],solve_task(go(Last),_)).



%%%%%%%%%%%%%%%%%%
%%% for part3

% checking, set the lowest energy here
checking(Agent):-
  query_world(agent_current_energy, [Agent, Energy]),
  ( Energy > 50 -> true
  ; otherwise -> solve_task_engergy(find(c(_)), _)
  ).


% find next oracle
solve_task_o(find_next_oracle(o(X)),Cost):-
  my_agent(Agent),
  checking(Agent),
  query_world( agent_current_position, [Agent,P] ),
  solve_task_bfs(find_next_oracle(o(X)),[[c(0,0,P),P]],R,Cost,_NewPos,[]),!
  ,
  reverse(R,[_Init|Path]),
  otherwise -> query_world( agent_do_moves, [Agent,Path] )
  .



% in case if you run out of energy
solve_task_engergy(find(c(X)),Cost):-
  my_agent(Agent),
  query_world( agent_current_position, [Agent,P] ),
  solve_task_bfs(find(c(X)),[[c(0,0,P),P]],R,Cost,_NewPos,[]),!
  ,
  reverse(R,[_Init|Path]),
  otherwise -> query_world( agent_do_moves, [Agent,Path] )
  ,
  write("topup energy now"),nl,
  query_world(agent_topup_energy, [Agent, c(X)]),
  query_world(agent_current_energy, [Agent, Energy]),
  write("ENERGY = "), write(Energy),nl.


%%%%%%%%% helper functions
solve_task_bfs(Task, [Current|_], R, Costs, NewPos, _):-
  Current = [c(_,G,P)|RPath],
  Costs = [cost(Cost), depth(G)],
  achieved(Task, [c(G,P)|RPath], R, Cost, NewPos).

solve_task_bfs(Task, [Current|Agenda], RR, Cost, NewPos, Visited):-
  Current = [c(_,G,P)|RPath],
  (setof(Child, find_child(P, RPath, G, Child, Visited), Children) -> append(Agenda, Children, NewAgenda)
  ; NewAgenda = Agenda
  ),
  remove_visited(NewAgenda, Visited, FinalAgenda),
  solve_task_bfs(Task,FinalAgenda,RR,Cost,NewPos,[P|Visited]).

remove_visited(Agenda, [], Agenda).
remove_visited(Agenda, [P|Tail], NewAgenda) :-
  delete(Agenda, [c(_,_,P)|_], NewAgenda),
  remove_visited(NewAgenda, Tail, _).


find_child(P, RPath, G, Child, Visited) :-
  search(P, P1, R, _),
  \+ memberchk(R, RPath),
  \+ memberchk(P1, Visited),
  G1 is G+1,
  Child = [c(G1, G1, P1), P1|RPath].




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


%%%% Agenda based A star %%%%
solve_task_Astar(Task,[Current|_],RPath,Cost,NewPos,_):-
  achieved_Astar(Task,Current,RPath,Cost,NewPos).

solve_task_Astar(Task,[Current|Agenda],RR,Cost,NewPos,Closelist) :-
  Current = [c(_,G,P)|RPath],
  G1 is G + 1,

  % add children to the adenda
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


manhattan(Pos, Goal, D):-
  Pos = p(X1,Y2),
  Goal = p(X2,Y2),
  D is abs(X1-X2) + abs(Y1-Y2).

search_Astar(P,N,F1,G1,Closelist):-
  map_adjacent(P,N,empty),
  \+ memberchk(N,Closelist),
  nb_getval(opt, Opt),

  (Opt = 1 -> b_getval(target_pos, go(Goal)),
  manhattan(N, Goal, D),
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


achieved(find_next_oracle(O), Current, RPath, Cost, NewPos) :-
  Current = [c(Cost, NewPos)|RPath],
  my_agent(Agent),
  ( O=none    -> true
  ; otherwise -> RPath = [Last|_], map_adjacent(Last,_,O),
  \+ query_world(agent_check_oracle, [Agent, O ]), write("ORACLE FOUND, I = "), write(O), nl
  ).

search(F,N,N,1) :-
  map_adjacent(F,N,empty).
