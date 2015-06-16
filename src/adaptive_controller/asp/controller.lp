% FACTS
#program base.

% problem instance
node(1, 0, 0).
node(2, 0, 0).
node(3, 0, 0).
node(4, 0, 0).
node(5, 0, 0).

edge(1, 5, 1).
edge(1, 2, 1).

edge(2, 1, 1).
edge(2, 3, 1).

edge(3, 2, 1).
edge(3, 4, 1).

edge(4, 3, 1).
edge(4, 5, 1).

edge(5, 4, 1).
edge(5, 1, 1).

holds(at(sBot, 1), 0).

% ROSoClingo settings
id(1..100).
robot(sBot).
value(success; failure).

% actions 
action(moveTo(Room)) :- node(Room, _, _).

% situations
state(at(Robot, Room)) :- node(Room, _, _), robot(Robot).

% context
context(doorClosed(A,B)) :- edge(A, B, _).
#external _context(Context): context(Context).

% TIME RULES
#program transition(t).

% externals
#external _request(ID, State, t): id(ID), state(State).
#external _cancel(ID, t) : id(ID).
#external _value(Robot, Action, Value, t): robot(Robot), action(Action), value(Value).

% situation calculus
{_action(Robot, Action, t): action(Action)}1 :- robot(Robot).
_action(Robot, Action, t) :- _value(Robot, Action, Value, t).
:- _action(Robot, Action, t), not possible(Robot, Action, t), not _value(Robot, Action, success, t).

possible(Robot, moveTo(B), t) :- holds(at(Robot, A), t-1), edge(A, B, _), robot(Robot), not _context(doorClosed(A,B)).

holds(at(Robot, B), t) :- _action(Robot, moveTo(B), t), not _value(Robot, _, failure,t).
holds(at(Robot, B), t) :- holds(at(Robot, B), t-1), not _action(Robot, moveTo(_), t).
holds(at(Robot, B), t) :- holds(at(Robot, B), t-1), _value(Robot, _, failure, t).

holds(request(ID, State), t) :- _request(ID, State, t), not _cancel(ID, t).
holds(request(ID, State), t) :- holds(request(ID, State), t-1), not _cancel(ID, t).

holds(fulfilled(ID), t) :- holds(request(ID, State), t), holds(State, t).
holds(fulfilled(ID), t) :- holds(fulfilled(ID), t-1).

% goals
goal(t) :- holds(fulfilled(ID), t) : holds(request(ID, _), t).
:- _action(_, _, t), goal(t-1), not _request(_, _, t).

#external horizon(t).
:- not goal(t), horizon(t).

% show
#show _action/3.
#show _request/3.
#show _cancel/2.
#show _value/4.
#show _context/1.