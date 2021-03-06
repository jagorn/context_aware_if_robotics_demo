%------ FACTS -------%
#program base.

% settings
robot(robot0).
id(1..100).
value(success; failure).

% actions 
action(goTo(X, Y)) :- xCoord(X), yCoord(Y).

% situations
state(at(Robot, Region)) :- region(Region, _, _, _, _, _), robot(Robot).
state(at(Robot, Item)) :- item(Item, _, _), robot(Robot).

% context
#external _context(Context): context(Context).

%------- TIME RULES --------%
#program transition(t).

% externals
#external _request(ID, State, t): id(ID), state(State).
#external _cancel(ID, t) : id(ID).
#external _value(Robot, Action, Value, t): robot(Robot), action(Action), value(Value).

% situation calculus
{_action(Robot, Action, t): action(Action)}1 :- robot(Robot).
_action(Robot, Action, t) :- _value(Robot, Action, success, t).
:- _action(Robot, Action, t), not possible(Robot, Action, t), not _value(Robot, Action, success, t).

possible(Robot, goTo(X, Y), t) :- robot(Robot), xCoord(X), yCoord(Y).

holds(at(Robot, B), t) :- item(B, X, Y), _action(Robot, goTo(X, Y), t), not _value(Robot, _, failure,t).
holds(at(Robot, B), t) :- region(B, _, X, Y, _, _), _action(Robot, goTo(X, Y), t), not _value(Robot, _, failure,t).
holds(at(Robot, B), t) :- holds(at(Robot, B), t-1), not _action(Robot, goTo(_), t).
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