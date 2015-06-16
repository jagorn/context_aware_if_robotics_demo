#program base.

#const id = 5.
#const job = 5.

robot(fulfill).
action(1..job).
value(success).
value(failue).


#program transition(t).

#external _request(1..id,1..job,t).
#external _cancel(1..id,t).
#external _value(fulfill,Value,t):value(Value).

{_action(Robot,Action,t):action(Action)}1 :- robot(Robot).
:- _action(Robot,Action,t), not possible(Robot,Action,t).

possible(fulfill,Job,t) :- action(Job).

holds(request(ID,Job),t) :- _request(ID,Job,t).
holds(fulfilled(ID),t) :- _cancel(ID,t).
holds(fulfilled(ID),t) :- holds(request(ID,Job),t), _action(Robot,Job,t), not _value(Robot,failure,t).

holds(Fluent,t) :- holds(Fluent,t-1), not abnormal(Fluent,t).

goal(t) :- holds(fulfilled(ID),t) : holds(request(ID,_),t).

#external horizon(t).

:- not goal(t), horizon(t).

