#program base.

% Facts
room_type(cafeteria).
room_type(corridor).
room_type(office).
prob(1..10).

% Context Variables
time(morning).
time(noon).
time(afternoon).
time(night).

light(dark).
light(bright).

% Context Rules
{_action(Robot, Action): action(Action)}1 :- robot(Robot).

crowding(Room, 5) :- _time(morning)
crowding(Room, 3) :- _time(noon)
crowding(Room, 2) :- _time(afternoon)
crowding(Room, 0) :- _time(night)

context_area(circle, scaled, X, Y, Range, Weight) :- person_at(X,Y), range(T, Range), _time(T), person.
context_area(rect, homogeneous, DownLeftX, DownLeftY, UpRightX, UpRightY, Weight).

#external _time(T): time(T).
#external _light(L): light(L).
#external _person_at(X,Y).

#show context_area/5.