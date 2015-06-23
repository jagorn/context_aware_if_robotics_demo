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

crowding(Room, 5) :- time(morning)
crowding(Room, 3) :- time(noon)
crowding(Room, 2) :- time(afternoon)
crowding(Room, 0) :- time(night), room(Room, Room


#external _time(T): time(T).
#external _light(L): light(L).
#external
